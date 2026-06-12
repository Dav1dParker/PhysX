#include "PxPhysicsAPI.h"
#include "deleters.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

using namespace physx;

static PxDefaultAllocator gAllocator;
static PxDefaultErrorCallback gErrorCallback;

static PxFoundationPtr gFoundation;
static PxPhysicsPtr gPhysics;
static PxScenePtr gScene;
static PxDefaultCpuDispatcherPtr gDispatcher;
static PxPvdPtr gPvd;
static PxPvdTransportPtr gTransport;

static PxMaterialPtr gGroundMaterial;
static PxMaterialPtr gPoleMaterial;
static PxMaterialPtr gClothMaterial;
static PxMaterialPtr gPinnedMaterial;
static PxMaterialPtr gLinkMaterial;

static constexpr float TIME_STEP = 1.0f / 60.0f;
static constexpr int FLAG_COLUMNS = 12;
static constexpr int FLAG_ROWS = 8;
static constexpr float FLAG_WIDTH = 4.0f;
static constexpr float FLAG_HEIGHT = 2.4f;
static constexpr float FLAG_TOP_Y = 3.2f;
static constexpr float FLAG_Z = 0.0f;
static constexpr float VERTEX_RADIUS = 0.035f;
static constexpr float LINK_RADIUS = 0.012f;
static constexpr float CLOTH_DAMPING = 0.992f;
static constexpr float CLOTH_MASS = 0.08f;
static constexpr int SOLVER_ITERATIONS = 10;

struct ClothVertex
{
    PxVec3 position;
    PxVec3 previousPosition;
    PxVec3 acceleration;
    PxVec3 pinnedPosition;
    bool pinned = false;
    PxRigidDynamicPtr visual;
};

struct ClothLink
{
    int a = 0;
    int b = 0;
    float restLength = 0.0f;
    PxRigidDynamicPtr visual;
};

static std::vector<PxRigidStaticPtr> gStaticActors;
static std::vector<ClothVertex> gVertices;
static std::vector<ClothLink> gLinks;
static float gSimulationTime = 0.0f;

static int vertexIndex(int row, int column)
{
    return row * FLAG_COLUMNS + column;
}

static PxQuat rotationFromXAxisToDirection(const PxVec3& direction)
{
    PxVec3 xAxis(1.0f, 0.0f, 0.0f);
    PxVec3 unitDirection = direction.getNormalized();
    float dot = PxClamp(xAxis.dot(unitDirection), -1.0f, 1.0f);

    if (dot > 0.9999f)
    {
        return PxQuat(PxIdentity);
    }

    if (dot < -0.9999f)
    {
        return PxQuat(PxPi, PxVec3(0.0f, 1.0f, 0.0f));
    }

    PxVec3 axis = xAxis.cross(unitDirection).getNormalized();
    return PxQuat(std::acos(dot), axis);
}

static PxRigidStaticPtr createStaticBox(const PxVec3& position, const PxVec3& halfSize, PxMaterial& material)
{
    PxRigidStaticPtr actor(gPhysics->createRigidStatic(PxTransform(position)));
    PxRigidActorExt::createExclusiveShape(*actor, PxBoxGeometry(halfSize), material);
    gScene->addActor(*actor);
    return actor;
}

static PxRigidDynamicPtr createKinematicSphere(const PxVec3& position, float radius, PxMaterial& material)
{
    PxRigidDynamicPtr actor(gPhysics->createRigidDynamic(PxTransform(position)));
    PxRigidActorExt::createExclusiveShape(*actor, PxSphereGeometry(radius), material);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
    gScene->addActor(*actor);
    return actor;
}

static PxRigidDynamicPtr createKinematicCapsule(const PxVec3& a, const PxVec3& b, float radius, PxMaterial& material)
{
    PxVec3 delta = b - a;
    float length = std::max(delta.magnitude(), 0.001f);
    PxTransform transform((a + b) * 0.5f, rotationFromXAxisToDirection(delta));

    PxRigidDynamicPtr actor(gPhysics->createRigidDynamic(transform));
    PxRigidActorExt::createExclusiveShape(*actor, PxCapsuleGeometry(radius, length * 0.5f), material);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
    actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
    gScene->addActor(*actor);
    return actor;
}

static void updateKinematicCapsule(PxRigidDynamic& actor, const PxVec3& a, const PxVec3& b)
{
    PxVec3 delta = b - a;
    float length = std::max(delta.magnitude(), 0.001f);

    PxShape* shape = nullptr;
    if (actor.getShapes(&shape, 1) == 1 && shape)
    {
        shape->setGeometry(PxCapsuleGeometry(LINK_RADIUS, length * 0.5f));
    }

    actor.setKinematicTarget(PxTransform((a + b) * 0.5f, rotationFromXAxisToDirection(delta)));
}

static void addLink(int a, int b)
{
    ClothLink link;
    link.a = a;
    link.b = b;
    link.restLength = (gVertices[b].position - gVertices[a].position).magnitude();
    link.visual = createKinematicCapsule(gVertices[a].position, gVertices[b].position, LINK_RADIUS, *gLinkMaterial);
    gLinks.push_back(std::move(link));
}

static PxVec3 calculateWind(float time)
{
    float horizontalAngle = 0.85f * std::sin(time * 0.55f) + 0.35f * std::sin(time * 1.35f);
    float strength = 4.5f + 2.2f * std::sin(time * 0.9f) + 1.1f * std::sin(time * 2.1f);
    strength = std::max(strength, 0.8f);

    return PxVec3(std::cos(horizontalAngle), 0.12f * std::sin(time * 1.7f), std::sin(horizontalAngle)) * strength;
}

static void integrateCloth(float dt)
{
    PxVec3 wind = calculateWind(gSimulationTime);

    for (int row = 0; row < FLAG_ROWS; ++row)
    {
        for (int column = 0; column < FLAG_COLUMNS; ++column)
        {
            ClothVertex& vertex = gVertices[vertexIndex(row, column)];

            if (vertex.pinned)
            {
                vertex.position = vertex.pinnedPosition;
                vertex.previousPosition = vertex.pinnedPosition;
                continue;
            }

            float wave = std::sin(gSimulationTime * 3.0f + column * 0.7f + row * 0.25f);
            vertex.acceleration = PxVec3(0.0f, -9.81f, 0.0f);
            vertex.acceleration += (wind * (1.0f + 0.18f * wave)) / CLOTH_MASS;

            PxVec3 velocity = (vertex.position - vertex.previousPosition) * CLOTH_DAMPING;
            PxVec3 nextPosition = vertex.position + velocity + vertex.acceleration * dt * dt;

            vertex.previousPosition = vertex.position;
            vertex.position = nextPosition;
        }
    }
}

static void satisfyConstraints()
{
    for (int iteration = 0; iteration < SOLVER_ITERATIONS; ++iteration)
    {
        for (const ClothLink& link : gLinks)
        {
            ClothVertex& a = gVertices[link.a];
            ClothVertex& b = gVertices[link.b];

            PxVec3 delta = b.position - a.position;
            float length = delta.magnitude();
            if (length < 0.0001f)
            {
                continue;
            }

            PxVec3 correction = delta * ((length - link.restLength) / length);

            if (!a.pinned && !b.pinned)
            {
                a.position += correction * 0.5f;
                b.position -= correction * 0.5f;
            }
            else if (a.pinned && !b.pinned)
            {
                b.position -= correction;
            }
            else if (!a.pinned && b.pinned)
            {
                a.position += correction;
            }
        }

        for (ClothVertex& vertex : gVertices)
        {
            if (vertex.pinned)
            {
                vertex.position = vertex.pinnedPosition;
            }
        }
    }
}

static void updateClothVisuals()
{
    for (ClothVertex& vertex : gVertices)
    {
        vertex.visual->setKinematicTarget(PxTransform(vertex.position));
    }

    for (ClothLink& link : gLinks)
    {
        updateKinematicCapsule(*link.visual, gVertices[link.a].position, gVertices[link.b].position);
    }
}

static void createBannerCloth()
{
    gVertices.clear();
    gLinks.clear();

    gVertices.reserve(FLAG_COLUMNS * FLAG_ROWS);

    float leftX = -FLAG_WIDTH * 0.5f;
    float dx = FLAG_WIDTH / static_cast<float>(FLAG_COLUMNS - 1);
    float dy = FLAG_HEIGHT / static_cast<float>(FLAG_ROWS - 1);

    for (int row = 0; row < FLAG_ROWS; ++row)
    {
        for (int column = 0; column < FLAG_COLUMNS; ++column)
        {
            PxVec3 position(leftX + column * dx, FLAG_TOP_Y - row * dy, FLAG_Z);

            ClothVertex vertex;
            vertex.position = position;
            vertex.previousPosition = position;
            vertex.pinnedPosition = position;
            vertex.pinned = (row == 0 && (column == 0 || column == FLAG_COLUMNS - 1));
            vertex.visual = createKinematicSphere(
                position,
                vertex.pinned ? VERTEX_RADIUS * 1.7f : VERTEX_RADIUS,
                vertex.pinned ? *gPinnedMaterial : *gClothMaterial
            );

            gVertices.push_back(std::move(vertex));
        }
    }

    for (int row = 0; row < FLAG_ROWS; ++row)
    {
        for (int column = 0; column < FLAG_COLUMNS; ++column)
        {
            int current = vertexIndex(row, column);

            if (column + 1 < FLAG_COLUMNS)
            {
                addLink(current, vertexIndex(row, column + 1));
            }
            if (row + 1 < FLAG_ROWS)
            {
                addLink(current, vertexIndex(row + 1, column));
            }
            if (column + 1 < FLAG_COLUMNS && row + 1 < FLAG_ROWS)
            {
                addLink(current, vertexIndex(row + 1, column + 1));
            }
            if (column > 0 && row + 1 < FLAG_ROWS)
            {
                addLink(current, vertexIndex(row + 1, column - 1));
            }
            if (column + 2 < FLAG_COLUMNS)
            {
                addLink(current, vertexIndex(row, column + 2));
            }
            if (row + 2 < FLAG_ROWS)
            {
                addLink(current, vertexIndex(row + 2, column));
            }
        }
    }
}

static void createScene()
{
    gStaticActors.clear();

    gStaticActors.push_back(createStaticBox(PxVec3(0.0f, -0.05f, 0.0f), PxVec3(4.0f, 0.05f, 2.5f), *gGroundMaterial));

    float poleHeight = FLAG_TOP_Y + 0.25f;
    gStaticActors.push_back(createStaticBox(PxVec3(-FLAG_WIDTH * 0.5f, poleHeight * 0.5f, -0.08f), PxVec3(0.04f, poleHeight * 0.5f, 0.04f), *gPoleMaterial));
    gStaticActors.push_back(createStaticBox(PxVec3(FLAG_WIDTH * 0.5f, poleHeight * 0.5f, -0.08f), PxVec3(0.04f, poleHeight * 0.5f, 0.04f), *gPoleMaterial));
    gStaticActors.push_back(createStaticBox(PxVec3(0.0f, FLAG_TOP_Y, -0.08f), PxVec3(FLAG_WIDTH * 0.5f + 0.08f, 0.035f, 0.035f), *gPoleMaterial));

    createBannerCloth();
}

static void updateScene(float dt)
{
    gSimulationTime += dt;
    integrateCloth(dt);
    satisfyConstraints();
    updateClothVisuals();
}

static void printStatus(int frame)
{
    if (frame % 30 != 0)
    {
        return;
    }

    PxVec3 wind = calculateWind(gSimulationTime);
    std::cout << std::fixed << std::setprecision(2)
              << "t=" << gSimulationTime
              << "s | vertices=" << gVertices.size()
              << " | pinned=2"
              << " | wind=(" << wind.x << ", " << wind.y << ", " << wind.z << ")"
              << " | strength=" << wind.magnitude()
              << '\n';
}

static void initPhysX()
{
    gFoundation.reset(PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback));

    gPvd.reset(PxCreatePvd(*gFoundation));
    gTransport.reset(PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10));

    if (gPvd && gTransport)
    {
        gPvd->connect(*gTransport, PxPvdInstrumentationFlag::eALL);
    }

    PxTolerancesScale scale;
    gPhysics.reset(PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, scale, true, gPvd.get()));
    PxInitExtensions(*gPhysics, gPvd.get());

    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

    gDispatcher.reset(PxDefaultCpuDispatcherCreate(2));
    sceneDesc.cpuDispatcher = gDispatcher.get();
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;

    gScene.reset(gPhysics->createScene(sceneDesc));

    PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
    if (pvdClient)
    {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    gGroundMaterial.reset(gPhysics->createMaterial(0.7f, 0.7f, 0.1f));
    gPoleMaterial.reset(gPhysics->createMaterial(0.5f, 0.5f, 0.2f));
    gClothMaterial.reset(gPhysics->createMaterial(0.25f, 0.25f, 0.05f));
    gPinnedMaterial.reset(gPhysics->createMaterial(0.25f, 0.25f, 0.05f));
    gLinkMaterial.reset(gPhysics->createMaterial(0.25f, 0.25f, 0.05f));
}

static void cleanupPhysX()
{
    gLinks.clear();
    gVertices.clear();
    gStaticActors.clear();

    gGroundMaterial.reset();
    gPoleMaterial.reset();
    gClothMaterial.reset();
    gPinnedMaterial.reset();
    gLinkMaterial.reset();

    gScene.reset();
    gDispatcher.reset();

    PxCloseExtensions();
    gPhysics.reset();

    if (gPvd)
    {
        gPvd->disconnect();
    }

    gPvd.reset();
    gTransport.reset();
    gFoundation.reset();
}

int main()
{
    initPhysX();
    createScene();

    std::cout << "PhysX banner cloth simulation started.\n";
    std::cout << "Open PhysX Visual Debugger and connect to 127.0.0.1:5425.\n";
    std::cout << "Flag mesh: " << FLAG_COLUMNS << " x " << FLAG_ROWS << " = "
              << FLAG_COLUMNS * FLAG_ROWS << " vertices. Top left and top right vertices are pinned.\n";

    for (int frame = 0; frame < 1800; ++frame)
    {
        updateScene(TIME_STEP);
        gScene->simulate(TIME_STEP);
        gScene->fetchResults(true);
        printStatus(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    cleanupPhysX();
    return 0;
}
