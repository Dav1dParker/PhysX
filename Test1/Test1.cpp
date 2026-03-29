#include "PxPhysicsAPI.h"
#include "deleters.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace physx;

static PxDefaultAllocator      gAllocator;
static PxDefaultErrorCallback  gErrorCallback;
static PxFoundationPtr gFoundation;
static PxPhysicsPtr gPhysics;
static PxScenePtr gScene;
static PxDefaultCpuDispatcherPtr gDispatcher;
static PxPvdPtr gPvd;
static PxPvdTransportPtr gTransport;

static PxMaterialPtr gTableMaterial;
static PxMaterialPtr gBallMaterial;
static PxMaterialPtr gRailMaterial;
static PxMaterialPtr gCueMaterial;

static constexpr float TABLE_LENGTH = 2.54f;
static constexpr float TABLE_WIDTH = 1.27f;

static constexpr float BALL_RADIUS = 0.028575f;
static constexpr float BALL_MASS = 0.17f;

static constexpr float RAIL_HEIGHT = 0.12f;
static constexpr float RAIL_THICKNESS = 0.08f;

static constexpr float POCKET_RADIUS = 0.06f;

static constexpr float TIME_STEP = 1.0f / 60.0f;

struct Ball
{
    PxRigidDynamicPtr actor;
    int number = 0;
    bool cue = false;
    bool pocketed = false;
};

static std::vector<Ball> gBalls;
static std::vector<PxRigidStaticPtr> gStaticActors;
static std::vector<PxVec3> gPocketPositions;
static std::vector<std::string> gActorNames;

static const char* storeName(const std::string& value)
{
    gActorNames.push_back(value);
    return gActorNames.back().c_str();
}

static PxRigidStaticPtr createStaticBox(
    const PxVec3& position,
    const PxVec3& halfExtents,
    PxMaterial& material,
    const char* name)
{
    PxRigidStaticPtr actor(gPhysics->createRigidStatic(PxTransform(position)));
    PxRigidActorExt::createExclusiveShape(*actor, PxBoxGeometry(halfExtents), material);
    actor->setName(name);
    gScene->addActor(*actor);
    return actor;
}

static void createPocketMarker(const PxVec3& position, int index)
{
    PxRigidStaticPtr marker(gPhysics->createRigidStatic(PxTransform(position)));
    PxShape* shape = PxRigidActorExt::createExclusiveShape(
        *marker,
        PxSphereGeometry(POCKET_RADIUS),
        *gTableMaterial
    );

    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);

    marker->setName(storeName("PocketMarker_" + std::to_string(index)));
    gScene->addActor(*marker);
    gStaticActors.push_back(std::move(marker));
}

static void createTable()
{
    const float halfL = TABLE_LENGTH * 0.5f;
    const float halfW = TABLE_WIDTH * 0.5f;

    gStaticActors.push_back(createStaticBox(
        PxVec3(0.0f, -0.02f, 0.0f),
        PxVec3(halfL, 0.02f, halfW),
        *gTableMaterial,
        "Table_Playing_Field"
    ));

    gStaticActors.push_back(createStaticBox(
        PxVec3(0.0f, RAIL_HEIGHT * 0.5f, halfW + RAIL_THICKNESS * 0.5f),
        PxVec3(halfL + RAIL_THICKNESS, RAIL_HEIGHT * 0.5f, RAIL_THICKNESS * 0.5f),
        *gRailMaterial,
        "Long_Rail_Top"
    ));

    gStaticActors.push_back(createStaticBox(
        PxVec3(0.0f, RAIL_HEIGHT * 0.5f, -halfW - RAIL_THICKNESS * 0.5f),
        PxVec3(halfL + RAIL_THICKNESS, RAIL_HEIGHT * 0.5f, RAIL_THICKNESS * 0.5f),
        *gRailMaterial,
        "Long_Rail_Bottom"
    ));

    gStaticActors.push_back(createStaticBox(
        PxVec3(halfL + RAIL_THICKNESS * 0.5f, RAIL_HEIGHT * 0.5f, 0.0f),
        PxVec3(RAIL_THICKNESS * 0.5f, RAIL_HEIGHT * 0.5f, halfW),
        *gRailMaterial,
        "Short_Rail_Right"
    ));

    gStaticActors.push_back(createStaticBox(
        PxVec3(-halfL - RAIL_THICKNESS * 0.5f, RAIL_HEIGHT * 0.5f, 0.0f),
        PxVec3(RAIL_THICKNESS * 0.5f, RAIL_HEIGHT * 0.5f, halfW),
        *gRailMaterial,
        "Short_Rail_Left"
    ));

    gPocketPositions.clear();

    gPocketPositions.push_back(PxVec3(-halfL, 0.0f, -halfW));
    gPocketPositions.push_back(PxVec3(-halfL, 0.0f, halfW));
    gPocketPositions.push_back(PxVec3(halfL, 0.0f, -halfW));
    gPocketPositions.push_back(PxVec3(halfL, 0.0f, halfW));
    gPocketPositions.push_back(PxVec3(0.0f, 0.0f, -halfW));
    gPocketPositions.push_back(PxVec3(0.0f, 0.0f, halfW));

    for (int i = 0; i < static_cast<int>(gPocketPositions.size()); ++i)
    {
        PxVec3 markerPosition = gPocketPositions[i];
        markerPosition.y = 0.012f;
        createPocketMarker(markerPosition, i + 1);
    }
}

static PxRigidDynamicPtr createBallActor(const PxVec3& position, int number, bool cue)
{
    PxRigidDynamicPtr actor(gPhysics->createRigidDynamic(PxTransform(position)));

    PxRigidActorExt::createExclusiveShape(
        *actor,
        PxSphereGeometry(BALL_RADIUS),
        *gBallMaterial
    );

    PxRigidBodyExt::setMassAndUpdateInertia(*actor, BALL_MASS);

    actor->setLinearDamping(0.120f);
    actor->setAngularDamping(0.140f);
    actor->setMaxAngularVelocity(120.0f);
    actor->setSolverIterationCounts(8, 4);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);

    if (cue)
        actor->setName("Cue_Ball");
    else
        actor->setName(storeName("Object_Ball_" + std::to_string(number)));

    gScene->addActor(*actor);
    return actor;
}

static void removeAllBalls()
{
    for (Ball& ball : gBalls)
    {
        if (ball.actor)
        {
            gScene->removeActor(*ball.actor);
            ball.actor.reset();
        }
    }

    gBalls.clear();
}

static void resetBalls()
{
    removeAllBalls();

    Ball cueBall;
    cueBall.actor = createBallActor(PxVec3(-0.72f, BALL_RADIUS, 0.0f), 0, true);
    cueBall.number = 0;
    cueBall.cue = true;
    cueBall.pocketed = false;
    gBalls.push_back(std::move(cueBall));

    const float spacing = BALL_RADIUS * 2.08f;
    const float xStep = spacing * 0.8660254f;

    const float apexX = 0.42f;
    int number = 1;

    for (int row = 0; row < 5; ++row)
    {
        const float x = apexX + row * xStep;

        for (int col = 0; col <= row; ++col)
        {
            const float z = (static_cast<float>(col) - static_cast<float>(row) * 0.5f) * spacing;

            Ball ball;
            ball.actor = createBallActor(PxVec3(x, BALL_RADIUS, z), number, false);
            ball.number = number;
            ball.cue = false;
            ball.pocketed = false;

            gBalls.push_back(std::move(ball));
            ++number;
        }
    }
}

static void initPhysX()
{
    gActorNames.reserve(128);

    gFoundation.reset(PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback));

    gPvd.reset(PxCreatePvd(*gFoundation));
    gTransport.reset(PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10));

    if (gPvd && gTransport)
    {
        gPvd->connect(*gTransport, PxPvdInstrumentationFlag::eALL);
    }

    PxTolerancesScale scale;

    gPhysics.reset(PxCreatePhysics(
        PX_PHYSICS_VERSION,
        *gFoundation,
        scale,
        true,
        gPvd.get()
    ));

    PxInitExtensions(*gPhysics, gPvd.get());

    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

    gDispatcher.reset(PxDefaultCpuDispatcherCreate(2));
    sceneDesc.cpuDispatcher = gDispatcher.get();
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;

    sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;
    sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;

    gScene.reset(gPhysics->createScene(sceneDesc));

    PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();

    if (pvdClient)
    {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    gTableMaterial.reset(gPhysics->createMaterial(0.18f, 0.12f, 0.02f));
    gBallMaterial.reset(gPhysics->createMaterial(0.08f, 0.05f, 0.97f));
    gRailMaterial.reset(gPhysics->createMaterial(0.0f, 0.0f, 1.0f));
    gCueMaterial.reset(gPhysics->createMaterial(0.20f, 0.12f, 0.35f));

    PxMaterial* billiardMaterials[] = {
        gTableMaterial.get(),
        gBallMaterial.get(),
        gRailMaterial.get(),
        gCueMaterial.get()
    };

    for (PxMaterial* material : billiardMaterials)
    {
        material->setFrictionCombineMode(PxCombineMode::eMIN);
        material->setRestitutionCombineMode(PxCombineMode::eMAX);
    }

    createTable();
    resetBalls();
}

static void cleanupPhysX()
{
    removeAllBalls();

    if (gScene)
    {
        for (PxRigidStaticPtr& actor : gStaticActors)
        {
            if (actor)
                gScene->removeActor(*actor);
        }
    }

    gStaticActors.clear();

    gTableMaterial.reset();
    gBallMaterial.reset();
    gRailMaterial.reset();
    gCueMaterial.reset();

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

    std::cout << "PhysX billiards base simulation started.\n";
    std::cout << "Open PhysX Visual Debugger and connect to 127.0.0.1:5425.\n";
    std::cout << "Scene contains a 2.54 x 1.27 m table and 16 billiard balls.\n";

    for (int frame = 0; frame < 600; ++frame)
    {
        gScene->simulate(TIME_STEP);
        gScene->fetchResults(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    cleanupPhysX();
    return 0;
}
