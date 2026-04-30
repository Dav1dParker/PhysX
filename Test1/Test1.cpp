#include "PxPhysicsAPI.h"
#include "deleters.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
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

static PxMaterialPtr gGroundMaterial;
static PxMaterialPtr gBoxMaterial;
static PxMaterialPtr gEnemyMaterial;
static PxMaterialPtr gGrenadeMaterial;
static PxMaterialPtr gTraceMaterial;

static constexpr float TIME_STEP = 1.0f / 60.0f;
static constexpr float FIELD_HALF_SIZE = 12.0f;
static constexpr float PLAYER_HEIGHT = 1.65f;
static constexpr float ENEMY_HEALTH = 100.0f;
static constexpr float ENEMY_RADIUS = 0.35f;
static constexpr float ENEMY_HALF_HEIGHT = 0.75f;

struct Enemy
{
    PxRigidDynamicPtr actor;
    float health = ENEMY_HEALTH;
};

static std::vector<PxRigidStaticPtr> gStaticActors;
static std::vector<std::unique_ptr<std::string>> gActorNames;
static Enemy gEnemy;

static float gAimAngleDegrees = 0.0f;
static int gFrameCounter = 0;

static const char* storeName(const std::string& name)
{
    gActorNames.push_back(std::make_unique<std::string>(name));
    return gActorNames.back()->c_str();
}

static float degreesToRadians(float degrees)
{
    return degrees * PxPi / 180.0f;
}

static PxVec3 getAimDirection()
{
    const float angle = degreesToRadians(gAimAngleDegrees);
    return PxVec3(std::cos(angle), 0.0f, std::sin(angle)).getNormalized();
}

static void clearSceneActors()
{
    gEnemy.actor.reset();
    gStaticActors.clear();
    gActorNames.clear();
}

static PxRigidStaticPtr createStaticBox(
    const PxVec3& position,
    const PxVec3& halfExtents,
    PxMaterial& material,
    const std::string& name)
{
    PxRigidStaticPtr actor(gPhysics->createRigidStatic(PxTransform(position)));
    PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtents), material, true);
    actor->attachShape(*shape);
    shape->release();
    actor->userData = const_cast<char*>(storeName(name));
    gScene->addActor(*actor);
    return actor;
}

static PxRigidDynamicPtr createEnemyCapsule(const PxVec3& position)
{
    PxRigidDynamicPtr actor(gPhysics->createRigidDynamic(PxTransform(position)));
    PxShape* shape = gPhysics->createShape(
        PxCapsuleGeometry(ENEMY_RADIUS, ENEMY_HALF_HEIGHT),
        *gEnemyMaterial,
        true);

    shape->setLocalPose(PxTransform(PxQuat(PxPi / 2.0f, PxVec3(0.0f, 0.0f, 1.0f))));
    actor->attachShape(*shape);
    shape->release();

    PxRigidBodyExt::updateMassAndInertia(*actor, 80.0f);
    actor->setAngularDamping(0.35f);
    actor->setLinearDamping(0.08f);
    actor->setName("Enemy capsule");
    actor->userData = const_cast<char*>(storeName("enemy"));
    gScene->addActor(*actor);
    return actor;
}

static void createShooterScene()
{
    clearSceneActors();

    gStaticActors.push_back(createStaticBox(
        PxVec3(0.0f, -0.05f, 0.0f),
        PxVec3(FIELD_HALF_SIZE, 0.05f, FIELD_HALF_SIZE),
        *gGroundMaterial,
        "ground"));

    gStaticActors.push_back(createStaticBox(
        PxVec3(3.5f, 0.75f, 1.7f),
        PxVec3(0.45f, 0.75f, 2.2f),
        *gBoxMaterial,
        "cover left"));

    gStaticActors.push_back(createStaticBox(
        PxVec3(6.0f, 0.6f, -1.7f),
        PxVec3(1.2f, 0.6f, 0.35f),
        *gBoxMaterial,
        "cover right"));

    gStaticActors.push_back(createStaticBox(
        PxVec3(7.5f, 0.9f, 2.8f),
        PxVec3(0.45f, 0.9f, 1.4f),
        *gBoxMaterial,
        "far cover"));

    gEnemy.actor = createEnemyCapsule(PxVec3(8.5f, ENEMY_RADIUS + ENEMY_HALF_HEIGHT, 0.0f));
    gEnemy.health = ENEMY_HEALTH;

    std::cout << "Scene reset. Enemy health: " << gEnemy.health << "\n";
}

static void printHud()
{
    if ((gFrameCounter++ % 20) != 0)
    {
        return;
    }

    const PxVec3 direction = getAimDirection();
    std::cout << std::fixed << std::setprecision(1)
        << "Aim: " << gAimAngleDegrees << " deg"
        << " | dir=(" << direction.x << ", " << direction.z << ")"
        << " | Enemy health: " << gEnemy.health
        << "\n";
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

    gGroundMaterial.reset(gPhysics->createMaterial(0.7f, 0.6f, 0.1f));
    gBoxMaterial.reset(gPhysics->createMaterial(0.6f, 0.5f, 0.25f));
    gEnemyMaterial.reset(gPhysics->createMaterial(0.5f, 0.4f, 0.15f));
    gGrenadeMaterial.reset(gPhysics->createMaterial(0.35f, 0.25f, 0.65f));
    gTraceMaterial.reset(gPhysics->createMaterial(0.0f, 0.0f, 0.0f));
}

static void cleanupPhysX()
{
    clearSceneActors();

    gGroundMaterial.reset();
    gBoxMaterial.reset();
    gEnemyMaterial.reset();
    gGrenadeMaterial.reset();
    gTraceMaterial.reset();

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
    createShooterScene();

    std::cout << "PhysX shooter simulation started.\n";
    std::cout << "Open PhysX Visual Debugger and connect to 127.0.0.1:5425.\n";

    for (int frame = 0; frame < 600; ++frame)
    {
        gScene->simulate(TIME_STEP);
        gScene->fetchResults(true);

        printHud();
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    cleanupPhysX();
    return 0;
}
