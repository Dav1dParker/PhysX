#include "PxPhysicsAPI.h"
#include "deleters.h"

#include <chrono>
#include <iostream>
#include <thread>

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

static constexpr float TIME_STEP = 1.0f / 60.0f;

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
}

static void cleanupPhysX()
{
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

    for (int frame = 0; frame < 120; ++frame)
    {
        gScene->simulate(TIME_STEP);
        gScene->fetchResults(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    cleanupPhysX();
    return 0;
}
