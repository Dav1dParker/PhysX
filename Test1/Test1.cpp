#include <iostream>
#include <PxPhysicsAPI.h>
#include <memory>
#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"
#include "deleters.h"







static physx::PxDefaultAllocator allocator;
static physx::PxDefaultErrorCallback errorCallback;
static bool gDidCleanup = false;

struct AppState {
    std::unique_ptr<physx::PxFoundation, PxFoundationDeleter> foundation;
    std::unique_ptr<physx::PxPhysics, PxPhysicsDeleter> physics;
    std::unique_ptr<physx::PxScene, PxSceneDeleter> scene;
    std::unique_ptr<physx::PxPvd, PxPvdDeleter> pvd;
    std::unique_ptr<physx::PxPvdTransport, PxPvdTransportDeleter> transport;
    std::unique_ptr<physx::PxDefaultCpuDispatcher, PxDefaultCpuDispatcherDeleter> cpuDispatcher;
    std::unique_ptr<Snippets::Camera> camera;
    physx::PxArray<physx::PxRigidActor*> actors;
};


static AppState* gApp = nullptr;
static int gFrameCount = 0;
static const int gMaxFrames = 600;
static bool gExitRequested = false;
static int cpuCoresCount = 2;



void keyPress(unsigned char key, const physx::PxTransform& camera)
{
    /*auto& app = gApp;
    switch (toupper(key))
    {
        if (app.actors.size() > 1)
        {
            ((physx::PxRigidDynamic*)app.actors[1])->addForce(physx::PxVec3(1.0f, 1.0f, 1.0f));
        }
    }*/
}

void initPhysics()
{
    //physx::PxDefaultAllocator allocator;
    //physx::PxDefaultErrorCallback errorCallback;
    //physx::PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
    //foundation->release();

    //std::unique_ptr<physx::PxFoundation, PxFoundationDeleter> foundation(PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback));

    auto& app = *gApp;

    app.foundation.reset(PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback));
    if (!app.foundation) {
        std::cerr << "PxCreateFoundation failed\n";
        std::abort();
    }


    app.pvd.reset(physx::PxCreatePvd(*app.foundation));
    app.transport.reset(physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10000));

    if (app.pvd && app.transport) {
        if (!app.pvd->connect(*app.transport, physx::PxPvdInstrumentationFlag::eALL)) {
            std::cerr << "PVD connect failed\n";
        }
    }

    app.physics.reset(PxCreatePhysics(PX_PHYSICS_VERSION, *app.foundation, physx::PxTolerancesScale(), true, app.pvd.get()));
    if (!app.physics) {
        std::cerr << "PxCreatePhysics failed\n";
        std::abort();
    }

    physx::PxSceneDesc sceneDesc(app.physics->getTolerancesScale());
    sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);

    app.cpuDispatcher.reset(physx::PxDefaultCpuDispatcherCreate(cpuCoresCount));
    if (!app.cpuDispatcher) {
        std::cerr << "Dispatcher create failed\n";
        std::abort();
    }

    sceneDesc.cpuDispatcher = app.cpuDispatcher.get();
    sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;

    app.scene.reset(app.physics->createScene(sceneDesc));
    if (!app.scene) {
        std::cerr << "createScene failed\n";
        std::abort();
    }




    //std::unique_ptr<physx::PxPhysics, PxPhysicsDeleter> physics(PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), true, pvd));
    //physx::PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), true, nullptr);
    //physx::PxSceneDesc sceneDesc = physx::PxSceneDesc(physics->getTolerancesScale());
    //sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
    //sceneDesc.cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    //sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
    //physx::PxScene* scene = physics->createScene(sceneDesc);
    //std::unique_ptr<physx::PxScene, PxSceneDeleter> scene(physics->createScene(sceneDesc));


    if (app.pvd && app.pvd->isConnected()) {
        physx::PxPvdSceneClient* pvdClient = app.scene->getScenePvdClient();
        if (pvdClient) {
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
        }
    }
    std::unique_ptr<physx::PxMaterial, PxMaterialDeleter> rockMaterial{app.physics->createMaterial(0.5f, 0.5f, 0.1f)};
    std::unique_ptr<physx::PxMaterial, PxMaterialDeleter> metalMaterial{ app.physics->createMaterial(0.15f, 0.15f, 0.1f) };
    std::unique_ptr<physx::PxMaterial, PxMaterialDeleter> iceMaterial{ app.physics->createMaterial(0.028f, 0.028f, 0.1f) };


    const physx::PxVec3 planeNormal = physx::PxVec3(0.4f, 1.0f, 0.0f).getNormalized();
    physx::PxPlane plane = physx::PxPlane(planeNormal, 0.0f);

    //physx::PxRigidStatic* groundActor = physx::PxCreatePlane(*app.physics, plane, *rockMaterial);
    physx::PxRigidStatic* groundActor = physx::PxCreatePlane(*app.physics, plane, *rockMaterial);

    app.scene->addActor(*groundActor);
    app.actors.pushBack(groundActor);

    physx::PxBoxGeometry boxGeometry = physx::PxBoxGeometry(physx::PxVec3(0.5f, 0.5f, 0.5f));

    std::unique_ptr<physx::PxShape, PxShapeDeleter> boxShape1{ app.physics->createShape(boxGeometry, *metalMaterial, false) };
    physx::PxRigidDynamic* boxActor1 = app.physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 0.0f)));
    boxActor1->attachShape(*boxShape1);
    physx::PxRigidBodyExt::updateMassAndInertia(*boxActor1, 10.0f);
    app.scene->addActor(*boxActor1);
    app.actors.pushBack(boxActor1);


    /*physx::PxShape* boxShape2 = physics->createShape(boxGeometry, *iceMaterial, true);
    physx::PxRigidDynamic* boxActor2 = physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 5.0f)));
    boxActor2->attachShape(*boxShape2);
    physx::PxRigidBodyExt::updateMassAndInertia(*boxActor2, 10.0f);
    scene->addActor(*boxActor2);
    actors.pushBack(boxActor2);*/

    std::unique_ptr<physx::PxShape, PxShapeDeleter> boxShape2{ app.physics->createShape(boxGeometry, *iceMaterial, false) };
    physx::PxRigidDynamic* boxActor2 = app.physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 5.0f)));
    boxActor2->attachShape(*boxShape2);
    physx::PxRigidBodyExt::updateMassAndInertia(*boxActor2, 10.0f);
    app.scene->addActor(*boxActor2);
    app.actors.pushBack(boxActor2);


    std::unique_ptr<physx::PxShape, PxShapeDeleter> boxShape3{ app.physics->createShape(boxGeometry, *rockMaterial, false) };
    physx::PxRigidDynamic* boxActor3 = app.physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 10.0f)));
    boxActor3->attachShape(*boxShape3);
    physx::PxRigidBodyExt::updateMassAndInertia(*boxActor3, 10.0f);
    app.scene->addActor(*boxActor3);
    app.actors.pushBack(boxActor3);

}


void renderCallback()
{
    auto& app = *gApp;
    if (!app.scene) return;

    app.scene->simulate(1.0f / 60.0f);
    app.scene->fetchResults(true);

    Snippets::startRender(app.camera.get());

    if (app.actors.size() > 0)
    {
        Snippets::renderActors(&app.actors[0], static_cast<uint32_t>(app.actors.size()), true);
    }

    Snippets::finishRender();

    ++gFrameCount;
    if (!gExitRequested && gFrameCount >= gMaxFrames)
    {
        gExitRequested = true;
        glutLeaveMainLoop();   // preferred with FreeGLUT
        // If glutLeaveMainLoop is unavailable, use: std::exit(0);
    }
}


void exitCallback()
{
    auto& app = *gApp;

    if (gDidCleanup) return;
    gDidCleanup = true;

    for (physx::PxRigidActor* a : app.actors) {
        if (a) a->release();
    }
    app.actors.clear();

    if (app.pvd)
    {
        app.pvd->disconnect();
    }

    app.scene.reset();
    app.cpuDispatcher.reset();
    app.physics.reset();
    app.pvd.reset();
    app.transport.reset();
    PxCloseExtensions();
    app.foundation.reset();
}



int main()
{
    AppState app;
    gApp = &app;
    app.camera = std::make_unique<Snippets::Camera>(physx::PxVec3(0.0f, 20.0f, 20.0f), physx::PxVec3(0.0f, -1.0f, -1.0f));
    Snippets::setupDefault("PhysX test", app.camera.get(), keyPress, renderCallback, exitCallback);
    initPhysics();
    glutMainLoop();
    //scene->release();
    //physics->release();
   

    return 0;
}

