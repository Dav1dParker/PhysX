#include <iostream>
#include <PxPhysicsAPI.h>
#include <memory>
#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"


//struct PxFoundationDeleter {
//    void operator()(physx::PxFoundation* p) const noexcept {
//        if (p) p->release();
//    }
//};
//
//struct PxPhysicsDeleter {
//    void operator()(physx::PxPhysics* p) const noexcept {
//        if (p) p->release();
//    }
//};
//
//
//struct PxSceneDeleter {
//    void operator()(physx::PxScene* p) const noexcept {
//        if (p) p->release();
//    }
//};




static physx::PxDefaultAllocator allocator;
static physx::PxDefaultErrorCallback errorCallback;
static bool gDidCleanup = false;

physx::PxFoundation* foundation = nullptr;
physx::PxPhysics* physics = nullptr;
physx::PxScene* scene = nullptr;
physx::PxPvd* pvd = nullptr;
physx::PxPvdTransport* transport = nullptr;
physx::PxDefaultCpuDispatcher* cpuDispatcher = nullptr;
Snippets::Camera* camera = nullptr;

physx::PxArray<physx::PxRigidActor*> actors;


static int gFrameCount = 0;
static const int gMaxFrames = 600;
static bool gExitRequested = false;



void keyPress(unsigned char key, const physx::PxTransform& camera)
{
    switch (toupper(key))
    {
        if (actors.size() > 1)
        {
            ((physx::PxRigidDynamic*)actors[1])->addForce(physx::PxVec3(1.0f, 1.0f, 1.0f));
        }
    }
}

void initPhysics()
{
    //physx::PxDefaultAllocator allocator;
    //physx::PxDefaultErrorCallback errorCallback;
    //physx::PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
    //foundation->release();

    //std::unique_ptr<physx::PxFoundation, PxFoundationDeleter> foundation(PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback));

    foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
    if (!foundation) {
        std::cerr << "PxCreateFoundation failed\n";
        std::abort();
    }


    pvd = physx::PxCreatePvd(*foundation);
    transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10000);

    if (pvd && transport) {
        if (!pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL)) {
            std::cerr << "PVD connect failed\n";
        }
    }

    physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), true, pvd);
    if (!physics) {
        std::cerr << "PxCreatePhysics failed\n";
        std::abort();
    }

    physx::PxSceneDesc sceneDesc(physics->getTolerancesScale());
    sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);

    cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    if (!cpuDispatcher) {
        std::cerr << "Dispatcher create failed\n";
        std::abort();
    }

    sceneDesc.cpuDispatcher = cpuDispatcher;
    sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;

    scene = physics->createScene(sceneDesc);
    if (!scene) {
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


    if (pvd && pvd->isConnected()) {
        physx::PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
        if (pvdClient) {
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
        }
    }

    physx::PxMaterial* rockMaterial = physics->createMaterial(0.5f, 0.5f, 0.1f);
    physx::PxMaterial* metalMaterial = physics->createMaterial(0.15f, 0.15f, 0.1f);
    physx::PxMaterial* iceMaterial = physics->createMaterial(0.028f, 0.028f, 0.1f);


    const physx::PxVec3 planeNormal = physx::PxVec3(0.4f, 1.0f, 0.0f).getNormalized();
    physx::PxPlane plane = physx::PxPlane(planeNormal, 0.0f);

    physx::PxRigidStatic* groundActor = physx::PxCreatePlane(*physics, plane, *rockMaterial);

    scene->addActor(*groundActor);
    actors.pushBack(groundActor);

    physx::PxBoxGeometry boxGeometry = physx::PxBoxGeometry(physx::PxVec3(0.5f, 0.5f, 0.5f));

    physx::PxShape* boxShape1 = physics->createShape(boxGeometry, *metalMaterial, true);
    physx::PxRigidDynamic* boxActor1 = physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 0.0f)));
    boxActor1->attachShape(*boxShape1);
    physx::PxRigidBodyExt::updateMassAndInertia(*boxActor1, 10.0f);
    scene->addActor(*boxActor1);
    actors.pushBack(boxActor1);


    physx::PxShape* boxShape2 = physics->createShape(boxGeometry, *iceMaterial, true);
    physx::PxRigidDynamic* boxActor2 = physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 5.0f)));
    boxActor2->attachShape(*boxShape2);
    physx::PxRigidBodyExt::updateMassAndInertia(*boxActor2, 10.0f);
    scene->addActor(*boxActor2);
    actors.pushBack(boxActor2);


    physx::PxShape* boxShape3 = physics->createShape(boxGeometry, *rockMaterial, true);
    physx::PxRigidDynamic* boxActor3 = physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 10.0f)));
    boxActor3->attachShape(*boxShape3);
    physx::PxRigidBodyExt::updateMassAndInertia(*boxActor3, 10.0f);
    scene->addActor(*boxActor3);
    actors.pushBack(boxActor3);

}


void renderCallback()
{
    if (!scene) return;

    scene->simulate(1.0f / 60.0f);
    scene->fetchResults(true);

    Snippets::startRender(camera);

    if (actors.size() > 0)
    {
        Snippets::renderActors(&actors[0], static_cast<uint32_t>(actors.size()), true);
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
    if (gDidCleanup) return;
    gDidCleanup = true;

    delete camera;
    camera = nullptr;

    // Release actors (optional, but safe if done once)
    for (physx::PxRigidActor* a : actors)
        if (a) a->release();
    actors.clear();

    if (scene) { scene->release(); scene = nullptr; }

    if (cpuDispatcher) { cpuDispatcher->release(); cpuDispatcher = nullptr; }

    PxCloseExtensions();

    if (physics) { physics->release(); physics = nullptr; }

    if (pvd)
    {
        pvd->disconnect(); // safe
        pvd->release();
        pvd = nullptr;
    }

    if (transport) { transport->release(); transport = nullptr; }

    if (foundation) { foundation->release(); foundation = nullptr; }
}



int main()
{
    camera = new Snippets::Camera(physx::PxVec3(0.0f, 20.0f, 20.0f), physx::PxVec3(0.0f, -1.0f, -1.0f));
    Snippets::setupDefault("PhysX test", camera, keyPress, renderCallback, exitCallback);
    initPhysics();
    glutMainLoop();
    //scene->release();
    //physics->release();
   

    return 0;
}

