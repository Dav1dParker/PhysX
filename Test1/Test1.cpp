#define NOMINMAX

#include "PxPhysicsAPI.h"
#include "deleters.h"

#include "NvCloth/Callbacks.h"
#include "NvCloth/Cloth.h"
#include "NvCloth/Factory.h"
#include "NvCloth/Fabric.h"
#include "NvCloth/Solver.h"
#include "NvClothExt/ClothFabricCooker.h"
#include "NvClothExt/ClothMeshDesc.h"
#include "snippetrender/SnippetCamera.h"
#include "snippetrender/SnippetRender.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
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

static constexpr float TIME_STEP = 1.0f / 60.0f;
static constexpr int FLAG_COLUMNS = 12;
static constexpr int FLAG_ROWS = 8;
static constexpr float FLAG_WIDTH = 4.0f;
static constexpr float FLAG_HEIGHT = 2.4f;
static constexpr float BANNER_TOP_Y = 3.4f;
static constexpr float RECT_FLAG_TOP_Y = 3.2f;
static constexpr float WIND_POWER_MIN = 0.0f;
static constexpr float WIND_POWER_MAX = 2.0f;
static constexpr float WIND_POWER_STEP = 0.1f;

struct NvClothAssertHandler final : nv::cloth::PxAssertHandler
{
    void operator()(const char* exp, const char* file, int line, bool& ignore) override
    {
        std::cerr << "NvCloth assert: " << exp << " (" << file << ':' << line << ")\n";
        ignore = false;
    }
};

struct ClothPatch
{
    std::string name;
    int rows = 0;
    int columns = 0;
    std::vector<PxVec3> points;
    std::vector<float> invMasses;
    std::vector<PxVec4> particles;
    std::vector<PxU32> indices;
    std::vector<PxU32> quads;
    nv::cloth::Fabric* fabric = nullptr;
    nv::cloth::Cloth* cloth = nullptr;
};

static std::vector<PxRigidStaticPtr> gStaticActors;
static std::vector<ClothPatch> gCloths;
static NvClothAssertHandler gNvClothAssertHandler;
static nv::cloth::Factory* gClothFactory = nullptr;
static nv::cloth::Solver* gClothSolver = nullptr;
static Snippets::Camera* gCamera = nullptr;
static float gSimulationTime = 0.0f;
static float gWindPower = 1.0f;
static int gFrame = 0;

static int vertexIndex(const ClothPatch& cloth, int row, int column)
{
    return row * cloth.columns + column;
}

static nv::cloth::Range<const PxVec4> constRange(const std::vector<PxVec4>& values)
{
    return nv::cloth::Range<const PxVec4>(values.data(), values.data() + values.size());
}

static PxRigidStaticPtr createStaticBox(const PxVec3& position, const PxVec3& halfSize, PxMaterial& material)
{
    PxRigidStaticPtr actor(gPhysics->createRigidStatic(PxTransform(position)));
    PxRigidActorExt::createExclusiveShape(*actor, PxBoxGeometry(halfSize), material);
    gScene->addActor(*actor);
    return actor;
}

static PxVec3 calculateWind(float time)
{
    float horizontalAngle = 0.85f * std::sin(time * 0.55f) + 0.35f * std::sin(time * 1.35f);
    float strength = 4.5f + 2.2f * std::sin(time * 0.9f) + 1.1f * std::sin(time * 2.1f);
    strength = std::max(strength, 0.8f);

    return PxVec3(std::cos(horizontalAngle), 0.12f * std::sin(time * 1.7f), std::sin(horizontalAngle)) * strength;
}

static void appendGridGeometry(ClothPatch& cloth)
{
    for (int row = 0; row + 1 < cloth.rows; ++row)
    {
        for (int column = 0; column + 1 < cloth.columns; ++column)
        {
            PxU32 a = static_cast<PxU32>(vertexIndex(cloth, row, column));
            PxU32 b = static_cast<PxU32>(vertexIndex(cloth, row, column + 1));
            PxU32 c = static_cast<PxU32>(vertexIndex(cloth, row + 1, column));
            PxU32 d = static_cast<PxU32>(vertexIndex(cloth, row + 1, column + 1));

            cloth.indices.push_back(a);
            cloth.indices.push_back(c);
            cloth.indices.push_back(b);

            cloth.indices.push_back(b);
            cloth.indices.push_back(c);
            cloth.indices.push_back(d);

            cloth.quads.push_back(a);
            cloth.quads.push_back(b);
            cloth.quads.push_back(d);
            cloth.quads.push_back(c);
        }
    }
}

static void addParticle(ClothPatch& cloth, const PxVec3& position, bool pinned)
{
    cloth.points.push_back(position);
    cloth.invMasses.push_back(pinned ? 0.0f : 1.0f);
    cloth.particles.emplace_back(position.x, position.y, position.z, pinned ? 0.0f : 1.0f);
}

static ClothPatch makeBannerCloth()
{
    ClothPatch cloth;
    cloth.name = "banner";
    cloth.rows = FLAG_ROWS;
    cloth.columns = FLAG_COLUMNS;

    float dy = FLAG_HEIGHT / static_cast<float>(FLAG_ROWS - 1);

    for (int row = 0; row < FLAG_ROWS; ++row)
    {
        float vertical = static_cast<float>(row) / static_cast<float>(FLAG_ROWS - 1);
        float rowWidth = std::max(FLAG_WIDTH * (1.0f - vertical), 0.18f);
        float leftX = -5.2f - rowWidth * 0.5f;

        for (int column = 0; column < FLAG_COLUMNS; ++column)
        {
            float horizontal = static_cast<float>(column) / static_cast<float>(FLAG_COLUMNS - 1);
            PxVec3 position(leftX + horizontal * rowWidth, BANNER_TOP_Y - row * dy, 0.0f);
            bool pinned = (row == 0 && (column == 0 || column == FLAG_COLUMNS - 1));
            addParticle(cloth, position, pinned);
        }
    }

    appendGridGeometry(cloth);
    return cloth;
}

static ClothPatch makeRectangularSidePinnedFlag()
{
    ClothPatch cloth;
    cloth.name = "rectangular flag";
    cloth.rows = FLAG_ROWS;
    cloth.columns = FLAG_COLUMNS;

    float leftX = 1.8f;
    float dx = FLAG_WIDTH / static_cast<float>(FLAG_COLUMNS - 1);
    float dy = FLAG_HEIGHT / static_cast<float>(FLAG_ROWS - 1);

    for (int row = 0; row < FLAG_ROWS; ++row)
    {
        for (int column = 0; column < FLAG_COLUMNS; ++column)
        {
            PxVec3 position(leftX + column * dx, RECT_FLAG_TOP_Y - row * dy, 0.0f);
            bool pinned = (column == 0 && (row == 0 || row == FLAG_ROWS - 1));
            addParticle(cloth, position, pinned);
        }
    }

    appendGridGeometry(cloth);
    return cloth;
}

static void cookCloth(ClothPatch& cloth)
{
    nv::cloth::ClothMeshDesc meshDesc;
    meshDesc.points.data = cloth.points.data();
    meshDesc.points.count = static_cast<PxU32>(cloth.points.size());
    meshDesc.points.stride = sizeof(PxVec3);
    meshDesc.invMasses.data = cloth.invMasses.data();
    meshDesc.invMasses.count = static_cast<PxU32>(cloth.invMasses.size());
    meshDesc.invMasses.stride = sizeof(float);
    meshDesc.quads.data = cloth.quads.data();
    meshDesc.quads.count = static_cast<PxU32>(cloth.quads.size() / 4);
    meshDesc.quads.stride = sizeof(PxU32) * 4;

    cloth.fabric = NvClothCookFabricFromMesh(gClothFactory, meshDesc, PxVec3(0.0f, -1.0f, 0.0f), nullptr, false);

    cloth.cloth = gClothFactory->createCloth(constRange(cloth.particles), *cloth.fabric);

    cloth.cloth->setGravity(PxVec3(0.0f, -9.81f, 0.0f));
    cloth.cloth->setDamping(PxVec3(0.12f, 0.12f, 0.12f));
    cloth.cloth->setSolverFrequency(600.0f);
    cloth.cloth->setStiffnessFrequency(60.0f);
    cloth.cloth->setTetherConstraintScale(1.1f);
    cloth.cloth->setTetherConstraintStiffness(0.7f);
    cloth.cloth->setDragCoefficient(0.35f);
    cloth.cloth->setLiftCoefficient(0.05f);
    cloth.cloth->setFluidDensity(1.0f);
    cloth.cloth->setSelfCollisionDistance(0.0f);
    cloth.cloth->setSelfCollisionStiffness(0.0f);
    cloth.cloth->clearInertia();

    std::vector<nv::cloth::PhaseConfig> phaseConfigs;
    phaseConfigs.reserve(cloth.fabric->getNumPhases());
    for (PxU32 i = 0; i < cloth.fabric->getNumPhases(); ++i)
    {
        nv::cloth::PhaseConfig config(static_cast<uint16_t>(i));
        config.mStiffness = 1.0f;
        config.mStiffnessMultiplier = 1.0f;
        config.mCompressionLimit = 1.0f;
        config.mStretchLimit = 1.05f;
        phaseConfigs.push_back(config);
    }
    cloth.cloth->setPhaseConfig(nv::cloth::Range<const nv::cloth::PhaseConfig>(phaseConfigs.data(), phaseConfigs.data() + phaseConfigs.size()));

    gClothSolver->addCloth(cloth.cloth);
}

static void initNvCloth()
{
    nv::cloth::InitializeNvCloth(&gAllocator, &gErrorCallback, &gNvClothAssertHandler, nullptr);

    gClothFactory = NvClothCreateFactoryCPU();
    gClothSolver = gClothFactory->createSolver();
}

static void releaseCloth(ClothPatch& cloth)
{
    if (gClothSolver && cloth.cloth)
    {
        gClothSolver->removeCloth(cloth.cloth);
    }

    delete cloth.cloth;
    cloth.cloth = nullptr;

    if (cloth.fabric)
    {
        cloth.fabric->decRefCount();
        cloth.fabric = nullptr;
    }
}

static void cleanupNvCloth()
{
    for (ClothPatch& cloth : gCloths)
    {
        releaseCloth(cloth);
    }
    gCloths.clear();

    delete gClothSolver;
    gClothSolver = nullptr;

    if (gClothFactory)
    {
        NvClothDestroyFactory(gClothFactory);
        gClothFactory = nullptr;
    }
}

static void createScene()
{
    gStaticActors.clear();
    gCloths.clear();

    gStaticActors.push_back(createStaticBox(PxVec3(0.0f, -0.05f, 0.0f), PxVec3(8.0f, 0.05f, 2.5f), *gGroundMaterial));

    float bannerPoleHeight = BANNER_TOP_Y + 0.25f;
    gStaticActors.push_back(createStaticBox(PxVec3(-7.2f, bannerPoleHeight * 0.5f, -0.08f), PxVec3(0.04f, bannerPoleHeight * 0.5f, 0.04f), *gPoleMaterial));
    gStaticActors.push_back(createStaticBox(PxVec3(-3.2f, bannerPoleHeight * 0.5f, -0.08f), PxVec3(0.04f, bannerPoleHeight * 0.5f, 0.04f), *gPoleMaterial));
    gStaticActors.push_back(createStaticBox(PxVec3(-5.2f, BANNER_TOP_Y, -0.08f), PxVec3(2.08f, 0.035f, 0.035f), *gPoleMaterial));

    float flagPoleHeight = RECT_FLAG_TOP_Y + 0.25f;
    gStaticActors.push_back(createStaticBox(PxVec3(1.8f, flagPoleHeight * 0.5f, -0.08f), PxVec3(0.04f, flagPoleHeight * 0.5f, 0.04f), *gPoleMaterial));

    gCloths.push_back(makeBannerCloth());
    gCloths.push_back(makeRectangularSidePinnedFlag());

    for (ClothPatch& cloth : gCloths)
    {
        cookCloth(cloth);
    }
}

static void updateScene(float dt)
{
    gSimulationTime += dt;
    PxVec3 wind = calculateWind(gSimulationTime) * gWindPower;

    for (ClothPatch& cloth : gCloths)
    {
        cloth.cloth->setWindVelocity(wind);
        cloth.cloth->wakeUp();
    }

    if (gClothSolver->beginSimulation(dt))
    {
        int chunkCount = gClothSolver->getSimulationChunkCount();
        for (int i = 0; i < chunkCount; ++i)
        {
            gClothSolver->simulateChunk(i);
        }
        gClothSolver->endSimulation();
    }

    gScene->simulate(dt);
    gScene->fetchResults(true);
}

static void printStatus()
{
    if (gFrame % 30 != 0)
    {
        return;
    }

    PxVec3 wind = calculateWind(gSimulationTime) * gWindPower;
    std::cout << std::fixed << std::setprecision(2)
              << " wind=(" << wind.x << ", " << wind.y << ", " << wind.z << ")"
              << " | strength=" << wind.magnitude()
              << " | power=" << gWindPower
              << '\n';
}

static void renderCallback()
{
    updateScene(TIME_STEP);
    printStatus();
    ++gFrame;

    Snippets::startRender(gCamera);

    std::vector<PxRigidActor*> actors;
    actors.reserve(gStaticActors.size());
    for (const PxRigidStaticPtr& actor : gStaticActors)
    {
        actors.push_back(actor.get());
    }

    if (!actors.empty())
    {
        Snippets::renderActors(actors.data(), static_cast<PxU32>(actors.size()), true, PxVec3(0.55f, 0.55f, 0.55f));
    }

    for (const ClothPatch& cloth : gCloths)
    {
        nv::cloth::MappedRange<const PxVec4> particles = nv::cloth::readCurrentParticles(*cloth.cloth);
        Snippets::renderMesh(
            static_cast<PxU32>(particles.size()),
            particles.begin(),
            static_cast<PxU32>(cloth.indices.size() / 3),
            cloth.indices.data(),
            cloth.name == "banner" ? PxVec3(0.18f, 0.65f, 0.28f) : PxVec3(0.14f, 0.46f, 0.95f),
            nullptr,
            false
        );
    }

    Snippets::print("NvCloth CPU cloth demo");
    Snippets::finishRender();
}

static void keyboardCallback(unsigned char key, const PxTransform&)
{
    if (key == 27)
    {
        glutLeaveMainLoop();
    }
    else if (key == '1')
    {
        gWindPower = std::max(WIND_POWER_MIN, gWindPower - WIND_POWER_STEP);
        std::cout << "wind power=" << std::fixed << std::setprecision(1) << gWindPower << '\n';
    }
    else if (key == '2')
    {
        gWindPower = std::min(WIND_POWER_MAX, gWindPower + WIND_POWER_STEP);
        std::cout << "wind power=" << std::fixed << std::setprecision(1) << gWindPower << '\n';
    }
}

static void exitCallback()
{
    cleanupNvCloth();

    delete gCamera;
    gCamera = nullptr;

    gStaticActors.clear();
    gGroundMaterial.reset();
    gPoleMaterial.reset();

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
}

int main()
{
    initPhysX();
    initNvCloth();
    createScene();

    gCamera = new Snippets::Camera(PxVec3(-1.8f, 2.3f, 9.0f), PxVec3(0.1f, -0.15f, -1.0f));
    Snippets::setupDefault("NvCloth Flags", gCamera, keyboardCallback, renderCallback, exitCallback);
    glutMainLoop();

    return 0;
}
