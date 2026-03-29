#include "PxPhysicsAPI.h"
#include "deleters.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <conio.h>
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

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

static constexpr float PI = 3.14159265358979323846f;

static constexpr float TABLE_LENGTH = 2.54f;
static constexpr float TABLE_WIDTH = 1.27f;

static constexpr float BALL_RADIUS = 0.028575f;
static constexpr float BALL_MASS = 0.17f;

static constexpr float RAIL_HEIGHT = 0.12f;
static constexpr float RAIL_THICKNESS = 0.08f;

static constexpr float POCKET_RADIUS = 0.06f;
static constexpr float POCKET_RADIUS_SQ = POCKET_RADIUS * POCKET_RADIUS;
static constexpr float STOP_SPEED = 0.006f;

static constexpr float MIN_SHOT_POWER = 0.20f;
static constexpr float DEFAULT_SHOT_POWER = 1.10f;
static constexpr float MAX_SHOT_POWER = 3.20f;
static constexpr float SHOT_POWER_CHANGE_PER_SECOND = 1.60f;
static constexpr float AIM_CHANGE_PER_SECOND = 120.0f * PI / 180.0f;

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

static float gAimAngle = 0.0f;
static float gShotPower = DEFAULT_SHOT_POWER;
static bool  gGameOver = false;
static bool  gPlayerWon = false;
static bool  gCueBallPocketed = false;
static bool  gQuitRequested = false;

static const char* storeName(const std::string& value)
{
    gActorNames.push_back(value);
    return gActorNames.back().c_str();
}

static int readKeyNonBlocking()
{
#ifdef _WIN32
    if (_kbhit())
        return _getch();
    return -1;
#else
    struct TerminalGuard
    {
        termios oldState{};

        TerminalGuard()
        {
            tcgetattr(STDIN_FILENO, &oldState);

            termios newState = oldState;
            newState.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
            tcsetattr(STDIN_FILENO, TCSANOW, &newState);

            int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }

        ~TerminalGuard()
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldState);
        }
    };

    static TerminalGuard guard;

    unsigned char ch = 0;
    ssize_t n = read(STDIN_FILENO, &ch, 1);
    if (n == 1)
        return static_cast<int>(ch);

    return -1;
#endif
}

#ifdef _WIN32
static bool wasVirtualKeyPressed(int virtualKey)
{
    static bool previousState[256] = {};

    const bool pressed = (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
    const bool pressedThisFrame = pressed && !previousState[virtualKey];
    previousState[virtualKey] = pressed;

    return pressedThisFrame;
}

static bool isVirtualKeyDown(int virtualKey)
{
    return (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
}
#endif

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

    gGameOver = false;
    gPlayerWon = false;
    gCueBallPocketed = false;

    gAimAngle = 0.0f;
    gShotPower = DEFAULT_SHOT_POWER;

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

static Ball* getCueBall()
{
    for (Ball& ball : gBalls)
    {
        if (ball.cue)
            return &ball;
    }

    return nullptr;
}

static bool allBallsStopped()
{
    for (const Ball& ball : gBalls)
    {
        if (ball.pocketed || !ball.actor)
            continue;

        const PxVec3 v = ball.actor->getLinearVelocity();
        const PxVec3 w = ball.actor->getAngularVelocity();

        if (v.magnitude() > STOP_SPEED)
            return false;

        if (w.magnitude() > 1.0f)
            return false;
    }

    return true;
}

static int remainingObjectBalls()
{
    int count = 0;

    for (const Ball& ball : gBalls)
    {
        if (!ball.cue && !ball.pocketed)
            ++count;
    }

    return count;
}

static bool isInsidePocketZone(const PxVec3& ballPosition)
{
    for (const PxVec3& pocket : gPocketPositions)
    {
        const float dx = ballPosition.x - pocket.x;
        const float dz = ballPosition.z - pocket.z;

        if (dx * dx + dz * dz <= POCKET_RADIUS_SQ)
            return true;
    }

    return false;
}

static void removePocketedBall(Ball& ball)
{
    if (!ball.actor)
        return;

    gScene->removeActor(*ball.actor);
    ball.actor.reset();
    ball.pocketed = true;
}

static void checkPocketedBalls()
{
    if (gGameOver)
        return;

    for (Ball& ball : gBalls)
    {
        if (ball.pocketed || !ball.actor)
            continue;

        const PxVec3 position = ball.actor->getGlobalPose().p;

        if (isInsidePocketZone(position))
        {
            if (ball.cue)
            {
                removePocketedBall(ball);
                gCueBallPocketed = true;
                std::cout << "\nCue ball pocketed. Waiting for balls to stop.\n";
                continue;
            }

            std::cout << "\nBall " << ball.number << " pocketed.\n";
            removePocketedBall(ball);
        }
    }

    if (!allBallsStopped())
        return;

    if (gCueBallPocketed)
    {
        gGameOver = true;
        gPlayerWon = false;
        std::cout << "\nDefeat: cue ball was pocketed.\n";
        return;
    }

    if (remainingObjectBalls() == 0)
    {
        gGameOver = true;
        gPlayerWon = true;
        std::cout << "\nVictory: all object balls were pocketed.\n";
    }
}

static void hitCueBall()
{
    if (gGameOver)
        return;

    if (!allBallsStopped())
        return;

    Ball* cueBall = getCueBall();

    if (!cueBall || cueBall->pocketed || !cueBall->actor)
        return;

    PxVec3 direction(std::cos(gAimAngle), 0.0f, std::sin(gAimAngle));
    direction.normalize();

    cueBall->actor->wakeUp();
    cueBall->actor->addForce(direction * gShotPower, PxForceMode::eIMPULSE, true);
}

static void printHud()
{
    std::cout
        << "\rAim: " << static_cast<int>(gAimAngle * 180.0f / PI)
        << " deg | Power: " << gShotPower
        << " | Balls left: " << remainingObjectBalls()
        << " | " << (allBallsStopped() ? "Ready" : "Moving")
        << "        "
        << std::flush;
}

static void handleInput()
{
#ifdef _WIN32
    if (isVirtualKeyDown('A') || isVirtualKeyDown(VK_LEFT))
        gAimAngle += AIM_CHANGE_PER_SECOND * TIME_STEP;

    if (isVirtualKeyDown('D') || isVirtualKeyDown(VK_RIGHT))
        gAimAngle -= AIM_CHANGE_PER_SECOND * TIME_STEP;

    if (isVirtualKeyDown('W') || isVirtualKeyDown(VK_UP))
        gShotPower = std::min(MAX_SHOT_POWER, gShotPower + SHOT_POWER_CHANGE_PER_SECOND * TIME_STEP);

    if (isVirtualKeyDown('S') || isVirtualKeyDown(VK_DOWN))
        gShotPower = std::max(MIN_SHOT_POWER, gShotPower - SHOT_POWER_CHANGE_PER_SECOND * TIME_STEP);

    if (wasVirtualKeyPressed(VK_SPACE))
        hitCueBall();

    if (wasVirtualKeyPressed('R'))
    {
        resetBalls();
        std::cout << "\nGame reset.\n";
    }

    if (wasVirtualKeyPressed('Q') || wasVirtualKeyPressed(VK_ESCAPE))
        gQuitRequested = true;
#else
    const int key = readKeyNonBlocking();

    if (key < 0)
        return;

    switch (key)
    {
    case 'a':
    case 'A':
        gAimAngle += 5.0f * PI / 180.0f;
        break;

    case 'd':
    case 'D':
        gAimAngle -= 5.0f * PI / 180.0f;
        break;

    case 'w':
    case 'W':
        gShotPower = std::min(MAX_SHOT_POWER, gShotPower + 0.10f);
        break;

    case 's':
    case 'S':
        gShotPower = std::max(MIN_SHOT_POWER, gShotPower - 0.10f);
        break;

    case ' ':
        hitCueBall();
        break;

    case 'r':
    case 'R':
        resetBalls();
        std::cout << "\nGame reset.\n";
        break;

    case 'q':
    case 'Q':
    case 27:
        gQuitRequested = true;
        break;

    default:
        break;
    }
#endif
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
    std::cout << "Controls: A/D aim, W/S power, Space hit, R reset, Q/Esc quit.\n";

    auto lastHud = std::chrono::steady_clock::now();

    bool quit = false;

    while (!quit)
    {
        handleInput();

        if (gQuitRequested)
        {
            quit = true;
            break;
        }

        gScene->simulate(TIME_STEP);
        gScene->fetchResults(true);

        checkPocketedBalls();

        const auto now = std::chrono::steady_clock::now();
        const auto hudDelta = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastHud).count();

        if (hudDelta > 250)
        {
            printHud();
            lastHud = now;
        }

        if (gGameOver)
        {
            if (gPlayerWon)
                std::cout << "\nGame over: victory.\n";
            else
                std::cout << "\nGame over.\n";

            std::cout << "Press R to restart or Q/Esc to quit.\n";

            while (true)
            {
#ifdef _WIN32
                if (wasVirtualKeyPressed('R'))
                {
                    resetBalls();
                    gGameOver = false;
                    break;
                }

                if (wasVirtualKeyPressed('Q') || wasVirtualKeyPressed(VK_ESCAPE))
                {
                    quit = true;
                    break;
                }
#else
                const int key = readKeyNonBlocking();

                if (key == 'r' || key == 'R')
                {
                    resetBalls();
                    gGameOver = false;
                    break;
                }

                if (key == 'q' || key == 'Q' || key == 27)
                {
                    quit = true;
                    break;
                }
#endif

                std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    cleanupPhysX();
    return 0;
}
