#include "PxPhysicsAPI.h"
#include "deleters.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
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
static constexpr float AIM_STEP_DEGREES = 4.0f;
static constexpr float BULLET_RANGE = 30.0f;
static constexpr float BULLET_SPREAD_DEGREES = 2.0f;
static constexpr float BULLET_DAMAGE = 20.0f;
static constexpr float BULLET_IMPULSE = 550.0f;
static constexpr float GRENADE_RADIUS = 0.16f;
static constexpr float GRENADE_MASS = 0.45f;
static constexpr float GRENADE_TTL = 2.0f;
static constexpr float GRENADE_SPEED = 6.5f;
static constexpr float GRENADE_UP_SPEED = 3.5f;
static constexpr float EXPLOSION_RADIUS = 7.0f;
static constexpr float EXPLOSION_MAX_DAMAGE = 65.0f;
static constexpr float EXPLOSION_MAX_IMPULSE = 900.0f;
static constexpr float TRACE_TTL = 0.45f;
static constexpr float AIM_REFERENCE_LENGTH = 4.0f;

struct Enemy
{
    PxRigidDynamicPtr actor;
    float health = ENEMY_HEALTH;
};

struct Grenade
{
    PxRigidDynamicPtr actor;
    float ttl = GRENADE_TTL;
};

struct TraceMarker
{
    PxRigidStaticPtr actor;
    float ttl = TRACE_TTL;
};

static std::vector<PxRigidStaticPtr> gStaticActors;
static std::vector<Grenade> gGrenades;
static std::vector<TraceMarker> gTraceMarkers;
static std::vector<std::unique_ptr<std::string>> gActorNames;
static PxRigidStaticPtr gPlayerMarker;
static PxRigidStaticPtr gAimReferenceMarker;
static PxRigidStaticPtr gAimEndMarker;
static Enemy gEnemy;

static float gAimAngleDegrees = 0.0f;
static bool gQuitRequested = false;
static int gFrameCounter = 0;
static std::mt19937 gRandomGenerator(std::random_device{}());

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

static PxVec3 getPlayerPosition()
{
    return PxVec3(0.0f, PLAYER_HEIGHT, 0.0f);
}

static bool wasVirtualKeyPressed(int key)
{
    return (GetAsyncKeyState(key) & 0x0001) != 0;
}

static bool isVirtualKeyDown(int key)
{
    return (GetAsyncKeyState(key) & 0x8000) != 0;
}

static bool actorHasName(const PxActor* actor, const char* name)
{
    return actor && actor->userData && std::string(static_cast<const char*>(actor->userData)) == name;
}

static void clearSceneActors()
{
    gTraceMarkers.clear();
    gGrenades.clear();
    gAimEndMarker.reset();
    gAimReferenceMarker.reset();
    gPlayerMarker.reset();
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

static PxRigidStaticPtr createVisualBox(
    const PxTransform& pose,
    const PxVec3& halfExtents,
    PxMaterial& material,
    const std::string& name)
{
    PxRigidStaticPtr actor(gPhysics->createRigidStatic(pose));
    PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtents), material, true);
    shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    actor->attachShape(*shape);
    shape->release();
    actor->userData = const_cast<char*>(storeName(name));
    gScene->addActor(*actor);
    return actor;
}

static PxRigidStaticPtr createVisualSphere(
    const PxVec3& position,
    float radius,
    PxMaterial& material,
    const std::string& name)
{
    PxRigidStaticPtr actor(gPhysics->createRigidStatic(PxTransform(position)));
    PxShape* shape = gPhysics->createShape(PxSphereGeometry(radius), material, true);
    shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    actor->attachShape(*shape);
    shape->release();
    actor->userData = const_cast<char*>(storeName(name));
    gScene->addActor(*actor);
    return actor;
}

static PxQuat rotationFromXAxisToDirection(const PxVec3& direction)
{
    const PxVec3 from(1.0f, 0.0f, 0.0f);
    const PxVec3 to = direction.getNormalized();
    const float dot = PxClamp(from.dot(to), -1.0f, 1.0f);

    if (dot > 0.9999f)
    {
        return PxQuat(PxIdentity);
    }

    if (dot < -0.9999f)
    {
        return PxQuat(PxPi, PxVec3(0.0f, 1.0f, 0.0f));
    }

    const PxVec3 axis = from.cross(to).getNormalized();
    return PxQuat(std::acos(dot), axis);
}

static void createTraceMarker(const PxVec3& start, const PxVec3& end)
{
    const PxVec3 delta = end - start;
    const float length = delta.magnitude();
    if (length <= 0.001f)
    {
        return;
    }

    const PxVec3 center = start + delta * 0.5f;
    PxRigidStaticPtr actor(gPhysics->createRigidStatic(PxTransform(center, rotationFromXAxisToDirection(delta))));
    PxShape* shape = gPhysics->createShape(PxBoxGeometry(length * 0.5f, 0.015f, 0.015f), *gTraceMaterial, true);
    shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    actor->attachShape(*shape);
    shape->release();
    actor->userData = const_cast<char*>(storeName("trace"));
    gScene->addActor(*actor);
    gTraceMarkers.push_back(TraceMarker{ std::move(actor), TRACE_TTL });
}

static void updateAimReferenceMarkers()
{
    if (!gAimReferenceMarker || !gAimEndMarker)
    {
        return;
    }

    const PxVec3 start = getPlayerPosition();
    const PxVec3 direction = getAimDirection();
    const PxVec3 end = start + direction * AIM_REFERENCE_LENGTH;
    const PxVec3 center = start + direction * (AIM_REFERENCE_LENGTH * 0.5f);

    gAimReferenceMarker->setGlobalPose(PxTransform(center, rotationFromXAxisToDirection(direction)));
    gAimEndMarker->setGlobalPose(PxTransform(end));
}

static void createReferenceMarkers()
{
    const PxVec3 playerPosition = getPlayerPosition();
    const PxVec3 aimDirection = getAimDirection();
    const PxVec3 aimCenter = playerPosition + aimDirection * (AIM_REFERENCE_LENGTH * 0.5f);
    const PxVec3 aimEnd = playerPosition + aimDirection * AIM_REFERENCE_LENGTH;

    gPlayerMarker = createVisualSphere(playerPosition, 0.12f, *gTraceMaterial, "player shoot origin");
    gAimReferenceMarker = createVisualBox(
        PxTransform(aimCenter, rotationFromXAxisToDirection(aimDirection)),
        PxVec3(AIM_REFERENCE_LENGTH * 0.5f, 0.025f, 0.025f),
        *gTraceMaterial,
        "aim reference line");
    gAimEndMarker = createVisualSphere(aimEnd, 0.10f, *gTraceMaterial, "aim reference end");
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

static PxRigidDynamicPtr createGrenadeActor(const PxVec3& position, const PxVec3& velocity)
{
    PxRigidDynamicPtr actor(gPhysics->createRigidDynamic(PxTransform(position)));
    PxShape* shape = gPhysics->createShape(PxSphereGeometry(GRENADE_RADIUS), *gGrenadeMaterial, true);
    actor->attachShape(*shape);
    shape->release();

    PxRigidBodyExt::updateMassAndInertia(*actor, GRENADE_MASS);
    actor->setLinearVelocity(velocity);
    actor->setAngularDamping(0.05f);
    actor->setLinearDamping(0.01f);
    actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
    actor->setName("Grenade");
    actor->userData = const_cast<char*>(storeName("grenade"));
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
    createReferenceMarkers();

    std::cout << "Scene reset. Enemy health: " << gEnemy.health << "\n";
}

static PxVec3 getShotDirectionWithSpread()
{
    std::uniform_real_distribution<float> spread(-BULLET_SPREAD_DEGREES, BULLET_SPREAD_DEGREES);
    const float yaw = degreesToRadians(gAimAngleDegrees + spread(gRandomGenerator));
    const float pitch = degreesToRadians(spread(gRandomGenerator));
    const float horizontal = std::cos(pitch);
    return PxVec3(
        std::cos(yaw) * horizontal,
        std::sin(pitch),
        std::sin(yaw) * horizontal).getNormalized();
}

static void damageEnemy(float damage)
{
    gEnemy.health = std::max(0.0f, gEnemy.health - damage);
}

static void shoot()
{
    const PxVec3 origin = getPlayerPosition();
    const PxVec3 direction = getShotDirectionWithSpread();

    PxRaycastBuffer hit;
    const bool hasHit = gScene->raycast(origin, direction, BULLET_RANGE, hit);
    const PxVec3 end = hasHit
        ? origin + direction * hit.block.distance
        : origin + direction * BULLET_RANGE;

    createTraceMarker(origin, end);

    if (!hasHit)
    {
        std::cout << "Shot missed.\n";
        return;
    }

    PxActor* hitActor = hit.block.actor;
    if (actorHasName(hitActor, "enemy"))
    {
        damageEnemy(BULLET_DAMAGE);
        gEnemy.actor->addForce(direction * BULLET_IMPULSE, PxForceMode::eIMPULSE);
        std::cout << "Bullet hit enemy. Damage: " << BULLET_DAMAGE
            << ". Enemy health: " << gEnemy.health << "\n";
        return;
    }

    const char* actorName = hitActor && hitActor->userData
        ? static_cast<const char*>(hitActor->userData)
        : "unknown";
    std::cout << "Bullet hit " << actorName << ".\n";
}

static void throwGrenade()
{
    const PxVec3 direction = getAimDirection();
    const PxVec3 position = getPlayerPosition() + direction * 0.7f + PxVec3(0.0f, -0.25f, 0.0f);
    const PxVec3 velocity = direction * GRENADE_SPEED + PxVec3(0.0f, GRENADE_UP_SPEED, 0.0f);
    gGrenades.push_back(Grenade{ createGrenadeActor(position, velocity), GRENADE_TTL });
    std::cout << "Grenade thrown.\n";
}

static bool isEnemyProtectedFromExplosion(const PxVec3& explosionPosition, const PxVec3& enemyPosition)
{
    const PxVec3 delta = enemyPosition - explosionPosition;
    const float distance = delta.magnitude();
    if (distance <= 0.001f)
    {
        return false;
    }

    PxRaycastBuffer hit;
    if (!gScene->raycast(explosionPosition, delta / distance, distance, hit))
    {
        return false;
    }

    return !actorHasName(hit.block.actor, "enemy");
}

static void explodeGrenade(const PxVec3& position)
{
    std::cout << "Grenade exploded at (" << position.x << ", " << position.y << ", " << position.z << ").\n";

    if (!gEnemy.actor)
    {
        return;
    }

    const PxVec3 enemyPosition = gEnemy.actor->getGlobalPose().p;
    const PxVec3 delta = enemyPosition - position;
    const float distance = delta.magnitude();

    if (distance > EXPLOSION_RADIUS)
    {
        std::cout << "Enemy is outside explosion radius.\n";
        return;
    }

    if (isEnemyProtectedFromExplosion(position, enemyPosition))
    {
        std::cout << "Enemy protected by cover. No explosion damage.\n";
        return;
    }

    const float factor = 1.0f - distance / EXPLOSION_RADIUS;
    const float damage = EXPLOSION_MAX_DAMAGE * factor;
    const float impulse = EXPLOSION_MAX_IMPULSE * factor;
    PxVec3 impulseDirection = delta;

    if (impulseDirection.magnitudeSquared() < 0.0001f)
    {
        impulseDirection = PxVec3(1.0f, 0.5f, 0.0f);
    }

    impulseDirection.normalize();
    damageEnemy(damage);
    gEnemy.actor->addForce(impulseDirection * impulse, PxForceMode::eIMPULSE);

    std::cout << "Explosion damaged enemy. Damage: " << damage
        << ". Enemy health: " << gEnemy.health << "\n";
}

static void updateTransientActors(float dt)
{
    for (Grenade& grenade : gGrenades)
    {
        grenade.ttl -= dt;
    }

    for (TraceMarker& marker : gTraceMarkers)
    {
        marker.ttl -= dt;
    }

    auto grenadeIt = std::remove_if(
        gGrenades.begin(),
        gGrenades.end(),
        [](Grenade& grenade)
        {
            if (grenade.ttl > 0.0f)
            {
                return false;
            }

            const PxVec3 position = grenade.actor->getGlobalPose().p;
            grenade.actor.reset();
            explodeGrenade(position);
            return true;
        });
    gGrenades.erase(grenadeIt, gGrenades.end());

    auto traceIt = std::remove_if(
        gTraceMarkers.begin(),
        gTraceMarkers.end(),
        [](TraceMarker& marker)
        {
            return marker.ttl <= 0.0f;
        });
    gTraceMarkers.erase(traceIt, gTraceMarkers.end());
}

static void handleInput()
{
    if (isVirtualKeyDown('A') || isVirtualKeyDown(VK_LEFT))
    {
        gAimAngleDegrees -= AIM_STEP_DEGREES;
    }

    if (isVirtualKeyDown('D') || isVirtualKeyDown(VK_RIGHT))
    {
        gAimAngleDegrees += AIM_STEP_DEGREES;
    }

    if (wasVirtualKeyPressed(VK_SPACE))
    {
        shoot();
    }

    if (wasVirtualKeyPressed('G'))
    {
        throwGrenade();
    }

    if (wasVirtualKeyPressed('R'))
    {
        createShooterScene();
    }

    if (wasVirtualKeyPressed('Q') || wasVirtualKeyPressed(VK_ESCAPE))
    {
        gQuitRequested = true;
    }
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
        << " | Grenades: " << gGrenades.size()
        << " | A/D aim, Space shoot, G grenade, R reset, Q/Esc quit"
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
    gGrenadeMaterial->setRestitutionCombineMode(PxCombineMode::eMAX);
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


    while (!gQuitRequested)
    {
        handleInput();
        updateAimReferenceMarkers();
        updateTransientActors(TIME_STEP);

        gScene->simulate(TIME_STEP);
        gScene->fetchResults(true);

        printHud();
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    cleanupPhysX();
    return 0;
}
