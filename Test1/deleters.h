#pragma once

#include <PxPhysicsAPI.h>

#include <memory>

struct PxFoundationDeleter {
    void operator()(physx::PxFoundation* p) const noexcept {
        if (p) p->release();
    }
};

struct PxPhysicsDeleter {
    void operator()(physx::PxPhysics* p) const noexcept {
        if (p) p->release();
    }
};

struct PxSceneDeleter {
    void operator()(physx::PxScene* p) const noexcept {
        if (p) p->release();
    }
};

struct PxPvdDeleter {
    void operator()(physx::PxPvd* p) const noexcept {
        if (p) p->release();
    }
};

struct PxPvdTransportDeleter {
    void operator()(physx::PxPvdTransport* p) const noexcept {
        if (p) p->release();
    }
};

struct PxDefaultCpuDispatcherDeleter {
    void operator()(physx::PxDefaultCpuDispatcher* p) const noexcept {
        if (p) p->release();
    }
};

struct PxMaterialDeleter {
    void operator()(physx::PxMaterial* p) const noexcept {
        if (p) p->release();
    }
};

struct PxRigidStaticDeleter {
    void operator()(physx::PxRigidStatic* p) const noexcept {
        if (p) p->release();
    }
};

struct PxShapeDeleter {
    void operator()(physx::PxShape* p) const noexcept {
        if (p) p->release();
    }
};

struct PxRigidDynamicDeleter {
    void operator()(physx::PxRigidDynamic* p) const noexcept {
        if (p) p->release();
    }
};

using PxFoundationPtr = std::unique_ptr<physx::PxFoundation, PxFoundationDeleter>;
using PxPhysicsPtr = std::unique_ptr<physx::PxPhysics, PxPhysicsDeleter>;
using PxScenePtr = std::unique_ptr<physx::PxScene, PxSceneDeleter>;
using PxPvdPtr = std::unique_ptr<physx::PxPvd, PxPvdDeleter>;
using PxPvdTransportPtr = std::unique_ptr<physx::PxPvdTransport, PxPvdTransportDeleter>;
using PxDefaultCpuDispatcherPtr = std::unique_ptr<physx::PxDefaultCpuDispatcher, PxDefaultCpuDispatcherDeleter>;
using PxMaterialPtr = std::unique_ptr<physx::PxMaterial, PxMaterialDeleter>;
using PxRigidStaticPtr = std::unique_ptr<physx::PxRigidStatic, PxRigidStaticDeleter>;
using PxShapePtr = std::unique_ptr<physx::PxShape, PxShapeDeleter>;
using PxRigidDynamicPtr = std::unique_ptr<physx::PxRigidDynamic, PxRigidDynamicDeleter>;
