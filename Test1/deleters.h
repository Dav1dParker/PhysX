#include <PxPhysicsAPI.h>


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
#pragma once
