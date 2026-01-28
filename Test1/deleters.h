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
#pragma once
