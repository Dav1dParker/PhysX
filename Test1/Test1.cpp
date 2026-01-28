#include <iostream>
#include <PxPhysicsAPI.h>

int main()
{
    physx::PxDefaultAllocator allocator;
    physx::PxDefaultErrorCallback errorCallback;
    physx::PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);

    foundation->release();
    return 0;
}

