#pragma once

#include "Vector.hpp"
#include "Material.hpp"

//class Object;

struct Intersection {
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        m=nullptr;
    }
    bool happened;
    Vector3f coords;
    Vector3f normal;
    Vector3f geometricNormal;
    Vector3f positionError;
    Vector3f emit;
    double distance;
    Material* m;
};

// Offset using the geometric surface and a bound on position rounding error.
// Round away from the surface so the offset cannot disappear in float arithmetic.
inline Vector3f offsetRayOrigin(const Intersection& hit, const Vector3f& direction) {
    const Vector3f n = dotProduct(direction, hit.geometricNormal) >= 0.f
        ? hit.geometricNormal : -hit.geometricNormal;
    const float distance = std::abs(n.x) * hit.positionError.x
                         + std::abs(n.y) * hit.positionError.y
                         + std::abs(n.z) * hit.positionError.z;
    Vector3f origin = hit.coords + n * distance;
    for (int axis = 0; axis < 3; ++axis) {
        if (n[axis] != 0.f)
            origin[axis] = std::nextafter(origin[axis], n[axis] > 0.f
                ? std::numeric_limits<float>::infinity()
                : -std::numeric_limits<float>::infinity());
    }
    return origin;
}
