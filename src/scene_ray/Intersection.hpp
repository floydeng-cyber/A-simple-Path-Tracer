#pragma once

#include <cmath>
#include <limits>

#include "Vector.hpp"
#include "Material.hpp"

struct Intersection {
    bool happened = false;      // happen or not
    Vector3f coords;            // position
    Vector3f normal;            // rendering surface normal
    Vector3f geometricNormal;   // geometric surface normal
    Vector3f positionError;     // impreciseness of hit position
    Material* m = nullptr;      // material of the hitted object

    double distance = std::numeric_limits<double>::infinity(); // distance from last object
};

// Surface offset to avoid self-intersection error
inline Vector3f offsetRayOrigin(const Intersection& point, const Vector3f& wi_outward) {
    const Vector3f n = dotProduct(wi_outward, point.geometricNormal) >= 0.f
                    ? point.geometricNormal : -point.geometricNormal;
    const float offset_distance = std::abs(n.x) * point.positionError.x
                         + std::abs(n.y) * point.positionError.y
                         + std::abs(n.z) * point.positionError.z;
    Vector3f origin = point.coords + n * offset_distance;
    for (int axis = 0; axis < 3; ++axis) {
        if (n[axis] != 0.f)
            origin[axis] = std::nextafter(origin[axis],
                (n[axis] > 0.f)
                ? std::numeric_limits<float>::infinity()
                : -std::numeric_limits<float>::infinity());
    }
    return origin;
}
