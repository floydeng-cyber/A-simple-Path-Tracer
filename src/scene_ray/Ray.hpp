#pragma once
#include "Vector.hpp"

struct Ray{
    Vector3f origin;
    Vector3f direction, direction_inv;

    Ray(const Vector3f& ori, const Vector3f& dir)
            : origin(ori), direction(dir) {
        direction_inv = Vector3f(1.f/direction.x, 1.f/direction.y, 1.f/direction.z);
    }
};
