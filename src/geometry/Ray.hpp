#pragma once
#include "core/Vector.hpp"

struct Ray{
    Vector3f origin;
    Vector3f direction, direction_inv;

    Ray(const Vector3f& ori, const Vector3f& dir)
            : origin(ori), direction(dir) {
        direction_inv = Vector3f(1. / direction.x, 1. / direction.y, 1. / direction.z);
    }
};
