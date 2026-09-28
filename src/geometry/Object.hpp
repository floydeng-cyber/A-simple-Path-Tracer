#pragma once

#include "core/Vector.hpp"
#include "core/global.hpp"
#include "geometry/Bounds3.hpp"
#include "geometry/Intersection.hpp"
#include "geometry/Ray.hpp"

class Sampler;

class Object {
public:
    Object() {}
    virtual ~Object() = default;

    virtual Intersection getIntersection(Ray _ray) = 0;

    virtual Bounds3 getBounds() = 0;
    virtual float getArea() = 0;
    virtual void Sample(Intersection &pos,
                        float &pdf,
                        Sampler& sampler) = 0;
    virtual bool hasEmit() = 0;
};
