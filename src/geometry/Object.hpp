#pragma once

#include "Vector.hpp"
#include "global.hpp"
#include "Bounds3.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"

class Sampler;

class Object {
public:
    //Object() {}
    virtual ~Object() = default;

    virtual Intersection getIntersection(Ray _ray) = 0;

    virtual Bounds3 getBounds() = 0;
    virtual float getArea() = 0;
    virtual void Sample(Intersection &pos,
                        float &pdf,
                        Sampler& sampler) = 0;
    virtual bool hasEmit() = 0;
};