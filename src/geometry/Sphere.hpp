#pragma once

#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"
#include "Sampler.hpp"

class Sphere : public Object{
public:
    Vector3f center;
    float radius, radius2;
    Material *m = nullptr;
    float area;
    Sphere(const Vector3f &pos, const float &r, Material* mt)
        : center(pos), radius(r), radius2(r * r), m(mt), area(4 * M_PI *r *r) {}
    bool intersect(const Ray& ray) {
        // analytic solution
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        float area = 4 * M_PI * radius2;
        if (!solveQuadratic(a, b, c, t0, t1)) return false;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
    bool intersect(const Ray& ray, float &tnear, uint32_t &index) const {
        // analytic solution
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1)) return false;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        tnear = t0;

        return true;
    }
    Intersection getIntersection(Ray ray){
        Intersection result;
        result.happened = false;
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1)) return result;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return result;
        result.happened=true;

        result.coords = Vector3f(ray.origin + ray.direction * t0);
        result.normal = normalize(Vector3f(result.coords - center));
        result.coords = center + result.normal * radius;
        result.geometricNormal = result.normal;
        for (int i=0;i<3;++i)
            result.positionError[i] = 8.f * std::numeric_limits<float>::epsilon()
                * (std::abs(center[i]) + radius);
        result.m = this->m;
        result.obj = this;
        result.distance = t0;
        return result;

    }
    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index, const Vector2f &uv, Vector3f &N, Vector2f &st) const
    { N = normalize(P - center); }

    Vector3f evalDiffuseColor(const Vector2f &st)const {
        //return m->getColor();
        return m->Kd;
    }
    Bounds3 getBounds(){
        return Bounds3(Vector3f(center.x-radius, center.y-radius, center.z-radius),
                       Vector3f(center.x+radius, center.y+radius, center.z+radius));
    }
    void Sample(Intersection& pos, float& pdf, Sampler& sampler){
        // ==== Abandoned Sampling in Polar Coordinate ====
        //float theta = 2.0 * M_PI * sampler.randFloat(),
        //       phi = M_PI * sampler.randFloat();
        //Vector3f dir(std::cos(phi), std::sin(phi)*std::cos(theta), std::sin(phi)*std::sin(theta));
        //pos.coords = center + radius * dir;
        //pos.normal = dir;
        //pos.emit = m->getEmission();
        //pdf = 1.0f / area;

        // == uniform sampling in Cartesian ==
        const float heightSample  = sampler.randFloat(); // ~ [0,1]
        const float azimuthSample = sampler.randFloat(); // ~ [0,1]

        const float norHeight = 1.f - 2.f * heightSample; // ~ [-1,1]
        const float horizontal_radius = std::sqrtf(
            std::max(0.f, 1.f - norHeight * norHeight)
        );
        const float azimuthAng = 2.f * M_PI * azimuthSample; // ~ [0, 2pi]

        const Vector3f surfaceNormal(   // surface normal
            horizontal_radius * std::cos(azimuthAng),
            horizontal_radius * std::sin(azimuthAng),
            norHeight
        );

        pos.coords = center + radius * surfaceNormal;
        pos.normal = surfaceNormal;
        pos.geometricNormal = surfaceNormal;
        for (int i=0;i<3;++i)
            pos.positionError[i] = 8.f * std::numeric_limits<float>::epsilon()
                * (std::abs(center[i]) + radius);
        pos.emit = m->getEmission();
        pdf = 1.f / area;
    }

    float getArea() { return area; }
    bool hasEmit() { return m->hasEmission(); }
};
