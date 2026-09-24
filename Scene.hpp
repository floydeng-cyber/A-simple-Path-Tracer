#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"
#include "AreaLight.hpp"
#include "BVH.hpp"
#include "Ray.hpp"

class Sampler;

class Scene {
private:
    // This order controls destruction order:
    // BVH, light(s), object(s), then material(s)
    std::vector<std::unique_ptr<Material>> materials;
    std::vector<std::unique_ptr<Object>> objects;
    std::vector<std::unique_ptr<Light>> lights;
    std::unique_ptr<BVHAccel> bvh;
public:
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    
    Scene() = default;

    Material& createMaterial(MaterialType type, const Vector3f& emission = Vector3f(0.f)) {
        auto material = std::make_unique<Material>(type, emission);
        Material& result = *material;
        materials.emplace_back(std::move(material));
        return result;
    }

    // add object to scene
    template<typename objType, typename... Args>
    objType& addObj(Args&&... args) {
        static_assert(std::is_base_of_v<Object, objType>);
        bvh.reset();

        auto object = std::make_unique<objType>(std::forward<Args>(args)...);

        objType& result = *object;
        objects.emplace_back(std::move(object));
        return result;
    }

    // add light to scene
    void Add(std::unique_ptr<Light> light) { lights.emplace_back(std::move(light)); }

    const std::vector<std::unique_ptr<Object>>& get_objects() const { return objects; }
    const std::vector<std::unique_ptr<Light>>& get_lights() const { return lights; }

    Intersection intersect(const Ray& ray) const;
    void buildBVH();

    void sampleLight(Intersection &pos, float &pdf, Sampler& sampler) const;
    bool trace(const Ray &ray, const std::vector<Object*> &objects, float &tNear, uint32_t &index, Object **hitObject);
    std::tuple<Vector3f, Vector3f> HandleAreaLight(const AreaLight &light, const Vector3f &hitPoint, const Vector3f &N,
                                                   const Vector3f &shadowPointOrig,
                                                   const std::vector<Object *> &objects, uint32_t &index,
                                                   const Vector3f &dir, float specularExponent);

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }
};