#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "acceleration/BVH.hpp"
#include "core/Vector.hpp"
#include "geometry/Object.hpp"
#include "geometry/Ray.hpp"

class Sampler;

struct PhotonEmissionSample {
    Vector3f position = Vector3f(0.f);
    Vector3f normal = Vector3f(0.f);
    Vector3f direction = Vector3f(0.f);
    Vector3f power = Vector3f(0.f);

    float pdfArea = 0.f;
    float pdfDir = 0.f;

    bool isValid() const { return pdfArea > 0.f && pdfDir > 0.f; }
};

class Scene {
private:
    std::vector<std::unique_ptr<Material>> materials;
    std::vector<std::unique_ptr<Object>> objects;
    std::unique_ptr<BVHAccel> bvh;
public: 
    Scene() = default;

    Material& createMaterial(MaterialType type, const Vector3f& emission = Vector3f(0.f)) {
        auto material = std::make_unique<Material>(type, emission);
        Material& result = *material;
        materials.emplace_back(std::move(material));
        return result;
    }

    template<typename objType, typename... Args>
    objType& addObj(Args&&... args) {
        static_assert(std::is_base_of_v<Object, objType>);
        bvh.reset();

        auto object = std::make_unique<objType>(std::forward<Args>(args)...);

        objType& result = *object;
        objects.emplace_back(std::move(object));
        return result;
    }

    Intersection intersect(const Ray& ray) const;
    void buildBVH();

    void sampleLight(Intersection &pos, float &pdf, Sampler& sampler) const;

    PhotonEmissionSample samplePhotonEmission(std::size_t photonCount, Sampler& sampler) const;
};
