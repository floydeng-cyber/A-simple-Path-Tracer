#include "Scene.hpp"
#include "Sampler.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    
    std::vector<Object*> objectView;
    objectView.reserve(objects.size());
    for (const auto& obj : objects)
        objectView.emplace_back(obj.get());

    bvh = std::make_unique<BVHAccel>(std::move(objectView), 1, BVHAccel::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray& ray) const {
    return bvh ? bvh->Intersect(ray) : Intersection{};
}

void Scene::sampleLight(Intersection& pos, float& pdf, Sampler& sampler) const {
    float emit_area_sum = 0.f;

    for (const auto& obj : objects) {
        if (obj->hasEmit())  // accumulate emisstion area
            emit_area_sum += obj->getArea();
    }
    if (emit_area_sum <= 0.f) {
        pos = Intersection();
        pdf = 0.f;
        return;
    }

    const float samplePos = sampler.randFloat() * emit_area_sum;
    float accumulate_emit_area = 0.f;
    
    for (const auto& obj : objects) {
        if (obj->hasEmit()) {
            accumulate_emit_area += obj->getArea();

            if (samplePos <= accumulate_emit_area){
                obj->Sample(pos, pdf, sampler);
                pdf *= obj->getArea() / emit_area_sum;
                return;
            }
        }
    }
    pos = Intersection();
    pdf = 0.f;
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float& tNear, uint32_t &index, Object **hitObject) {
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}