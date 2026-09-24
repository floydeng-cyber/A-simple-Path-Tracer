#include "Scene.hpp"
#include "Sampler.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const {
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf, Sampler& sampler) const {
    float emit_area_sum = 0.f;

    for (Object* object : objects) {
        if (object->hasEmit())  // accumulate emisstion area
            emit_area_sum += object->getArea();
    }
    if (emit_area_sum <= 0.f) {
        pos = Intersection();
        pdf = 0.f;
        return;
    }

    const float samplePos = sampler.randFloat() * emit_area_sum;
    float total_emit_area = emit_area_sum;
    emit_area_sum = 0;

    for (Object* object : objects) {
        if (object->hasEmit()) {
            emit_area_sum += object->getArea();

            if (samplePos <= emit_area_sum){
                object->Sample(pos, pdf, sampler);
                const float emitProbability = samplePos / total_emit_area;
                pdf *= emitProbability;
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