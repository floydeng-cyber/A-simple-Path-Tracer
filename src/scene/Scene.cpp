#include "Scene.hpp"
#include "Sampler.hpp"

Vector3f sampleCosineHemisphere(const Vector3f& surfaceNormal, Sampler& sampler) {
    const float u1 = sampler.randFloat();   // distance from disk center
    const float u2 = sampler.randFloat();   // rotation around normal

    const float radius = std::sqrtf(u1);
    const float phi = 2.f * M_PI * u2;
    
    const Vector3f localDir(
        radius * std::cos(phi),
        radius * std::sin(phi),
        std::sqrtf(std::max(0.f, 1.f - u1))
    );

    const Vector3f normal = normalize(surfaceNormal);

    Vector3f tangent = (std::abs(normal.x) > std::abs(normal.y))
        ? normalize(Vector3f(normal.z, 0.f, -normal.x))
        : normalize(Vector3f(0.f, normal.z, -normal.y));

    const Vector3f bitangent = crossProduct(normal, tangent);

    return normalize(
          tangent * localDir.x
        + bitangent * localDir.y
        + normal * localDir.z
    );
}

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

PhotonEmissionSample Scene::samplePhotonEmission(
    std::size_t photonCount, Sampler& sampler) const {

    PhotonEmissionSample sample;
    if (photonCount == 0) return sample;

    Intersection lightPoint;
    sampleLight(lightPoint, sample.pdfArea, sampler);
    if (sample.pdfArea <= EPSILON) return sample;

    sample.normal = normalize(lightPoint.normal);
    sample.direction = sampleCosineHemisphere(sample.normal, sampler);
    sample.position = offsetRayOrigin(lightPoint, sample.direction);

    const float cosTheta = std::max(0.f, dotProduct(sample.normal, sample.direction));

    sample.pdfDir = cosTheta / M_PI;
    if (sample.pdfDir <= EPSILON) return PhotonEmissionSample{};

    const float jointPDF = sample.pdfArea * sample.pdfDir;

    sample.power = lightPoint.emit * cosTheta
                / (jointPDF * static_cast<float>(photonCount));

    return sample;
}