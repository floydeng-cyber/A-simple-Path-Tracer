#include "HybridIntegrator.hpp"

#include "global.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Sampler.hpp"
#include "Scene.hpp"

HybridIntegrator::HybridIntegrator(const RenderSettings& renderSettings)
    : settings(renderSettings),
      pathTracer(renderSettings),
      photonMapper(renderSettings, PhotonMapBuildPolicy::CausticsOnly) {}

void HybridIntegrator::preProcess(const Scene& scene, Sampler& sampler) {
    photonMapper.preProcess(scene, sampler);
}

Vector3f HybridIntegrator::radiance(
    const Ray& ray,
    const Scene& scene,
    Sampler& sampler,
    int depth) const {

    if (depth > settings.MaxDepth) return Vector3f(0.f);

    const Intersection hit = scene.intersect(ray);
    if (!hit.happened || hit.m == nullptr) return Vector3f(0.f);
    if (hit.m->hasEmission()) return hit.m->getEmission();

    if (hit.m->isSpecular()) {
        const Vector3f outgoing = -ray.direction;
        const BSDFSample scattering = hit.m->sample(
            outgoing, hit.normal, sampler);
        if (!scattering.isValid()) return Vector3f(0.f);

        const Vector3f offsetNormal =
            dotProduct(scattering.direction, hit.normal) >= 0.f
            ? hit.normal
            : -hit.normal;

        return radiance(
            Ray(hit.coords + offsetNormal * EPSILON, scattering.direction),
            scene,
            sampler,
            depth + 1) * scattering.weight;
    }

    return pathTracer.radiance(ray, scene, sampler, depth)
         + photonMapper.estimateCaustic(hit);
}
