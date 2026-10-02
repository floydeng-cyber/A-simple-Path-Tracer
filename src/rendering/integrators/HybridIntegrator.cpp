#include "HybridIntegrator.hpp"

#include "core/Sampler.hpp"
#include "core/global.hpp"
#include "geometry/Intersection.hpp"
#include "scene/Material.hpp"
#include "scene/Scene.hpp"

HybridIntegrator::HybridIntegrator(const RenderSettings& renderSettings)
    : settings(renderSettings),
      pathTracer(renderSettings),
      photonMapper(renderSettings, true) {} // Build only caustic photons for the hybrid.

void HybridIntegrator::buildPhotonMaps(const Scene& scene, Sampler& sampler) {
    photonMapper.buildPhotonMaps(scene, sampler);
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
        const BSDFSample scattering = hit.m->sampleSurface(
            outgoing, hit.normal, hit.geometricNormal, sampler);
        if (!scattering.isValid()) return Vector3f(0.f);


        return radiance(
            Ray(offsetRayOrigin(hit, scattering.direction), scattering.direction),
            scene,
            sampler,
            depth + 1) * scattering.weight;
    }

    return pathTracer.radianceAtHit(ray, hit, scene, sampler, depth)
         + photonMapper.getPhotonInRadius(photonMapper.causticMap, hit);
}
