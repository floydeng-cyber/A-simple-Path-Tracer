#include "HybridIntegrator.hpp"

#include "Sampler.hpp"
#include "global.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Scene.hpp"

HybridIntegrator::HybridIntegrator(const RenderSettings& renderSettings)
    : settings(renderSettings),
      pathTracer(renderSettings),
      photonMapper(renderSettings, true) {} // Build only caustic photons for the hybrid.

void HybridIntegrator::buildPhotonMaps(const Scene& scene, Sampler& sampler) {
    photonMapper.buildPhotonMaps(scene, sampler);
}

Vector3f HybridIntegrator::radiance(
    const Ray& ray,     const Scene& scene,
    Sampler& sampler,   int depth) const {

    if (depth > settings.MaxDepth) return Vector3f(0.f);

    const Intersection hit = scene.intersect(ray);
    if (!hit.happened || hit.m == nullptr) return Vector3f(0.f);

    if (hit.m->hasEmission())
        return (depth == 0)
            ? hit.m->getEmission().getClamped()
            : hit.m->getEmission();

    if (hit.m->isSpecular()) {
        const Vector3f outgoing = -ray.direction;
        const BSDFSample scattering = hit.m->sampleSurface(
            outgoing, hit.normal, hit.geometricNormal, sampler);
        if (!scattering.isValid()) return Vector3f(0.f);

        if (hit.m->getType() == DIELECTRIC && depth < settings.specularBothWayDepth) {
            const DielectricBranches bothBranch = hit.m->dielectricBranches(
                outgoing, hit.normal, hit.geometricNormal, TransportMode::Radiance
            );

            Vector3f L_specular(0.f);

            if (bothBranch.reflection.isValid()) {
                const Ray ray_r(offsetRayOrigin(hit, bothBranch.reflection.direction),
                    bothBranch.reflection.direction);

                L_specular += radiance(ray_r, scene, sampler, depth + 1
                ) * bothBranch.reflection.weight;
            }
            if (bothBranch.transmission.isValid()) {
                const Ray ray_r(offsetRayOrigin(hit, bothBranch.transmission.direction),
                    bothBranch.transmission.direction);
                L_specular += radiance(
                    ray_r, scene, sampler, depth + 1
                ) * bothBranch.transmission.weight;
            }
            return L_specular;
        }

        Vector3f L_specular = radiance(
            Ray(offsetRayOrigin(hit, scattering.direction), scattering.direction),
            scene, sampler, depth + 1) * scattering.weight;

        if (hit.m->getType() == DIELECTRIC) {
            constexpr float threshold = 4.f;
            constexpr float extend = 4.f;
            float peak = std::max(L_specular.x, std::max(L_specular.y, L_specular.z));
            if (peak > threshold) {
                float excess = peak - threshold;
                float compressed_peak = threshold + extend * excess / (extend + excess);
                L_specular = L_specular * compressed_peak / peak;
            }
        }
        return L_specular;
    }

    return pathTracer.radianceAtHit(ray, hit, scene, sampler, depth)
         + photonMapper.getPhotonInRadius(photonMapper.causticMap, hit) * hit.m->Kd / M_PI;
}
