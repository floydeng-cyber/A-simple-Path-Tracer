#include "PhotonMappingIntegrator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "core/Sampler.hpp"
#include "core/global.hpp"
#include "geometry/Intersection.hpp"
#include "scene/Material.hpp"
#include "scene/Scene.hpp"

PhotonMappingIntegrator::PhotonMappingIntegrator(
    const RenderSettings& set,
    bool _trackCausticsOnly
): setting(set), trackCausticsOnly(_trackCausticsOnly) {}

void PhotonMappingIntegrator::tracePhoton(
    const Ray& ray,     const Vector3f& power,
    const Scene& scene, Sampler& sampler,
    int depth,          bool hittedDiffuse) {

    if ( depth >= setting.MaxDepth    // if reached max. depth
      || std::max(power.x, std::max(power.y, power.z)) < EPSILON) { // if power too low
        return;
    }

    // check hit - reject if [no hit], [hit 'nothing'], or [hit light]
    const Intersection hit = scene.intersect(ray);
    if (!hit.happened || hit.m == nullptr || hit.m->hasEmission()) return;

    const Vector3f wi = -ray.direction; // reverse ray direction for (wi)-related computations

    if (!hit.m->isSpecular()) {     // Hit diffuse surface: store @photon map
        if (depth > 0 && !hittedDiffuse) { // First diffuse hit after mirror/glass: caustic photon.
            causticMap.emplace_back(Photon{ hit.coords, wi, normalize(hit.normal), power });
        }

        if (trackCausticsOnly) return;  // extract casutic photons only (for hybrid rendering)

        if (depth == 0 || hittedDiffuse) {  // store direct AND indirect photon @diffuse
            diffuseMap.emplace_back(Photon{ hit.coords, wi, normalize(hit.normal), power });
        }

        // Continue the photon with roulette-compensated power.
        const float roulette_firstNsaved = (depth < setting.savefirstN)
                                           ? 1.f : setting.rusRoulette;
        if (sampler.randFloat() > roulette_firstNsaved) return;

        const BSDFSample bsdf = hit.m->sampleSurface(wi, hit.normal,
            hit.geometricNormal, sampler, TransportMode::Importance);
        if (!bsdf.isValid()) return;

        tracePhoton(
            Ray(offsetRayOrigin(hit, bsdf.direction), bsdf.direction),
            power * bsdf.weight / roulette_firstNsaved,
            scene, sampler, depth + 1, true);
        return;
    }

    // Mirror/Glass: Start recursion through BSDF sampling
    const BSDFSample bsdf = hit.m->sampleSurface(
        wi, hit.normal, hit.geometricNormal,
        sampler, TransportMode::Importance
    );
    if (!bsdf.isValid()) return;

    tracePhoton(
        Ray(offsetRayOrigin(hit, bsdf.direction), bsdf.direction),
        power * bsdf.weight, scene, sampler, depth + 1,
        hittedDiffuse
    );
}

void PhotonMappingIntegrator::buildPhotonTree(
    std::vector<Photon>& photons,
    std::size_t begin, std::size_t end, int depth) {

    if (begin >= end) return;

    const std::size_t middle = (begin+end)/2;
    const int axis = depth % 3;

    std::nth_element(photons.begin() + begin,
                     photons.begin() + middle,
                     photons.begin() + end,
        [axis](const Photon& lhs, const Photon& rhs) {
            return lhs.position[axis] < rhs.position[axis];
        });

    buildPhotonTree(photons, begin,      middle, depth + 1);
    buildPhotonTree(photons, middle + 1, end,    depth + 1);
}

void PhotonMappingIntegrator::buildPhotonMaps(const Scene& scene, Sampler& sampler) {
    // 1. Reset storage for this scene.
    causticMap.clear();
    diffuseMap.clear();
    causticMap.reserve(setting.photonCount / 8);
    if (!trackCausticsOnly)
        diffuseMap.reserve(setting.photonCount / 2);

    // 2. Emit photons from the lights and store their surface hits.
    for (int i = 0; i < setting.photonCount; ++i) {
        const PhotonEmitSample emission =
            scene.samplePhotonEmission(setting.photonCount, sampler);

        if (!emission.isValid()) continue;

        tracePhoton(Ray(emission.position, emission.direction), emission.power,
                    scene, sampler, 0, false);
    }

    // 3. Arrange stored photons into balanced trees for radius searches.
    buildPhotonTree(causticMap, 0, causticMap.size(), 0);
    std::cout << "Caustic photons stored: " << causticMap.size() << "\n";
    if (!trackCausticsOnly) {
        buildPhotonTree(diffuseMap, 0, diffuseMap.size(), 0);
        std::cout << "Global photons stored: " << diffuseMap.size() << "\n";
    }
}

Vector3f PhotonMappingIntegrator::getPhotonInRadius(
    const std::vector<Photon>& photons, const Intersection& hit) const {

    if (photons.empty() || setting.photonGatherRadius < EPSILON) return Vector3f(0.f);

    Vector3f powerSum(0.f);
    const float radius2 = setting.photonGatherRadius*setting.photonGatherRadius;

    auto gatherRange = [&](auto&& self, std::size_t begin, std::size_t end,
        int depth, Vector3f& powerSum)->void {

            if (begin >= end) return;

            const std::size_t middle = begin + (end - begin) / 2;
            const Photon& photon = photons[middle];
            const float dist2 = (photon.position - hit.coords).norm2();

            Vector3f normal = normalize(hit.normal);

            if (dist2 <= radius2 &&
                dotProduct(normal, photon.normal) > 0.85f &&
                dotProduct(normal, photon.incoming) > 0.f) {
                powerSum += photon.power
                    * (setting.photonConeFilter
                      ? 1.f - std::sqrt(dist2 / radius2)
                      : 1.f);
            }

            const int axis = depth % 3;
            const float axisDistance = hit.coords[axis] - photon.position[axis];

            if (axisDistance <= 0.f) {
                self(self, begin, middle, depth + 1, powerSum);
                if (axisDistance * axisDistance <= radius2) {
                    self(self, middle + 1, end, depth + 1, powerSum);
                }
            }
            else {
                self(self, middle + 1, end, depth + 1, powerSum);
                if (axisDistance * axisDistance <= radius2) {
                    self(self, begin, middle, depth + 1, powerSum);
                }
            }
        };

    gatherRange(gatherRange, 0, photons.size(), 0, powerSum);
    // Cone kernel w(d)=1-d/r integrates to pi*r^2/3 on a plane.
    return powerSum * ((setting.photonConeFilter ? 3.f : 1.f) / (M_PI * radius2));
}

Vector3f PhotonMappingIntegrator::radiance(
    const Ray& ray, const Scene& scene,
    Sampler& sampler, int depth) const {

    if (depth > setting.MaxDepth) return Vector3f(0.f);

    const Intersection hit = scene.intersect(ray);
    if (!hit.happened || hit.m == nullptr) return Vector3f(0.f);
    if (hit.m->hasEmission())   return hit.m->getEmission();

    // Store properties of the surface hit
    const Vector3f N_hit = hit.normal;
    const Vector3f wo = -ray.direction;

    // Recursion for mirror or dielectric surface
    if (hit.m->isSpecular()) {
        if (hit.m->getType() == DIELECTRIC && depth < setting.specularBothWayDepth) {
            const DielectricBranches bothBranch = hit.m->dielectricBranches(
                wo, N_hit, hit.geometricNormal, TransportMode::Radiance
            );

            Vector3f L_specular(0.f);

            if (bothBranch.reflection.isValid()) {
                const Ray ray_r(offsetRayOrigin(hit, bothBranch.reflection.direction),
                    bothBranch.reflection.direction);
                Vector3f L_reflect = radiance(
                    ray_r, scene, sampler, depth + 1
                ) * bothBranch.reflection.weight;
                //L_specular += Vector3f{
                //    L_reflect.x / (1.f + L_reflect.x/15),
                //    L_reflect.y / (1.f + L_reflect.y/15),
                //    L_reflect.z / (1.f + L_reflect.z/15)
                //};
                L_specular += 18 * Vector3f{
                    1 / (1.f + exp(-L_reflect.x / 5)) -0.5f,
                    1 / (1.f + exp(-L_reflect.y / 5)) -0.5f,
                    1 / (1.f + exp(-L_reflect.z / 5)) -0.5f
                };
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

        sampler.setDepth(depth);
        const BSDFSample bsdf = hit.m->sampleSurface(wo, N_hit, hit.geometricNormal, sampler);

        if (!bsdf.isValid())    return Vector3f(0.f);

        const Ray pathRay(offsetRayOrigin(hit, bsdf.direction), bsdf.direction);

        return radiance(pathRay, scene, sampler, depth + 1) * bsdf.weight;
    }


    // For diffuse surface
    // ===========================================================================
    const Vector3f L_diffuse = getPhotonInRadius(diffuseMap, hit) * hit.m->Kd / M_PI;
    const Vector3f L_caustic = getPhotonInRadius(causticMap, hit) * hit.m->Kd / M_PI;

    return L_diffuse + L_caustic;
}