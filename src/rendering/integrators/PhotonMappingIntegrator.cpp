#include "PhotonMappingIntegrator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "global.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Scene.hpp"
#include "Sampler.hpp"

float component(const Vector3f& value, int axis) {
    return value[axis];
}
float maxComponent(const Vector3f& value) {
    return std::max(value.x, std::max(value.y, value.z));
}

void PhotonMap::reserve(std::size_t count) {
    photons.reserve(count);
}

void PhotonMap::clear() {
    photons.clear();
}

void PhotonMap::store(const Photon& photon) {
    photons.emplace_back(photon);
}

void PhotonMap::balance() {
    balanceRange(0, photons.size(), 0);
}

void PhotonMap::balanceRange(std::size_t begin, std::size_t end, int depth) {
    if (begin >= end) return;

    const std::size_t middle = begin + (end - begin) / 2;
    const int axis = depth % 3;

    std::nth_element(
        photons.begin() + begin,
        photons.begin() + middle,
        photons.begin() + end,
        [axis](const Photon& lhs, const Photon& rhs) {
            return component(lhs.position, axis) < component(rhs.position, axis);
        });

    balanceRange(begin, middle, depth + 1);
    balanceRange(middle + 1, end, depth + 1);
}

Vector3f PhotonMap::estimateIrradiance(
    const Vector3f& position,
    const Vector3f& normal,
    float radius) const {

    if (photons.empty() || radius <= 0.f) return Vector3f(0.f);

    Vector3f powerSum(0.f);
    const float radiusSquared = radius * radius;
    gatherRange(
        0,
        photons.size(),
        0,
        position,
        normalize(normal),
        radiusSquared,
        powerSum);

    return powerSum / (M_PI * radiusSquared);
}

void PhotonMap::gatherRange(
    std::size_t begin,
    std::size_t end,
    int depth,
    const Vector3f& position,
    const Vector3f& normal,
    float radiusSquared,
    Vector3f& powerSum) const {

    if (begin >= end) return;

    const std::size_t middle = begin + (end - begin) / 2;
    const Photon& photon = photons[middle];
    const Vector3f offset = photon.position - position;
    const float distanceSquared = offset.norm2();

    if (distanceSquared <= radiusSquared &&
        dotProduct(normal, photon.normal) > 0.85f &&
        dotProduct(normal, photon.incoming) > 0.f) {
        powerSum += photon.power;
    }

    const int axis = depth % 3;
    const float axisDistance = component(position, axis) -
                               component(photon.position, axis);

    if (axisDistance <= 0.f) {
        gatherRange(begin, middle, depth + 1, position, normal,
                    radiusSquared, powerSum);
        if (axisDistance * axisDistance <= radiusSquared) {
            gatherRange(middle + 1, end, depth + 1, position, normal,
                        radiusSquared, powerSum);
        }
    } else {
        gatherRange(middle + 1, end, depth + 1, position, normal,
                    radiusSquared, powerSum);
        if (axisDistance * axisDistance <= radiusSquared) {
            gatherRange(begin, middle, depth + 1, position, normal,
                        radiusSquared, powerSum);
        }
    }
}

PhotonMappingIntegrator::PhotonMappingIntegrator(
    const RenderSettings& renderSettings,
    PhotonMapBuildPolicy photonMapBuildPolicy)
    : settings(renderSettings),
      buildPolicy(photonMapBuildPolicy) {}

void PhotonMappingIntegrator::preProcess(const Scene& scene, Sampler& sampler) {
    causticMap.clear();
    globalMap.clear();
    causticMap.reserve(settings.photonCount / 8);
    if (buildPolicy == PhotonMapBuildPolicy::Full)
        globalMap.reserve(settings.photonCount / 2);

    for (std::size_t photonIndex = 0;
         photonIndex < settings.photonCount;
         ++photonIndex) {
        const PhotonEmissionSample emission =
            scene.samplePhotonEmission(settings.photonCount, sampler);

        if (!emission.isValid()) continue;

        tracePhoton(
            Ray(emission.position + emission.direction * EPSILON,
                emission.direction),
            emission.power,
            scene,
            sampler,
            0,
            false,
            false);
    }

    causticMap.balance();
    std::cout << "Caustic photons stored: " << causticMap.size() << "\n";
    if (buildPolicy == PhotonMapBuildPolicy::Full) {
        globalMap.balance();
        std::cout << "Global photons stored: " << globalMap.size() << "\n";
    }
}

void PhotonMappingIntegrator::tracePhoton(
    const Ray& ray,
    const Vector3f& power,
    const Scene& scene,
    Sampler& sampler,
    int depth,
    bool passedThroughSpecular,
    bool hadDiffuseBounce) {

    if (depth >= settings.photonMaxDepth ||
        maxComponent(power) <= EPSILON) {
        return;
    }

    const Intersection hit = scene.intersect(ray);
    if (!hit.happened || hit.m == nullptr || hit.m->hasEmission()) return;

    const Vector3f incoming = -ray.direction;

    if (!hit.m->isSpecular()) {
        if (passedThroughSpecular && !hadDiffuseBounce) {
            causticMap.store(Photon{
                hit.coords,
                incoming,
                normalize(hit.normal),
                power
            });
        }

        if (buildPolicy == PhotonMapBuildPolicy::CausticsOnly) return;

        if (hadDiffuseBounce) {
            globalMap.store(Photon{
                hit.coords,
                incoming,
                normalize(hit.normal),
                power
            });
        }

        if (sampler.randFloat() > settings.rusRoulette) return;

        const BSDFSample scattering = hit.m->sample(incoming, hit.normal, sampler);
        if (!scattering.isValid()) return;

        const Vector3f nextPower = power
                                 * scattering.weight
                                 / settings.rusRoulette;

        tracePhoton(
            Ray(hit.coords + hit.normal * EPSILON, scattering.direction),
            nextPower,
            scene,
            sampler,
            depth + 1,
            false,
            true);
        return;
    }

    const BSDFSample scattering = hit.m->sample(incoming, hit.normal, sampler);
    if (!scattering.isValid()) return;

    const Vector3f nextPower = power * scattering.weight;
    const Vector3f offsetNormal =
        dotProduct(scattering.direction, hit.normal) >= 0.f
        ? hit.normal
        : -hit.normal;

    tracePhoton(
        Ray(hit.coords + offsetNormal * EPSILON, scattering.direction),
        nextPower,
        scene,
        sampler,
        depth + 1,
        passedThroughSpecular || !hadDiffuseBounce,
        hadDiffuseBounce);
}

Vector3f PhotonMappingIntegrator::radiance(
    const Ray& ray,
    const Scene& scene,
    Sampler& sampler,
    int depth) const {

    if (depth > settings.MaxDepth) return Vector3f(0.f);

    const Intersection hit = scene.intersect(ray);
    if (!hit.happened || hit.m == nullptr) return Vector3f(0.f);

    if (hit.m->hasEmission()) {
        const Vector3f emission = hit.m->getEmission();
        if (depth > 0) return emission;

        return Vector3f(
            clamp(0.f, 1.f, emission.x),
            clamp(0.f, 1.f, emission.y),
            clamp(0.f, 1.f, emission.z));
    }

    const Vector3f hitPoint = hit.coords;
    const Vector3f normal = hit.normal;
    const Vector3f outgoing = -ray.direction;

    if (hit.m->isSpecular()) {
        const BSDFSample scattering = hit.m->sample(outgoing, normal, sampler);
        if (!scattering.isValid()) return Vector3f(0.f);

        const Vector3f offsetNormal =
            dotProduct(scattering.direction, normal) >= 0.f
            ? normal
            : -normal;

        return radiance(
            Ray(hitPoint + offsetNormal * EPSILON, scattering.direction),
            scene,
            sampler,
            depth + 1) * scattering.weight;
    }

    Vector3f directLight(0.f);
    Intersection lightPoint;
    float lightPdf = 0.f;
    scene.sampleLight(lightPoint, lightPdf, sampler);

    if (lightPdf > EPSILON) {
        const Vector3f lightOffset = lightPoint.coords - hitPoint;
        const float distanceSquared = dotProduct(lightOffset, lightOffset);

        if (distanceSquared > EPSILON) {
            const float distance = std::sqrt(distanceSquared);
            const Vector3f lightDirection = normalize(lightOffset);
            const Intersection shadowHit = scene.intersect(
                Ray(hitPoint + normal * EPSILON, lightDirection));

            const float distanceDifference = std::abs(
                static_cast<float>(shadowHit.distance) - distance);
            const float surfaceCosine = std::max(
                0.f, dotProduct(lightDirection, normal));
            const float lightCosine = std::max(
                0.f, dotProduct(-lightDirection, lightPoint.normal));

            if (shadowHit.happened && distanceDifference < 0.005f &&
                surfaceCosine > 0.f && lightCosine > 0.f) {
                directLight = lightPoint.emit
                            * hit.m->eval(outgoing, lightDirection, normal)
                            * surfaceCosine
                            * lightCosine
                            / distanceSquared
                            / lightPdf;
            }
        }
    }

    Vector3f result = directLight
                    + estimateGlobal(hit)
                    + estimateCaustic(hit);
    for (int i = 0; i < 3; ++i)
        result[i] = clamp(0, 1, result[i]);

    return result;
}

Vector3f PhotonMappingIntegrator::estimateCaustic(
    const Intersection& hit) const {

    const Vector3f irradiance = causticMap.estimateIrradiance(
        hit.coords,
        hit.normal,
        settings.photonGatherRadius);

    return irradiance * hit.m->Kd / M_PI;
}

Vector3f PhotonMappingIntegrator::estimateGlobal(
    const Intersection& hit) const {

    const Vector3f irradiance = globalMap.estimateIrradiance(
        hit.coords,
        hit.normal,
        settings.photonGatherRadius);

    return irradiance * hit.m->Kd / M_PI;
}
