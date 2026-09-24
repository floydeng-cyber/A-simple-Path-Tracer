#include "PhotonMappingIntegrator.hpp"

#include "Scene.hpp"
#include "RenderSettings.hpp"
#include "Sampler.hpp"

PhotonMappingIntegrator::PhotonMappingIntegrator(
    const RenderSettings& set
) : setting(set) {}

void PhotonMappingIntegrator::preProcess(const Scene&, Sampler&) {
    // Future implementation:
    // 1. Emit photons.
    // 2. Trace photons through the scene.
    // 3. Store diffuse-surface photon interactions.
    // 4. Build the photon-map search structure.
}

Vector3f PhotonMappingIntegrator::radiance(
    const Ray&,
    const Scene&,
    Sampler&,
    int) const {

    // Future camera-ray gathering
    return Vector3f(0.f);
}