#pragma once

#include "RenderSettings.hpp"
#include "Integrator.hpp"
#include "PathTracingIntegrator.hpp"
#include "PhotonMappingIntegrator.hpp"

class HybridIntegrator final : public Integrator {
public:
    explicit HybridIntegrator(const RenderSettings& settings);

    void buildPhotonMaps(const Scene& scene, Sampler& sampler) override;

    Vector3f radiance(
        const Ray& ray,
        const Scene& scene,
        Sampler& sampler,
        int depth) const override;

private:
    RenderSettings settings;
    PathTracingIntegrator pathTracer;
    PhotonMappingIntegrator photonMapper;
};
