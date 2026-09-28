#pragma once

#include "core/RenderSettings.hpp"
#include "rendering/integrators/Integrator.hpp"
#include "rendering/integrators/PathTracingIntegrator.hpp"
#include "rendering/integrators/PhotonMappingIntegrator.hpp"

class HybridIntegrator final : public Integrator {
public:
    explicit HybridIntegrator(const RenderSettings& settings);

    void preProcess(const Scene& scene, Sampler& sampler) override;

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
