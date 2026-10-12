#pragma once

#include "RenderSettings.hpp"
#include "Integrator.hpp"

class CelluloidIntegrator final : public Integrator {
private:
    RenderSettings setting;
public:
    explicit CelluloidIntegrator(const RenderSettings& set);

    Vector3f radiance(
        const Ray& ray,
        const Scene& scene,
        Sampler& sampler,
        int depth) const override;
};
