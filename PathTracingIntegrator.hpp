#pragma once
#include "Integrator.hpp"
#include "RenderSettings.hpp"

class PathTracingIntegrator final: public Integrator {
public:
	explicit PathTracingIntegrator(const RenderSettings& set);

	Vector3f radiance(
		const Ray& ray,
		const Scene& scene,
		Sampler& sampler,
		int depth
	) const override;

private:
	RenderSettings setting;
};