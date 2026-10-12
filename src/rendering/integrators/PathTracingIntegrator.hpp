#pragma once

#include "RenderSettings.hpp"
#include "Integrator.hpp"

class PathTracingIntegrator final: public Integrator {
private:
	RenderSettings setting;
public:
	explicit PathTracingIntegrator(const RenderSettings& set);

	Vector3f radiance(
		const Ray& ray,
		const Scene& scene,
		Sampler& sampler,
		int depth) const override;

	Vector3f radianceAtHit(
		const Ray& ray,
		const struct Intersection& hit,
		const Scene& scene,
		Sampler& sampler,
		int depth) const;
};
