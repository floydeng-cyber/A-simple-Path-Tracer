#pragma once

#include "Integrator.hpp"
#include "RenderSettings.hpp"

class PhotonMappingIntegrator final: public Integrator {
private:
	RenderSettings setting;
public:
	explicit PhotonMappingIntegrator(const RenderSettings& set);

	Vector3f radiance(
		const Ray& ray,
		const Scene& scene,
		Sampler& sampler,
		int depth) const override;

	void preProcess(const Scene& scene, Sampler& sampler) override;
};