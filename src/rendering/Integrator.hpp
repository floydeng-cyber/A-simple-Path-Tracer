#pragma once

#include "Vector.hpp"
#include "Ray.hpp"

class Scene;
class Sampler;

class Integrator {
public:
	virtual ~Integrator() = default;

	virtual void buildPhotonMaps(const Scene&, Sampler&) {}

	virtual Vector3f radiance(
		const Ray& ray,
		const Scene& scene,
		Sampler& sampler,
		int depth
	) const = 0;
};
