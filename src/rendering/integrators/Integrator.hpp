#pragma once

#include "core/Vector.hpp"
#include "geometry/Ray.hpp"

class Scene;
class Sampler;

class Integrator {
public:
	virtual ~Integrator() = default;

	virtual void preProcess(const Scene&, Sampler&) {}

	virtual Vector3f radiance(
		const Ray& ray,
		const Scene& scene,
		Sampler& sampler,
		int depth
	) const = 0;
};
