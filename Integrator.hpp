#pragma once

#include "Ray.hpp"
#include "Vector.hpp"

class Scene;

class Integrator {
public:
	virtual ~Integrator() = default;
	virtual Vector3f radiance(
		const Ray& ray,
		const Scene& scene,
		int depth
	) const = 0;
};