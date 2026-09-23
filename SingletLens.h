#pragma once

#include "Object.hpp"
#include "Vector.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

class SingletLens : public Object {
public:
	Vector3f center;
	float roc;	// radius of curvature
	float thi;	// center thickness
	float ap;	// lens half-aperture size
	float tiltX;	// radians, positive turns the lens around the world X axis
	float tiltY;	// radians, positive turns the lens around the world Y axis

	Material* m;

	// Default to bi-convex
	SingletLens(const Vector3f& center,
				float radiusOfCurvature,
				float thickness,
				float apertureRadius,
				Material* material,
				float tiltXRadians = 0.f,
				float tiltYRadians = 0.f):
		center(center),
		roc(radiusOfCurvature),
		thi(thickness),
		ap(apertureRadius),
		tiltX(tiltXRadians),
		tiltY(tiltYRadians),
		m(material)
	{}

	Intersection getIntersection(Ray ray) override {
		Intersection result;
		Ray localRay(worldToLocalPoint(ray.origin), worldToLocalVector(ray.direction));

		// centers of curvature of two spherical surfaces
		Vector3f c1(0.f, 0.f, roc - thi / 2.f);
		Vector3f c2(0.f, 0.f, -roc + thi / 2.f);

		auto intersectLensSurface = [&](const Vector3f& sphereCenter, bool isFront) {
			Vector3f rayToSphereOrigin = localRay.origin - sphereCenter;

			float quadracticA = dotProduct(localRay.direction, localRay.direction);
			float quadracticB = 2.f * dotProduct(localRay.direction, rayToSphereOrigin);
			float quadracticC = dotProduct(rayToSphereOrigin, rayToSphereOrigin) - roc * roc;

			float discriminat = quadracticB * quadracticB - 4 * quadracticA * quadracticC;
			if (discriminat < 0) return;	// no intersection occurs

			float sqrtDiscriminat = std::sqrt(discriminat);
			float afterIntersectDistance[2] = {
				(-quadracticB - sqrtDiscriminat) / (2 * quadracticA),
				(-quadracticB + sqrtDiscriminat) / (2 * quadracticA)
			};

			for (float hitDistance : afterIntersectDistance) {
				if (hitDistance < 0.0001f) continue;

				Vector3f intersectPoint = localRay(hitDistance);
				float radialDistance = std::sqrt(
					intersectPoint.x * intersectPoint.x +
					intersectPoint.y * intersectPoint.y
				);
				if (radialDistance > ap) continue; // ray hits outside aperture
				if (isFront && intersectPoint.z > 0.f) continue;
				if (!isFront && intersectPoint.z < 0.f) continue;

				if (!result.happened || hitDistance < result.distance) {
					result.happened = true;
					result.distance = hitDistance;
					result.coords = localToWorldPoint(intersectPoint);
					Vector3f localNormal = normalize(intersectPoint - sphereCenter);
					if (!isFront) localNormal = -localNormal;
					result.normal = normalize(localToWorldVector(localNormal));

					result.obj = this;
					result.m = m;
				}
			}
			};
		intersectLensSurface(c1, true);
		intersectLensSurface(c2, false);
		intersectRim(localRay, result);

		return result;
	}
	
	void Sample(Intersection& pos, float& pdf) override {
		pdf = 0;
	}
	float getArea() override {
		return 0;
	}
	bool hasEmit() override {
		return false;
	}

	bool intersect(const Ray& ray) override {
		return getIntersection(ray).happened;
	}

	bool intersect(const Ray& ray, float& tnear, uint32_t& index) const override {
		Intersection hit = const_cast<SingletLens*>(this)->getIntersection(ray);
		if (!hit.happened) return false;
		tnear = static_cast<float>(hit.distance);
		index = 0;
		return true;
	}

	void getSurfaceProperties(
		const Vector3f& P,
		const Vector3f& I,
		const uint32_t& index,
		const Vector2f& uv,
		Vector3f& N,
		Vector2f& st
	) const override {
		N = Vector3f(0.f);
		st = uv;
	}

	Vector3f evalDiffuseColor(const Vector2f& st) const override {
		return m ? m->Kd : Vector3f(0.f);
	}

	Vector3f opticalAxis() const {
		return normalize(localToWorldVector(Vector3f(0.f, 0.f, 1.f)));
	}

	Vector3f focusPoint(float focalDistance) const {
		return center + opticalAxis() * focalDistance;
	}

	float apertureRadius() const {
		return ap;
	}

	Bounds3 getBounds() override {
		float sag = roc - std::sqrt(std::max(0.f, roc * roc - ap * ap));
		float halfDepth = thi / 2.f + sag;
		Bounds3 bounds;
		for (int xi = -1; xi <= 1; xi += 2) {
			for (int yi = -1; yi <= 1; yi += 2) {
				for (int zi = -1; zi <= 1; zi += 2) {
					Vector3f p = localToWorldPoint(Vector3f(xi * ap, yi * ap, zi * halfDepth));
					bounds = Union(bounds, p);
				}
			}
		}
		return bounds;
	}

private:
	Vector3f localToWorldVector(const Vector3f& v) const {
		float cx = std::cos(tiltX);
		float sx = std::sin(tiltX);
		float cy = std::cos(tiltY);
		float sy = std::sin(tiltY);

		Vector3f afterX(v.x, cx * v.y - sx * v.z, sx * v.y + cx * v.z);
		return Vector3f(cy * afterX.x + sy * afterX.z, afterX.y, -sy * afterX.x + cy * afterX.z);
	}

	Vector3f worldToLocalVector(const Vector3f& v) const {
		float cx = std::cos(tiltX);
		float sx = std::sin(tiltX);
		float cy = std::cos(tiltY);
		float sy = std::sin(tiltY);

		Vector3f afterY(cy * v.x - sy * v.z, v.y, sy * v.x + cy * v.z);
		return Vector3f(afterY.x, cx * afterY.y + sx * afterY.z, -sx * afterY.y + cx * afterY.z);
	}

	Vector3f localToWorldPoint(const Vector3f& p) const {
		return center + localToWorldVector(p);
	}

	Vector3f worldToLocalPoint(const Vector3f& p) const {
		return worldToLocalVector(p - center);
	}

	float frontEdgeZ() const {
		return (roc - thi / 2.f) - std::sqrt(std::max(0.f, roc * roc - ap * ap));
	}

	float backEdgeZ() const {
		return (-roc + thi / 2.f) + std::sqrt(std::max(0.f, roc * roc - ap * ap));
	}

	void intersectRim(const Ray& localRay, Intersection& result) {
		float a = localRay.direction.x * localRay.direction.x + localRay.direction.y * localRay.direction.y;
		if (a < EPSILON) return;

		float b = 2.f * (localRay.origin.x * localRay.direction.x + localRay.origin.y * localRay.direction.y);
		float c = localRay.origin.x * localRay.origin.x + localRay.origin.y * localRay.origin.y - ap * ap;
		float discriminant = b * b - 4.f * a * c;
		if (discriminant < 0.f) return;

		float zMin = std::min(frontEdgeZ(), backEdgeZ());
		float zMax = std::max(frontEdgeZ(), backEdgeZ());
		float sqrtDiscriminant = std::sqrt(discriminant);
		float hitDistances[2] = {
			(-b - sqrtDiscriminant) / (2.f * a),
			(-b + sqrtDiscriminant) / (2.f * a)
		};

		for (float hitDistance : hitDistances) {
			if (hitDistance < 0.0001f) continue;

			Vector3f intersectPoint = localRay(hitDistance);
			if (intersectPoint.z < zMin || intersectPoint.z > zMax) continue;

			if (!result.happened || hitDistance < result.distance) {
				result.happened = true;
				result.distance = hitDistance;
				result.coords = localToWorldPoint(intersectPoint);
				Vector3f localNormal = normalize(Vector3f(intersectPoint.x, intersectPoint.y, 0.f));
				result.normal = normalize(localToWorldVector(localNormal));
				result.obj = this;
				result.m = m;
			}
		}
	}
};
