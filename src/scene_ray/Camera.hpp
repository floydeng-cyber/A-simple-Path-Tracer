#pragma once
#include "Vector.hpp"
#include "Ray.hpp"

class Camera {
public:
	Camera(const Vector3f& pos,	// position
		   const Vector3f& tar,	// ray-to-target direction
		   const Vector3f& up,	// upward direciton of camera's axis
		   float vFOV,			// vertical field of view
		   float aspectRatio);	// aspect ratio

	Ray generateRay(float norX, float norY) const;

private:
	Vector3f position;	// camera position
	Vector3f target;	// direction to target
	Vector3f upward;	// upwide of camera
	float FOV;			// vertical field of view
	float AR;			// aspect ratio

	Vector3f forward;	// camera ray direction
	Vector3f right;		// right-side of camera
	Vector3f cameraUP;	// upward direction of tilted camera
	float imageScale;	// scaled image due to FOV
};
