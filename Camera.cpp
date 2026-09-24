#include "Camera.hpp"

#include <cmath>

constexpr float camPI = 3.141592653589793f;
inline float deg2rad(const float& deg) { return deg * camPI / 180; }

Camera::Camera(const Vector3f& pos, // position
			   const Vector3f& tar, // ray-to-target direction
			   const Vector3f& up,	// upward direciton of camera's axis
			   float vFOV,			// vertical field of view
			   float aspectRatio)	// aspect ratio)
	:	position(pos),
		target(tar),
		upward(up),
		FOV(vFOV),
		AR(aspectRatio){
	forward = normalize(target - position);
	right = normalize(crossProduct(forward, upward));
	cameraUP = normalize(crossProduct(right, forward));
	imageScale = std::tan(deg2rad(FOV * 0.5f));
}

// normalized X and Y for the pixel
Ray Camera::generateRay(float norX, float norY) const {
	// (1,0) -> (2,0) -> (-1,1) -> (-ar,ar) -> scale(-ar,ar)
	const float screenX = (2.f * norX - 1.f) * AR * imageScale;
	// similarly, but flip due to screen orientation
	const float screenY = (1.f - 2.f * norY) * imageScale;

	const Vector3f direction = normalize(
		forward + screenX * right + screenY * cameraUP
	);

	return Ray(position, direction);
}