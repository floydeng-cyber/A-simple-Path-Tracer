#pragma once
#include <cstddef>
#include <string>
#include <cstdint>

enum class RenderMode {
	PathTracing,
	PhotonMapping,
	Hybrid
};

inline const char* renderModeName(RenderMode mode) {
	switch (mode) {
	case RenderMode::PathTracing: return "path-tracing";
	case RenderMode::PhotonMapping: return "photon-mapping";
	case RenderMode::Hybrid: return "hybrid";
	}
	return "unknown";
}

struct RenderSettings {
	// Independent Setting Parameters
	RenderMode renderMode = RenderMode::Hybrid;
	int		width  = 500;		// Image width  [pixel]
	int		height = 500;		// Image height [pixel]
	int		SPP	   = 64;		// Sample Per Pixel
	int		MaxDepth = 12;		// Allow long internal-reflection chains.
	float	rusRoulette = 0.85f;	// Russian Roulette for light bouncing
	int		savefirstN = 2;		// Keep first N camera ray(s) against Russian Roulette
	int		threadCount = 0;	// # of threads (default = 0 = hardware_concurrency)
	float	fov = 40;			// vertical field of view

	std::string outputFile = std::string("render_img/cornellBox_")
						   + renderModeName(renderMode) + ".ppm";

	std::uint64_t randSeed = 42;	// ultimate answer of everything

	std::size_t photonCount = 250000;
	float photonGatherRadius = 25.f;
	int specularBothWayDepth = 4;

	// Dependent Parameters
	float AspectRatio = (float)width/height;	// Aspect Ratio
};
