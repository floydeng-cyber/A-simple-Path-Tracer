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
	int		width  = 800;		// Image width  [pixel]
	int		height = 800;		// Image height [pixel]
	int		SPP	   = 400;		// Sample Per Pixel
	int		MaxDepth = 8;		// Max.# of light ray bounce
	float	rusRoulette = 0.8f;	// Russian Roulette for light bouncing
	int		threadCount = 0;	// # of threads (default = 0 = hardware_concurrency)
	float	fov = 40;			// vertical field of view

	std::string outputFile = std::string("render_img/cornellBox_")
						   + renderModeName(renderMode) + ".ppm";

	std::uint64_t randSeed = 42;	// ultimate answer of everything

	std::size_t photonCount = 250000;
	int photonMaxDepth = 8;
	float photonGatherRadius = 30.f;

	// Dependent Parameters
	float AspectRatio = (float)width/height;	// Aspect Ratio
};
