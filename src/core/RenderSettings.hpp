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
	RenderMode renderMode = RenderMode::PhotonMapping;
	int		width  = 500;		// Image width  [pixel]
	int		height = 500;		// Image height [pixel]
	int		SPP	   = 100;		// Sample Per Pixel
	int		MaxDepth = 32;		// Allow long internal-reflection chains.
	float	rusRoulette = 0.9f;	// Russian Roulette for light bouncing
	int		savefirstNray = 2;	// Keep first N ray(s) against Russian Roulette
	int		threadCount = 0;	// # of threads (default = 0 = hardware_concurrency)
	float	fov = 40;			// vertical field of view

	std::string outputFile = std::string("render_img/cornellBox_")
						   + renderModeName(renderMode) + ".ppm";

	std::uint64_t randSeed = 42;	// ultimate answer of everything

	std::size_t photonCount = 250000;
	int photonMaxDepth = 32;
	float photonGatherRadius = 30.f;
	int specularSplitDepth = 3; // PM camera paths: trace both Fresnel branches before this depth.
	bool photonConeFilter = true;
	std::uint64_t photonSeed = 42 ^ 0xd1b54a32d192ed03ULL;
	bool smoothMesh = true;

	// Dependent Parameters
	float AspectRatio = (float)width/height;	// Aspect Ratio
};
