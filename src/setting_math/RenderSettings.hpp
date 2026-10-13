#pragma once
#include <cstddef>
#include <string>
#include <cstdint>

enum class RenderMode {
	PathTracing,
	PhotonMapping,
	Hybrid,
	Celluloid
};
inline const char* renderModeName(RenderMode mode) {
	switch (mode) {
	case RenderMode::PathTracing: return "path-tracing";
	case RenderMode::PhotonMapping: return "photon-mapping";
	case RenderMode::Hybrid: return "hybrid";
	case RenderMode::Celluloid: return "celluloid";
	}
	return "unknown";
}

enum class DenoiserMode {
	Bilateral,
	NonlocalMeans
};
inline const char* denoiserModeName(DenoiserMode mode) {
	switch (mode) {
	case DenoiserMode::Bilateral: return "Bilateral";
	case DenoiserMode::NonlocalMeans: return "Nonlocal-means";
	}
	return "unknown";
}

struct RenderSettings {
	RenderMode renderMode = RenderMode::PathTracing;
	int		width  = 500;		 // Image width  [pixel]
	int		height = 500;		 // Image height [pixel]
	int		SPP	   = 64;		 // Sample Per Pixel
	int		MaxDepth = 12;		 // Allow long internal-reflection chains.
	float	rusRoulette = 0.85f; // Russian Roulette for light bouncing
	int		savefirstN = 2;		 // Keep first N camera ray(s) against Russian Roulette
	int		threadCount = 0;	 // # of threads (default = 0 = hardware_concurrency)
	float	fov = 40;			 // vertical field of view

	DenoiserMode denoiserMode = DenoiserMode::NonlocalMeans;
	bool denoiserOn = false;	// switch for denoiser

	std::string outputPath = std::string("assets/images/cornellBox_");
	std::string outputFile = outputPath
						   + renderModeName(renderMode) + ".ppm";

	std::uint64_t randSeed = 42;	// ultimate answer of everything

	std::size_t photonCount = 250000;
	float photonGatherRadius = 25.f;
	int specularBothWayDepth = 4;

	float AspectRatio = (float)width/height;	// Aspect Ratio
};
