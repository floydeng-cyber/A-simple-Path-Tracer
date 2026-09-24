#pragma once
#include <string>
#include <cstdint>

struct RenderSettings {
	int width		= 400;		// Image width  [pixel]
	int height		= 400;		// Image height [pixel]
	int SPP			= 40;		// Sample Per Pixel
	int MaxDepth	= 8;		// Max.# of light ray bounce
	float rusRoulette = 0.8f;	// Russian Roulette for light bouncing
	int threadCount	= 0;		// # of threads (default = 0 = hardware_concurrency)
	int fov			= 40;		// vertical field of view
	std::string outputFile = "cornellBox.ppm";	// rendered image name

	std::uint64_t randSeed = 42;	// ultimate answer of everything
};