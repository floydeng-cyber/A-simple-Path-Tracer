#pragma once

#include <cstdint>
#include <random>
#include "Vector.hpp"

class Sampler {
private:
	// stratify the sampling
	int grid_idx = 0;
	int grid_wid = 1;	// [wid X wid] sampling grid
	int grid_depth = 0;

	std::mt19937_64 engine;
	std::uniform_real_distribution<float> distribution;
	static std::uint64_t scrambleBits(std::uint64_t val) {
		val += 0x9e3779b97f4a7c15ULL;
		val = (val ^(val>>30)) * 0xbf58476d1ce4e5b9ULL;
		val = (val ^(val>>27)) * 0x94d049bb133111ebULL;
		return val^(val>> 31);
	}
public:
	explicit Sampler(std::uint64_t seed) // constructor
		: engine(seed), distribution(0.f, 1.f) {}

	float randFloat() { return distribution(engine); } // get one random number
	Vector2f rand2D() {		// get two random number
		return Vector2f(randFloat(), randFloat());
	}

	static std::uint64_t makeSeed(
		std::uint64_t baseSeed, std::uint64_t pixelIdx) {
		return scrambleBits(baseSeed^scrambleBits(pixelIdx));
	}
	
	void startGrid(int idx, int spp) {
		grid_idx = idx;
		grid_wid = (int)sqrt(spp);
		if (grid_wid*grid_wid != spp) grid_wid = 1;
		grid_depth = 0;
	}
	void updateDepth(int depth) {
		grid_depth = depth;
	}

	Vector2f randDiffuse2D() {
		const Vector2f sample = rand2D();
		if (grid_depth != 0 || grid_wid <= 1) return sample;
		//if (grid_wid <= 1) return sample;

		const float x = (grid_idx % grid_wid + sample.x) / grid_wid;
		const float y = (grid_idx / grid_wid + sample.y) / grid_wid;
		return Vector2f(
			std::min(x, std::nextafterf(1.f, 0.f)),
			std::min(y, std::nextafterf(1.f, 0.f))
		);
	}
};