#pragma once

#include <cstdint>
#include <random>
#include "Vector.hpp"

class Sampler {
private:
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
		const float x = randFloat();
		const float y = randFloat();
		return Vector2f(x, y);
	}

	static std::uint64_t makeSeed(
		std::uint64_t baseSeed, std::uint64_t pixelIdx, std::uint64_t sampleIdx) {
		return scrambleBits(baseSeed
			 ^ scrambleBits(pixelIdx)
			 ^ scrambleBits(sampleIdx + 0x9e3779b97f4a7c15ULL)
		);
	}
};