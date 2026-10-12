#pragma once
#include <vector>
#include "Vector.hpp"

class Denoiser {
private:    // denoiser settings
    static constexpr int Np = 5;         // neighorhood: Np x Np [pixel]
    static constexpr float ks = 1.f;    // spatial similarity factor (large ks rejects distant pixels more)
    static constexpr float kr = 1.f;     // color similarity factor (large kr rejects different colors more)

public:
    using vectorRGB = std::vector<Vector3f>;

    static vectorRGB apply(vectorRGB& framebuffer, int height, int width) {
        BilaterialFilter(framebuffer, height, width);
        return framebuffer;
    }

    static void BilaterialFilter(vectorRGB& framebuffer, int height, int width) {
        vectorRGB rawColor = framebuffer;   // preserve raw pixel colors

        // nested loop across the frame buffer rows and columns
        for (int y = 0; y < height; ++y) { for (int x = 0; x < width; ++x) {
            const int pIdx = y * width + x; // central Pixel index
            Vector3f WeightedColor(0.f); // weighted sum: sum w(p,q) C(q)
            float Wp = 0.f;             // sum of weights
            
            // nested loop across neighborhood pixels
            for (int dy = -Np; dy <= Np; ++dy) { for (int dx = -Np; dx <= Np; ++dx) {
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                int qIdx = ny * width + nx;  // THE neighbor Pixel index

                float ds2 = float(dx*dx + dy*dy);                       // ds(p,q)^2 = ||p-q||^2
                float dr2 = (rawColor[pIdx] - rawColor[qIdx]).norm2();  // dr(p,q)^2 = ||C(p)-C(q)||^2
                float w = std::exp(-ks*ks * ds2 - kr*kr * dr2);         // w(p,q) = e^{-d^2(p,q)}

                WeightedColor += w * rawColor[qIdx]; // sum { w(p,q) * C(q) }
                Wp += w;                             // Wp = sum { w(p,q) }
            }}
            framebuffer[pIdx] = WeightedColor / Wp;
        }}
    }
};