#pragma once
#include <vector>
#include "Vector.hpp"

class Denoiser {
public:
    // denoiser settings
    static constexpr int Np = 4;       // neighorhood: Np x Np [pixel]
    static constexpr float ks = 0.2f;  // spatial similarity factor (large ks rejects distant pixels more)
    static constexpr float kr = 1.f;   // color similarity factor (large kr rejects different colors more)
    static constexpr int f = 3;        // path extend: (2f + 1) x (2f + 1)
    static constexpr float kv = 1.f;   // refinement tunning constant

    inline static std::vector<Vector3f> rawColor;
    inline static std::vector<Vector3f> variance;
    inline static int height, width;   // scene parameters [pixel]
    inline static int px, py, qx, qy;
    inline static float ds2;

    using vectorRGB = std::vector<Vector3f>;

    static vectorRGB apply(vectorRGB& framebuffer, vectorRGB& PixelVariance, int sceneHeight, int sceneWidth) {
        rawColor = framebuffer;   // preserve raw pixel colors
        variance = PixelVariance; // pixel color variance
        height = sceneHeight, width = sceneWidth;

        // loop through scene pixels
        for (py = 0; py < height; ++py) { for (px = 0; px < width; ++px) {
            const int pIdx = py * width + px; // central Pixel index
            Vector3f WeightedColor(0.f);   // weighted sum: sum w(p,q) C(q)
            float Wp = 0.f;               // sum of weights
            
            // loop through neighborhood pixels
            for (int dy = -Np; dy <= Np; ++dy) {
                for (int dx = -Np; dx <= Np; ++dx) {
                    qx = px + dx, qy = py + dy;
                    if (qx < 0 || qx >= width || qy < 0 || qy >= height) continue;

                    int qIdx = qy * width + qx;  // THE neighbor Pixel index
                    ds2 = float(dx*dx + dy*dy);  // pixel separation factor: ds(p,q)^2 = ||p-q||^2

                    //float w = BilateralFilter(rawColor[pIdx], rawColor[qIdx]);
                    float w = NonlocalMeansFilter();

                    WeightedColor += w * rawColor[qIdx]; // sum { w(p,q) * C(q) }
                    Wp += w;                             // Wp = sum { w(p,q) }
                }
            }
            framebuffer[pIdx] = WeightedColor / Wp;
        }}

        return framebuffer;
    }

    // return w(p,q) weight for each pixel color
    inline static float BilateralFilter(auto& pColor, auto& qColor) {
        float dr2 = (pColor - qColor).norm2();  // dr(p,q)^2 = ||C(p)-C(q)||^2
        float dp2 = ks*ks * ds2 + kr*kr * dr2;  // d^2(p,q)  = ks^2 ds^2(p,q) + kr^2 dr^2(p,q)
        return std::exp(-dp2);      // w(p,q) = e^{-d^2(p,q)}
    }

    // return w(p,q) weight for each pixel color
    inline static float NonlocalMeansFilter() {
        float w_pq = 0.f;  // weight of each patch pair
        int N = 0;         // number of patch pairs
        for (int aveWeightY = -f; aveWeightY <= f; ++aveWeightY) { for (int aveWeightX = -f; aveWeightX <= f; ++aveWeightX) {
            int p_centerX = px + aveWeightX, p_centerY = py + aveWeightY;
            int q_centerX = qx + aveWeightX, q_centerY = qy + aveWeightY;
            if (p_centerX < 0 || p_centerX >= width || p_centerY < 0 || p_centerY >= height ||
                q_centerX < 0 || q_centerX >= width || q_centerY < 0 || q_centerY >= height) continue;

            float dr2 = 0.f;    // range distance sum for patch
            float noise2 = 0.f; // variance sum (of RGB; for refinement)
            int N_patch = 0;    // patch size for each patch pair
            for (int patchY = -f; patchY <= f; ++patchY) { for (int patchX = -f; patchX <= f; ++patchX) {
                int cur_px = p_centerX + patchX, cur_py = p_centerY + patchY;
                int cur_qx = q_centerX + patchX, cur_qy = q_centerY + patchY;
                if (cur_px < 0 || cur_px >= width || cur_py < 0 || cur_py >= height ||
                    cur_qx < 0 || cur_qx >= width || cur_qy < 0 || cur_qy >= height) continue;

                int cur_pIdx = cur_py * width + cur_px;
                int cur_qIdx = cur_qy * width + cur_qx;
                dr2 += (rawColor[cur_qIdx] - rawColor[cur_pIdx]).norm2();
                
                Vector3f pairVariance = variance[cur_pIdx] + variance[cur_qIdx];
                noise2 += pairVariance.x + pairVariance.y + pairVariance.z;

                ++N_patch;
            }}

            float dp2 = dr2 / N_patch;
            noise2 /= N_patch;
            //float d2 = ks*ks * ds2 + kr*kr * dp2;
            float d2 = ks*ks * ds2 + std::max(0.f, dp2 - noise2)
                / (kv*kv * std::max(1e-8f, noise2));
            w_pq += std::exp(-d2);
            ++N;
        }}
        return w_pq / N;
    }
};