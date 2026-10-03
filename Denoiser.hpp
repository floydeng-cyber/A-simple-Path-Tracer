#pragma once

void denoiser() {
    // Read from an unchanged copy so filtering does not spread.
    const auto original = framebuffer;

    const auto luminance = [](const Vector3f& c) {
        return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
        };

    for (int y = 1; y < setting.height - 1; ++y) {
        for (int x = 1; x < setting.width - 1; ++x) {
            const int index = y * setting.width + x;

            float neighbors[8];
            int n = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    neighbors[n++] = luminance(
                        original[(y + dy) * setting.width + x + dx]);
                }
            }

            std::sort(neighbors, neighbors + 8);

            // Second-brightest neighbor: one other hot pixel cannot
            // protect an isolated pair of bright pixels.
            const float reference = std::max(neighbors[6], 0.f);
            const float current = luminance(original[index]);

            // Only replace a strong, locally unsupported spike.
            if (current > 4.f * reference + 0.1f) {
                framebuffer[index] =
                    original[index] * (reference / current);
            }
        }
    }
}