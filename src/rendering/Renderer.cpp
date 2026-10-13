#include "Renderer.hpp" // Renderer declarations
#include "Denoiser.hpp"

#include <algorithm>
#include <atomic> // Thread-safe progress counter
#include <chrono> // Timing and progress interval
#include <cmath>
#include <cstdio> // PPM file output
#include <filesystem> // Output directory creation
#include <iostream>
#include <stdexcept> // File error reporting
#include <thread>
#include <vector>

#include "Sampler.hpp"
#include "Scene.hpp"

const float EPSILON = 1e-5f; // Shared ray-offset tolerance

RenderStats Renderer::Render(
    const Scene& scene,
    const Camera& camera,
    const Integrator& integrator,
    const RenderSettings& setting) {

    using vectorRGB = std::vector<Vector3f>;    // type alias for vector<RGB-value>

    using Clock = std::chrono::steady_clock; // Monotonic timer

    vectorRGB framebuffer(setting.width * setting.height); // Pixel colors
    vectorRGB variance(setting.width * setting.height); // color variance (for nonlocal means filter)
    const float sampleWeight = 1.f / static_cast<float>(setting.SPP); // Weight of one sample
    std::cout << "SPP: " << setting.SPP << " | "
              << "Renderer: " << renderModeName(setting.renderMode)
              << "\n"; // Print sample count.

    const int threadCount = (setting.threadCount > 0)
        ? setting.threadCount
        : static_cast<int>(std::max(1u, std::thread::hardware_concurrency())); // Otherwise use the CPU count

    std::atomic<int> completedRows{0}; // Rows completed by all workers
    const auto integrationStart = Clock::now(); // Start integration timer

    auto renderRows = [&](int firstRow) { // Each worker renders every threadCount-th row
        for (int j = firstRow; j < setting.height; j += threadCount) { // Visit this worker's rows
            for (int i = 0; i < setting.width; ++i) {
                const int frameIndex = j * setting.width + i; // Convert 2D position to array index
                Sampler sampler(Sampler::makeSeed(
                    setting.randSeed, static_cast<std::uint64_t>(frameIndex)
                ));

                for (int sample_idx = 0; sample_idx < setting.SPP; ++sample_idx) {
                    sampler.startGrid(sample_idx, setting.SPP);
                    const Vector2f XYdeviation = sampler.randDiffuse2D();
                    const float normalizedX =
                        (static_cast<float>(i) + XYdeviation.x) / setting.width;
                    const float normalizedY =
                        (static_cast<float>(j) + XYdeviation.y) / setting.height;
                    const Ray primaryRay = camera.generateRay(normalizedX, normalizedY); // Create camera ray

                    Vector3f sampleColor = integrator.radiance(
                        primaryRay, scene, sampler, 0);
                    framebuffer[frameIndex] += sampleColor * sampleWeight;
                    variance[frameIndex] += (sampleColor*sampleColor) * sampleWeight;
                }
                if (setting.SPP > 1) {
                    variance[frameIndex] = Vector3f::Max(Vector3f(0.f),
                      (variance[frameIndex] - framebuffer[frameIndex]*framebuffer[frameIndex]
                          ) / float(setting.SPP-1)
                    );
                }
            }
            ++completedRows;
        }
    };

    std::vector<std::thread> threads; // Worker collection
    threads.reserve(threadCount);
    for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
        threads.emplace_back(renderRows, threadIndex); // Construct the thread directly in the vector
    }

    while (completedRows.load() < setting.height) { // Update progress until rendering finishes
        UpdateProgress(completedRows.load() / static_cast<float>(setting.height));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    for (std::thread& worker : threads)
        worker.join(); // Wait for all workers to finish

    UpdateProgress(1.f); // Show 100%
    std::cout << "\n";
    const auto integrationEnd = Clock::now(); // Stop integration timer

    const auto denoiseStart = Clock::now(); // start denosing
    if (setting.denoiserOn) {
        std::cout << "denoising ...\n";
        Denoiser::apply(framebuffer, variance, setting.height, setting.width);
    }
    const auto denoiseEnd = Clock::now();   // end denosing

    const auto imageWriteStart = Clock::now(); // Start output timer

    const std::filesystem::path outputPath(setting.outputFile); // Parse output path
    if (outputPath.has_parent_path()) { // Check for output directory
        std::filesystem::create_directories(outputPath.parent_path()); // Create it when needed
    }

    FILE* output = nullptr; // Output file handle
    fopen_s(&output, setting.outputFile.c_str(), "wb"); // Open binary output
    if (output == nullptr) {
        throw std::runtime_error("Unable to open render output: " + setting.outputFile); // Report failed path
    }

    std::fprintf(output, "P6\n%d %d\n255\n", setting.width, setting.height); // Write PPM header

    const auto toByte = [](float value) { // Convert a color channel to a display byte
        return static_cast<unsigned char>(
            255 * std::pow(clamp(0, 1, value), 0.6f));
    };

    for (const Vector3f& pixel : framebuffer) { // Write each pixel
        const unsigned char color[3] = {
            toByte(pixel.x), toByte(pixel.y), toByte(pixel.z)
        };
        std::fwrite(color, 1, 3, output); // Write three bytes
    }

    std::fclose(output); // Close output file.
    const auto imageWriteEnd = Clock::now(); // Stop output timer

    if (setting.denoiserOn) {
        return RenderStats{ // Return elapsed times
            std::chrono::duration<double>(integrationEnd - integrationStart).count(), // Integration seconds
            std::chrono::duration<double>(imageWriteEnd - imageWriteStart).count(), // Output seconds
            std::chrono::duration<double>(denoiseEnd - denoiseStart).count()    // Denoise seconds
        };
    } else {
        return RenderStats{ // Return elapsed times
            std::chrono::duration<double>(integrationEnd - integrationStart).count(), // Integration seconds
            std::chrono::duration<double>(imageWriteEnd - imageWriteStart).count() // Output seconds
        };
    }
}