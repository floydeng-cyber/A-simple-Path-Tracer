#include "Renderer.hpp" // Renderer declarations.

#include <algorithm> // std::max.
#include <atomic> // Thread-safe progress counter.
#include <chrono> // Timing and progress interval.
#include <cmath> // std::pow.
#include <cstdio> // PPM file output.
#include <filesystem> // Output directory creation.
#include <iostream> // Console output.
#include <stdexcept> // File error reporting.
#include <thread> // Worker threads.
#include <vector> // Framebuffer and thread storage.

#include "core/Sampler.hpp" // Random sampling.
#include "scene/Scene.hpp" // Scene data.

const float EPSILON = 1e-5f; // Shared ray-offset tolerance.

RenderStats Renderer::Render( // Renders the scene and returns timing data.
    const Scene& scene, // Scene to render.
    const Camera& camera, // Camera used to create rays.
    const Integrator& integrator, // Light-transport implementation.
    const RenderSettings& setting) { // Render configuration.

    using Clock = std::chrono::steady_clock; // Monotonic timer.

    std::vector<Vector3f> framebuffer(setting.width * setting.height); // Pixel colors.
    const float sampleWeight = 1.f / static_cast<float>(setting.SPP); // Weight of one sample.
    std::cout << "SPP: " << setting.SPP << "\n"; // Print sample count.

    const int threadCount = setting.threadCount > 0 // Use requested threads when provided.
        ? setting.threadCount // Explicit thread count.
        : static_cast<int>(std::max(1u, std::thread::hardware_concurrency())); // Otherwise use the CPU count.

    std::atomic<int> completedRows{0}; // Rows completed by all workers
    const auto integrationStart = Clock::now(); // Start integration timer

    auto renderRows = [&](int firstRow) { // Each worker renders every threadCount-th row
        for (int j = firstRow; j < setting.height; j += threadCount) { // Visit this worker's rows
            for (int i = 0; i < setting.width; ++i) { // Visit each pixel in the row
                const int frameIndex = j * setting.width + i; // Convert 2D position to array index
                Sampler sampler(Sampler::makeSeed(
                    setting.randSeed, static_cast<std::uint64_t>(frameIndex)
                ));

                for (int sample_idx = 0; sample_idx < setting.SPP; ++sample_idx) { // Trace all pixel samples
                    sampler.startGrid(sample_idx, setting.SPP);
                    const Vector2f XYdeviation = sampler.randDiffuse2D(); // Random subpixel offset
                    const float normalizedX = // Horizontal camera coordinate
                        (static_cast<float>(i) + XYdeviation.x) / setting.width; // Normalize jittered x
                    const float normalizedY = // Vertical camera coordinate
                        (static_cast<float>(j) + XYdeviation.y) / setting.height; // Normalize jittered y
                    const Ray primaryRay = camera.generateRay(normalizedX, normalizedY); // Create camera ray

                    framebuffer[frameIndex] += // Accumulate the averaged sample.
                        integrator.radiance(primaryRay, scene, sampler, 0) * sampleWeight; // Evaluate radiance
                } // End sampling loop
            } // End pixel loop

            ++completedRows; // Acculmulate finished row
        } // End row loop
    }; // End worker function

    std::vector<std::thread> threads; // Worker collection.
    threads.reserve(threadCount); // Avoid vector reallocations.
    for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) { // Start each worker.
        threads.emplace_back(renderRows, threadIndex); // Construct the thread directly in the vector.
    } // End worker-start loop.

    while (completedRows.load() < setting.height) { // Update progress until rendering finishes.
        UpdateProgress(completedRows.load() / static_cast<float>(setting.height)); // Show completion fraction.
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Refresh per second.
    } // End progress loop.

    for (std::thread& worker : threads) { // Visit every worker.
        worker.join(); // Wait for it to finish.
    } // End join loop.

    UpdateProgress(1.f); // Show 100%.
    std::cout << "\n"; // Finish the progress line.
    const auto integrationEnd = Clock::now(); // Stop integration timer.

    const auto imageWriteStart = Clock::now(); // Start output timer.

    const std::filesystem::path outputPath(setting.outputFile); // Parse output path.
    if (outputPath.has_parent_path()) { // Check for output directory
        std::filesystem::create_directories(outputPath.parent_path()); // Create it when needed.
    }

    FILE* output = nullptr; // Output file handle.
    fopen_s(&output, setting.outputFile.c_str(), "wb"); // Open binary output
    if (output == nullptr) {
        throw std::runtime_error("Unable to open render output: " + setting.outputFile); // Report failed path
    }

    std::fprintf(output, "P6\n%d %d\n255\n", setting.width, setting.height); // Write PPM header

    const auto toByte = [](float value) { // Convert a color channel to a display byte
        return static_cast<unsigned char>(
            255 * std::pow(clamp(0, 1, value), 0.6f));
    };

    for (const Vector3f& pixel : framebuffer) { // Write each pixel.
        const unsigned char color[3] = { // RGB byte triplet.
            toByte(pixel.x), toByte(pixel.y), toByte(pixel.z) // Convert all channels.
        }; // End RGB array.
        std::fwrite(color, 1, 3, output); // Write three bytes.
    } // End pixel-write loop.

    std::fclose(output); // Close output file.
    const auto imageWriteEnd = Clock::now(); // Stop output timer.

    return RenderStats{ // Return elapsed times.
        std::chrono::duration<double>(integrationEnd - integrationStart).count(), // Integration seconds.
        std::chrono::duration<double>(imageWriteEnd - imageWriteStart).count() // Output seconds.
    }; // End result.
} // End render function.
