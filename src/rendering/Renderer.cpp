#include "Renderer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "Sampler.hpp"
#include "Scene.hpp"

const float EPSILON = 1e-5f;

RenderStats Renderer::Render(
    const Scene& scene,
    const Camera& camera,
    const Integrator& integrator,
    const RenderSettings& setting) {

    using Clock = std::chrono::steady_clock;

    std::vector<Vector3f> framebuffer(setting.width * setting.height);
    const int spp = setting.SPP;
    std::cout << "SPP: " << spp << "\n";

    const int threadCount = (setting.threadCount > 0)
        ? std::max(1, setting.threadCount)
        : static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
    std::vector<std::thread> threads(threadCount);

    std::atomic<int> completedRows = 0;
    std::condition_variable progressChanged;
    std::mutex progressMutex;

    const auto integrationStart = Clock::now();

    auto renderRows = [&](int startY, int endY) {
        for (int j = startY; j < endY; ++j) {
            for (int i = 0; i < setting.width; ++i) {
                const int frameIndex = j * setting.width + i;
                const float inverseSpp = 1.f / static_cast<float>(spp);

                for (int sampleIndex = 0; sampleIndex < spp; ++sampleIndex) {
                    Sampler sampler(Sampler::makeSeed(
                        setting.randSeed,
                        static_cast<std::uint64_t>(frameIndex),
                        static_cast<std::uint64_t>(sampleIndex)));

                    const Vector2f deviation = sampler.rand2D();
                    const float normalizedX =
                        (static_cast<float>(i) + deviation.x) /
                        static_cast<float>(setting.width);
                    const float normalizedY =
                        (static_cast<float>(j) + deviation.y) /
                        static_cast<float>(setting.height);
                    const Ray primaryRay = camera.generateRay(
                        normalizedX, normalizedY);

                    framebuffer[frameIndex] += integrator.radiance(
                        primaryRay, scene, sampler, 0) * inverseSpp;
                }
            }

            const int rowsDone = ++completedRows;
            if (rowsDone == setting.height)
                progressChanged.notify_one();
        }
    };

    const int rowsPerThread = setting.height / threadCount;
    for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
        const int startY = threadIndex * rowsPerThread;
        const int endY = (threadIndex == threadCount - 1)
            ? setting.height
            : startY + rowsPerThread;
        threads[threadIndex] = std::thread(renderRows, startY, endY);
    }

    while (completedRows < setting.height) {
        std::unique_lock<std::mutex> lock(progressMutex);
        progressChanged.wait_for(
            lock,
            std::chrono::milliseconds(100),
            [&] { return completedRows.load() >= setting.height; });
        UpdateProgress(
            completedRows.load() / static_cast<float>(setting.height));
    }

    for (std::thread& worker : threads)
        worker.join();

    UpdateProgress(1.f);
    std::cout << "\n";
    const auto integrationEnd = Clock::now();

    const auto imageWriteStart = Clock::now();
    const std::filesystem::path outputPath(setting.outputFile);
    if (outputPath.has_parent_path())
        std::filesystem::create_directories(outputPath.parent_path());

    FILE* output = nullptr;
    fopen_s(&output, setting.outputFile.c_str(), "wb");
    if (output == nullptr)
        throw std::runtime_error("Unable to open render output: " + setting.outputFile);

    std::fprintf(output, "P6\n%d %d\n255\n", setting.width, setting.height);
    for (const Vector3f& pixel : framebuffer) {
        const unsigned char color[3] = {
            static_cast<unsigned char>(255 * std::pow(clamp(0, 1, pixel.x), 0.6f)),
            static_cast<unsigned char>(255 * std::pow(clamp(0, 1, pixel.y), 0.6f)),
            static_cast<unsigned char>(255 * std::pow(clamp(0, 1, pixel.z), 0.6f))
        };
        std::fwrite(color, 1, 3, output);
    }
    std::fclose(output);
    const auto imageWriteEnd = Clock::now();

    return RenderStats{
        std::chrono::duration<double>(integrationEnd - integrationStart).count(),
        std::chrono::duration<double>(imageWriteEnd - imageWriteStart).count()
    };
}
