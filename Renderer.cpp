//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene) {
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 400;
    std::cout << "SPP: " << spp << "\n";

    //                  Without multi-threading ver.
    // =================================================================================
    //for (uint32_t j = 0; j < scene.height; ++j) {
    //    for (uint32_t i = 0; i < scene.width; ++i) {
    //        // generate primary ray direction
    //        float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //            imageAspectRatio * scale;
    //        float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //        Vector3f dir = normalize(Vector3f(-x, y, 1));
    //        for (int k = 0; k < spp; ++k) {
    //            framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
    //        }
    //        m++;
    //    }
    //    if (j % 4 == 1) UpdateProgress(j / (float)scene.height);
    //}
    //UpdateProgress(1.f);
    // =================================================================================

    int num_thread = std::max(1u, std::thread::hardware_concurrency() );
    std::vector<std::thread> threads(num_thread);

    std::atomic<int> completed_row = 0;

    auto idv_thread = [&](int startY, int endY) {
        for (uint32_t j = startY; j < endY; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                int frame_idx = j * scene.width + i;
                float spp_inv = 1.f / spp;
                for (int k = 0; k < spp; ++k) {
                    float dx = get_random_float(), dy = get_random_float();
                    // generate primary ray direction
                    float x = (2 * (i + dx) / (float)scene.width - 1) *
                        imageAspectRatio * scale;
                    float y = (1 - 2 * (j + dy) / (float)scene.height) * scale;

                    Vector3f dir = normalize(Vector3f(-x, y, 1));

                    framebuffer[frame_idx] += scene.castRay(Ray(eye_pos, dir), 0) * spp_inv;
                }
            }
            ++completed_row;
            //if (j % 4 == 1) UpdateProgress(j / (float)scene.height);
        }
        //UpdateProgress(1.f);
        };

    int row_per_thread = scene.height / num_thread;

    for (int i = 0; i < num_thread; ++i) {
        int startY = i * row_per_thread;
        int endY   = (i == num_thread - 1)
                   ? scene.height
                   : startY + row_per_thread;
        
        threads[i] = std::thread(idv_thread, startY, endY);
    }

    float total_progress = 0.f;
    while (completed_row < scene.height) {
        total_progress = completed_row.load() / static_cast<float>(scene.height);

        UpdateProgress(total_progress);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    for (std::thread& thread_progress : threads)
        thread_progress.join(); // Waiting for progress to finish

    UpdateProgress(1.f);

    // save framebuffer to file
    //FILE* fp = fopen("binary.ppm", "wb");
    FILE* fp = nullptr;
    fopen_s(&fp, "cornellBox.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}