#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>

//inline float deg2rad(const float& deg) { return deg * M_PI/180; }

const float EPSILON = 1e-5;

// The main render function: iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene.
// The content of the framebuffer is saved to a file.
void Renderer::Render(const Scene& scene,
                      const Camera& camera,
                      const Integrator& integrator,
                      const RenderSettings& setting) {
    std::vector<Vector3f> framebuffer(setting.width * setting.height);

    //float scale = tan(deg2rad(setting.fov * 0.5));
    //float imageAspectRatio = setting.width / (float)setting.height;
    //Vector3f eye_pos(278, 273, -800);

    // spp value that determines sample ammount
    int spp = setting.SPP;
    std::cout << "SPP: " << spp << "\n";

    int num_thread = (setting.threadCount > 0) // check whether is default
        ? std::max(1, setting.threadCount)     // setting OR at least one thread
        : std::thread::hardware_concurrency(); // otherwise, using all we got
    std::vector<std::thread> threads(num_thread);

    std::atomic<int> completed_row = 0; // initialize record of progress

    // individual thread, given start row AND end row
    auto idv_thread = [&](int startY, int endY) {
        for (uint32_t j = startY; j < endY; ++j) {
            for (uint32_t i = 0; i < setting.width; ++i) {
                int frame_idx = j * setting.width + i;  // index of current frame
                float spp_inv = 1.f / spp;              // inverse of SPP
                for (int k = 0; k < spp; ++k) {     // loop through SPP
                    float dx = get_random_float(), dy = get_random_float(); // random sampling range
                    // generate primary ray direction
                    const float norX = (static_cast<float>(i) + dx) /
                                        static_cast<float>(setting.width);
                    const float norY = (static_cast<float>(j) + dy) /
                                        static_cast<float>(setting.height);
                    const Ray rayDir = camera.generateRay(norX, norY);
                    framebuffer[frame_idx] += integrator.radiance(rayDir, scene, 0) * spp_inv;
                }
            }
            ++completed_row;
            //if (j % 4 == 1) UpdateProgress(j / (float)scene.height);
        }
        //UpdateProgress(1.f);
        };

    int row_per_thread = setting.height / num_thread; // split rendering among threads

    for (int i = 0; i < num_thread; ++i) {
        int startY = i * row_per_thread;
        int endY   = (i == num_thread-1)
                   ? setting.height
                   : startY + row_per_thread;
        
        threads[i] = std::thread(idv_thread, startY, endY);
    }

    float total_progress = 0.f;
    while (completed_row < setting.height) {
        total_progress = completed_row.load() / static_cast<float>(setting.height);

        UpdateProgress(total_progress);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    for (std::thread& thread_progress : threads)
        thread_progress.join(); // Waiting for progress to finish

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = nullptr;
    fopen_s(&fp, setting.outputFile.c_str(), "wb");

    (void)fprintf(fp,
            "P6\n%d %d\n255\n",
            setting.width, setting.height);

    for (auto i = 0; i < setting.height * setting.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}