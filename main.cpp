#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>
#include <atomic>

#include "SingletLens.h"
#include "RenderSettings.hpp"
#include "Camera.hpp"
#include "PathTracingIntegrator.hpp"

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv) {
    RenderSettings setting;     // Import Render Settings

    const float aspectRatio =   // aspect ratio
        static_cast<float>(setting.width) / static_cast<float>(setting.height);

    Camera camera(
        Vector3f(278.0f, 273.0f, -800.0f),  // position
        Vector3f(278.0f, 273.0f, 0.0f),     // target
        Vector3f(0.0f, 1.0f, 0.0f),         // upward
        static_cast<float>(setting.fov),    // fov
        aspectRatio);                       // aspect ratio

    Scene scene;  // Create scene

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.65f);

    Material* glossy = new Material(MICROFACET);
    glossy->Kd = Vector3f(0.6f, 0.2f, 0.1f);
    glossy->roughness = 0.25f;
    glossy->F0 = Vector3f(0.04f);

    Material* mirror = new Material(MIRROR);
    mirror->Ks = Vector3f(0.95f);

    // Creating the singlet lens ===============================
    Material* lensMaterial = new Material(DIELECTRIC);
    lensMaterial->Kd = Vector3f(0.55f, 0.82f, 0.95f);
    lensMaterial->Ks = Vector3f(0.92f, 0.97f, 1.0f);
    lensMaterial->ior = 1.52f;

    // Transparent sphere centered in the camera view and clear of the boxes.
    Material* sphereMaterial = new Material(DIELECTRIC);
    sphereMaterial->Ks = Vector3f(1.0f);
    sphereMaterial->ior = 1.52f;
    Sphere transparentSphere(Vector3f(278.f, 273.f, 170.f), 65.f, sphereMaterial);

    MeshTriangle floor      ("models/cornellbox/floor.obj", white);

    MeshTriangle shortbox   ("models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox    ("models/cornellbox/tallbox.obj", white);

    MeshTriangle left       ("models/cornellbox/left.obj", red);
    MeshTriangle right      ("models/cornellbox/right.obj", green);
    MeshTriangle light_     ("models/cornellbox/light.obj", light);

    scene.Add(&floor);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);
    scene.Add(&transparentSphere);

    scene.buildBVH();

    Renderer FloydRender;
    PathTracingIntegrator integrator1(setting);

    auto start = std::chrono::system_clock::now();
    FloydRender.Render(scene, camera, integrator1, setting);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}
