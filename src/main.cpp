#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>

#include "core/RenderSettings.hpp"
#include "core/Sampler.hpp"
#include "geometry/Triangle.hpp"
#include "rendering/Camera.hpp"
#include "rendering/Renderer.hpp"
#include "rendering/integrators/HybridIntegrator.hpp"
#include "rendering/integrators/PathTracingIntegrator.hpp"
#include "rendering/integrators/PhotonMappingIntegrator.hpp"
#include "scene/Scene.hpp"

int main() {
    using Clock = std::chrono::steady_clock;
    const auto applicationStart = Clock::now();

    RenderSettings settings;

    Camera camera(
        Vector3f(278.0f, 273.0f, -800.0f),
        Vector3f(278.0f, 273.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        settings.fov,
        settings.AspectRatio);

    const auto sceneStart = Clock::now();   // timer starts
    Scene scene;

    Material& red = scene.createMaterial(DIFFUSE);
    red.Kd = Vector3f(0.63f, 0.065f, 0.05f);

    Material& green = scene.createMaterial(DIFFUSE);
    green.Kd = Vector3f(0.14f, 0.45f, 0.091f);

    Material& white = scene.createMaterial(DIFFUSE);
    white.Kd = Vector3f(0.725f, 0.71f, 0.68f);

    Material& light = scene.createMaterial(
        DIFFUSE,
        47.f * Vector3f(1.f, 0.8f, 0.6f));
    light.Kd = Vector3f(0.65f);

    //Material& glossy = scene.createMaterial(MICROFACET);
    //glossy.Kd = Vector3f(0.6f, 0.2f, 0.1f);
    //glossy.F0 = Vector3f(0.04f);
    //glossy.roughness = 0.25f;

    //Material& mirror = scene.createMaterial(MIRROR);
    //mirror.Ks = Vector3f(0.95f);

    Material& sphereMaterial = scene.createMaterial(DIELECTRIC);
    sphereMaterial.Ks = Vector3f(1.0f);
    sphereMaterial.ior = 1.52f;
    scene.addObj<MeshTriangle>(
        "assets/models/complexSpheres.obj",
        &sphereMaterial,
        MeshTriangleOptions{true, settings.smoothMesh});

    scene.addObj<MeshTriangle>("assets/models/cornellbox/floor.obj", &white);
    scene.addObj<MeshTriangle>("assets/models/cornellbox/left.obj", &red);
    scene.addObj<MeshTriangle>("assets/models/cornellbox/right.obj", &green);
    scene.addObj<MeshTriangle>("assets/models/cornellbox/light.obj", &light);
    //scene.addObj<MeshTriangle>("assets/models/cornellbox/shortbox.obj", &white);
    //scene.addObj<MeshTriangle>("assets/models/cornellbox/tallbox.obj", &white);

    const auto sceneLoadEnd = Clock::now();
    scene.buildBVH();
    const auto BVHEnd = Clock::now();

    const auto pipelineStart = Clock::now();
    std::unique_ptr<Integrator> integrator;
    switch (settings.renderMode) {
    case RenderMode::PathTracing:
        integrator = std::make_unique<PathTracingIntegrator>(settings);
        break;
    case RenderMode::PhotonMapping:
        integrator = std::make_unique<PhotonMappingIntegrator>(settings);
        break;
    case RenderMode::Hybrid:
        integrator = std::make_unique<HybridIntegrator>(settings);
        break;
    }

    const auto preprocessStart = Clock::now();
    Sampler preprocessSampler(settings.randSeed);
    integrator->buildPhotonMaps(scene, preprocessSampler);
    const auto preprocessEnd = Clock::now();

    Renderer renderer;
    const RenderStats renderStats = renderer.Render(
        scene, camera, *integrator, settings);
    const auto pipelineEnd = Clock::now();

    const double sceneLoadSeconds =
        std::chrono::duration<double>(sceneLoadEnd - sceneStart).count();
    const double BVHBuildSeconds =
        std::chrono::duration<double>(BVHEnd - sceneLoadEnd).count();
    const double preprocessSeconds =
        std::chrono::duration<double>(preprocessEnd - preprocessStart).count();
    const double pipelineSeconds =
        std::chrono::duration<double>(pipelineEnd - pipelineStart).count();
    const double endToEndSeconds =
        std::chrono::duration<double>(pipelineEnd - applicationStart).count();

    std::cout << std::fixed << std::setprecision(3)
              << "Render complete\n"
              << "  Mode:                  " << renderModeName(settings.renderMode) << "\n"
              << "  Output:                " << settings.outputFile << "\n"
              << "  Scene loading:         " << sceneLoadSeconds << " s\n"
              << "  BVH construction:      " << BVHBuildSeconds << " s\n"
              << "  Integrator preprocess: " << preprocessSeconds << " s\n"
              << "  Pixel computation:     " << renderStats.integrationSeconds << " s\n"
              << "  Image write:           " << renderStats.imageWriteSeconds << " s\n"
              << "  Render pipeline total: " << pipelineSeconds << " s\n"
              << "  End-to-end total:      " << endToEndSeconds << " s\n";

    return 0;
}
