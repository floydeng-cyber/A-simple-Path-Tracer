#pragma once
#include "RenderSettings.hpp"
#include "Camera.hpp"
#include "Integrator.hpp"
#include "Scene.hpp"

struct RenderStats {
    double integrationSeconds = 0.0;
    double imageWriteSeconds = 0.0;
    double denoiseSeconds = 0.0;
};

class Renderer {
public:
    RenderStats Render(const Scene& scene,
                       const Camera& camera,
                       const Integrator& integrator,
                       const RenderSettings& setting);
};
