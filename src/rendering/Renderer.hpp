#pragma once
#include "Scene.hpp"
#include "Camera.hpp"
#include "RenderSettings.hpp"
#include "Integrator.hpp"

struct RenderStats {
    double integrationSeconds = 0.0;
    double imageWriteSeconds = 0.0;
};

class Renderer {
public:
    RenderStats Render(const Scene& scene,
                       const Camera& camera,
                       const Integrator& integrator,
                       const RenderSettings& setting);
};
