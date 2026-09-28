#pragma once
#include "core/RenderSettings.hpp"
#include "rendering/Camera.hpp"
#include "rendering/integrators/Integrator.hpp"
#include "scene/Scene.hpp"

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
