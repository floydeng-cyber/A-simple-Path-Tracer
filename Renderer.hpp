#pragma once
#include "Scene.hpp"
#include "Camera.hpp"
#include "RenderSettings.hpp"
#include "Integrator.hpp"

struct hit_payload {
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer {
public:
    void Render(const Scene& scene,
                const Camera& camera,
                const Integrator& integrator,
                const RenderSettings& setting);
};
