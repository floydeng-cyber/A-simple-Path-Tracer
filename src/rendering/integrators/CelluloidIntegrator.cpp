#include "CelluloidIntegrator.hpp"

#include <algorithm>
#include <cmath>

#include "Sampler.hpp"
#include "Scene.hpp"

CelluloidIntegrator::CelluloidIntegrator(const RenderSettings& set)
    : setting(set) {}

Vector3f CelluloidIntegrator::radiance(
    const Ray& ray, const Scene& scene,
    Sampler& /*sampler*/, int depth) const {

    if (depth > setting.MaxDepth) return Vector3f(0.f);

    const Intersection hit = scene.intersect(ray);
    if (!hit.happened || hit.m == nullptr) return Vector3f(0.f);

    if (hit.m->hasEmission())
        return (depth == 0)
            ? hit.m->getEmission().getClamped()
            : hit.m->getEmission();

    const Vector3f N_hit = normalize(hit.normal);
    const Vector3f wo = -normalize(ray.direction);
    //if (dotProduct(N_hit, wo) < 0.f) N_hit = -N_hit;

    // A narrow ink rim on edges of objects, defined by upper bound of grazing angle
    constexpr float outlineThreshold = 0.18f;
    if (std::abs(dotProduct(N_hit, wo)) < outlineThreshold)
        return Vector3f(0.01f);

    // Reuse the same light samples at each hit so bands do not shimmer with SPP
    constexpr int lightSamples = 16;
    Sampler lightSampler(setting.randSeed);
    Vector3f L_dir(0.f);
    for (int i = 0; i < lightSamples; ++i) {
        Intersection sampled_light;
        float pdf_light = 0.f;
        scene.sampleLight(sampled_light, pdf_light, lightSampler);
        if (pdf_light <= 0.f) continue;

        const Vector3f lightOffset = sampled_light.coords - hit.coords;
        const float hit_light_dist2 = lightOffset.norm2();
        if (hit_light_dist2 <= EPSILON * EPSILON) continue;

        const Vector3f ws = normalize(lightOffset);
        const float cos_surface = std::max(0.f, dotProduct(ws, N_hit));
        const float cos_light = std::max(0.f, dotProduct(-ws, sampled_light.normal));
        if (cos_surface <= 0.f || cos_light <= 0.f) continue;

        const Vector3f shadowOrigin = offsetRayOrigin(hit, ws);
        const Vector3f shadowOffset = sampled_light.coords - shadowOrigin;
        const Intersection shadow_hit = scene.intersect(
            Ray(shadowOrigin, normalize(shadowOffset)));
        if (shadow_hit.happened && shadow_hit.distance < shadowOffset.norm() - 5e-3f)
            continue;

        // Unit-albedo diffuse lighting: keep material color out of band selection.
        L_dir += sampled_light.m->getEmission() * (cos_surface * cos_light
            / (M_PI * hit_light_dist2 * pdf_light));
    }
    L_dir = L_dir / static_cast<float>(lightSamples);

    // Quantize the averaged lighting into shadow, middle, and bright paint tones.
    const float brightness = dotProduct(L_dir, Vector3f(0.2126f, 0.7152f, 0.0722f));
    constexpr float shadowThreshold = 0.2f, brightThreshold = 0.6f;
    const float band = brightness < shadowThreshold ? 0.18f
                     : brightness < brightThreshold ? 0.5f : 1.f;

    // This example paints every surface opaque, including existing glass/mirrors.
    const Vector3f base_color = hit.m->Kd.norm2() > EPSILON * EPSILON
        ? hit.m->Kd : (hit.m->isSpecular() ? hit.m->Ks : hit.m->Kd);
    return base_color * band;
}
