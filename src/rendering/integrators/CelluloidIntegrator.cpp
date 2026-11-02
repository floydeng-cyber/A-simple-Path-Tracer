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

    Vector3f N_hit = normalize(hit.normal);
    const Vector3f wo = -normalize(ray.direction);
    if (dotProduct(N_hit, wo) < 0.f) N_hit = -N_hit;

    // Cel materials stay opaque; Ks supplies the paint color for glass/mirrors.
    const Vector3f base_color = hit.m->Kd.norm2() > EPSILON * EPSILON
        ? hit.m->Kd : (hit.m->isSpecular() ? hit.m->Ks : hit.m->Kd);

    // A narrow ink rim on edges of objects, defined by upper bound of grazing angle
    constexpr float outlineThreshold = 0.18f;
    if (std::abs(dotProduct(N_hit, wo)) < outlineThreshold)
        return base_color * 0.015f + Vector3f(0.003f, 0.004f, 0.008f);

    // Reuse the same light samples at each hit so bands do not shimmer with SPP
    constexpr int lightSamples = 16;
    Sampler lightSampler(setting.randSeed);
    const Vector3f luminance(0.2126f, 0.7152f, 0.0722f);
    Vector3f light_dir(0.f);
    float light_weight = 0.f, diffuse_weight = 0.f, visible_weight = 0.f;
    for (int i = 0; i < lightSamples; ++i) {
        Intersection sampled_light;
        float pdf_light = 0.f;
        scene.sampleLight(sampled_light, pdf_light, lightSampler);
        if (pdf_light <= 0.f || sampled_light.m == nullptr) continue;

        const Vector3f lightOffset = sampled_light.coords - hit.coords;
        const float hit_light_dist2 = lightOffset.norm2();
        if (hit_light_dist2 <= EPSILON * EPSILON) continue;

        const Vector3f ws = normalize(lightOffset);
        const float cos_surface = std::max(0.f, dotProduct(ws, N_hit));
        const float cos_light = std::max(0.f, dotProduct(-ws, sampled_light.normal));
        if (cos_light <= 0.f) continue;

        // Normalize out light intensity, keeping its direction and cast shadows.
        const float weight = dotProduct(sampled_light.m->getEmission(), luminance)
                           * cos_light / (hit_light_dist2 * pdf_light);
        light_dir += ws * weight;
        light_weight += weight;
        diffuse_weight += weight * cos_surface;
        if (cos_surface <= 0.f) continue;

        const Vector3f shadowOrigin = offsetRayOrigin(hit, ws);
        const Vector3f shadowOffset = sampled_light.coords - shadowOrigin;
        const Intersection shadow_hit = scene.intersect(
            Ray(shadowOrigin, normalize(shadowOffset)));
        if (shadow_hit.happened && shadow_hit.distance < shadowOffset.norm() - 5e-3f)
            continue;

        visible_weight += weight * cos_surface;
    }

    const float brightness = light_weight > 0.f ? diffuse_weight / light_weight : 0.f;
    const float visibility = diffuse_weight > 0.f ? visible_weight / diffuse_weight : 0.f;
    constexpr float shadowThreshold = 0.2f, brightThreshold = 0.65f;

    // Cool shadow paint and warm lit paint preserve the material's color identity.
    const float color_max = std::max({base_color.x, base_color.y, base_color.z});
    const Vector3f shadow_color = base_color * Vector3f(0.42f, 0.45f, 0.64f)
                                + Vector3f(0.025f, 0.012f, 0.04f) * color_max;
    const Vector3f middle_color = base_color * 0.82f;
    const Vector3f bright_color = base_color * Vector3f(0.96f, 0.94f, 0.90f);
    Vector3f color = visibility < 0.5f || brightness < shadowThreshold ? shadow_color
                   : brightness < brightThreshold ? middle_color : bright_color;

    // One hard highlight patch on glossy paint, using the same sampled lights.
    if (hit.m->getType() != DIFFUSE && visibility >= 0.5f && brightness >= shadowThreshold) {
        const Vector3f half = normalize(wo + normalize(light_dir));
        const float roughness = clamp(0.f, 1.f, hit.m->roughness);
        const float highlightThreshold = 0.995f - 0.04f * roughness;
        if (dotProduct(N_hit, half) > highlightThreshold)
            color = color * 0.15f + Vector3f(1.f, 0.96f, 0.86f) * 0.85f;
    }
    return color.getClamped();
}
