#include "PathTracingIntegrator.hpp"

#include "core/RenderSettings.hpp"
#include "core/Sampler.hpp"
#include "scene/Scene.hpp"

#include <algorithm>
#include <cmath>

PathTracingIntegrator::PathTracingIntegrator(
    const RenderSettings& set
) : setting(set) {}

Vector3f PathTracingIntegrator::radiance(
    const Ray& ray,
    const Scene& scene,
    Sampler& sampler,
    int depth) const {

    if (depth > setting.MaxDepth) return Vector3f(0.f); // max depth reached; return 0's
    
    return radianceAtHit(ray, scene.intersect(ray), scene, sampler, depth);
}

Vector3f PathTracingIntegrator::radianceAtHit(
    const Ray& ray,     const Intersection& hit,
    const Scene& scene, Sampler& sampler,           int depth) const {

    if (!hit.happened || depth > setting.MaxDepth)        // If not intersecting,
        return Vector3f(0.f); // return no light/color

    // Store properties of the surface hit
    const Vector3f p_hit = hit.coords;  // Point coordinate  of the hit
    const Vector3f N_hit = hit.normal;  // Surface normal    of the hit
    const Vector3f wo = -ray.direction; // Inward ray vector of the hit

    // For indirect light : Sampling into space randomly
    // ===========================================================================
    Vector3f L_indir(0.f);
    const float roulette_firstNsaved = (depth < setting.savefirstN)
                                       ? 1.f : setting.rusRoulette;
    if (!hit.m->hasEmission() &&    // check cur. depth AND emission
        sampler.randFloat() <= roulette_firstNsaved) { // Survive the Russian roulette?
        
        sampler.setDepth(depth);
        const BSDFSample bsdf = hit.m->sampleSurface(wo, N_hit, hit.geometricNormal, sampler);

        if (bsdf.isValid()) {
            const Ray pathRay(offsetRayOrigin(hit, bsdf.direction), bsdf.direction);

            L_indir =
                radiance(pathRay, scene, sampler, depth+1) // Light      (next object)
                * bsdf.weight
                / roulette_firstNsaved;
        }
    }

    if (hit.m->isSpecular()) return L_indir;

    // For direct light : Sampling light sources randomly
    // ===========================================================================
    Intersection sampled_light;   // Initiate Detection of light source
    float pdf_light = 0.f;          // Initiate Probablity Distribution Function
    scene.sampleLight(sampled_light, pdf_light, sampler); // Area sampling of one light source

    Vector3f L_dir(0.f); // Direct light: If shadow ray hits the light source
    if (pdf_light > EPSILON) {
        const Vector3f lightOffset = sampled_light.coords - p_hit;
        float hit_light_dist2 = dotProduct(lightOffset, lightOffset);

        if (hit_light_dist2 > 2.5e-5f) {    // away enough from sampled light source
            const Vector3f ws = normalize(lightOffset);    // direction towards the light source
            const Vector3f N_light = sampled_light.normal; // Normal of the light source

            const float cos_surface = std::max(0.f, dotProduct(ws, N_hit));
            const float cos_light = std::max(0.f, dotProduct(-ws, N_light));

            if (cos_surface > 0.f && cos_light > 0.f) {
                const Ray shadow_ray(offsetRayOrigin(hit, ws), ws);
                const Intersection shadow_hit = scene.intersect(shadow_ray);

                const float dist_diff = std::abs(
                    static_cast<float>(shadow_hit.distance) - sqrtf(hit_light_dist2)
                );

                if (shadow_hit.happened && (dist_diff < 5e-3f))
                    L_dir = sampled_light.emit            // Emission           (light source)
                          * (hit.m->eval(wo, ws, N_hit)) // BRDF                (hit surface)
                          * cos_surface                 // Direct-ness          (ray onto hit surface)
                          * cos_light                  // Direct-ness           (ray from area light)
                          / hit_light_dist2           // Surface-light distance (squared inverse)
                          / pdf_light;               // PDF                     (sampled light source)
            }
        }
    }

    Vector3f shade_color = L_dir + L_indir;

    if (hit.m->hasEmission())      // If hitting light source
        shade_color += hit.m->getEmission(); // add emission

    return shade_color.getClamped();
}
