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
    
    if (depth > setting.MaxDepth)
        return Vector3f(0.f);   // max depth reached; return 0's

    const Intersection hit = scene.intersect(ray);  // Instanciate the intersection (w/ something)

    if (!hit.happened)        // If not intersecting,
        return Vector3f(0.f); // return no light/color

    // Store properties of the surface hit
    const Vector3f p_hit = hit.coords;  // Point coordinate  of the hit
    const Vector3f N_hit = hit.normal;  // Surface normal    of the hit
    const Vector3f wo = -ray.direction; // Inward ray vector of the hit

    // For indirect light : Sampling into space randomly
    // ===========================================================================
    Vector3f L_indir(0.f);
    if (depth <= setting.MaxDepth && !hit.m->hasEmission() && // check cur. depth AND emission
        sampler.randFloat() <= setting.rusRoulette) { // Survive the Russian roulette?
        
        const BSDFSample bsdf = hit.m->sampleSurface(wo, N_hit, hit.geometricNormal, sampler);

        if (bsdf.isValid()) {
            const Ray pathRay(offsetRayOrigin(hit, bsdf.direction), bsdf.direction);

            L_indir =
                radiance(pathRay, scene, sampler, depth+1) // Light      (next object)
                * bsdf.weight
                / setting.rusRoulette;
        }
    }

    if (hit.m->isSpecular()) return L_indir;

    // For direct light : Sampling light sources randomly
    // ===========================================================================
    Intersection intersect_light;   // Initiate Detection of light source
    float pdf_light = 0.f;          // Initiate Probablity Distribution Function
    scene.sampleLight(intersect_light, pdf_light, sampler); // Chose one light source (changed inputted values)

    Vector3f L_dir(0.f); // Direct light: If shadow ray hits the light source
    if (pdf_light > EPSILON) {
        const Vector3f lightOffset = intersect_light.coords - p_hit;
        const float hit_light_dist2 = dotProduct(lightOffset, lightOffset);
        const float hit_light_dist = sqrtf(hit_light_dist2);

        if (hit_light_dist2 > EPSILON) {
            const Vector3f ws = normalize(lightOffset);
            const Vector3f NN = intersect_light.normal;

            const Ray shadow_ray(offsetRayOrigin(hit, ws), ws);
            const Intersection shadow_hit = scene.intersect(shadow_ray);

            const float dist_diff = std::abs(
                static_cast<float>(shadow_hit.distance) - hit_light_dist
            );

            const float cos_surface = std::max(0.f, dotProduct(ws, N_hit));
            const float cos_light = std::max(0.f, dotProduct(-ws, NN));

            const bool isVisible = shadow_hit.happened &&
                                   dist_diff < 0.005f;

            if (isVisible && cos_surface > 0.f && cos_light > 0.f)
                L_dir = intersect_light.emit          // Emission           (light source)
                      * (hit.m->eval(wo, ws, N_hit)) // BRDF                (hit surface)
                      * cos_surface                 // Direct-ness          (ray onto hit surface)
                      * cos_light                  // Direct-ness           (ray from area light)
                      / hit_light_dist2           // Surface-light distance (squared inverse)
                      / pdf_light;               // PDF                     (sampled light source)
        }
    }

    Vector3f shade_color = L_dir + L_indir;

    if (hit.m->hasEmission())      // If hitting light source
        shade_color += hit.m->getEmission(); // add emission

    return shade_color.getClamped();
}
