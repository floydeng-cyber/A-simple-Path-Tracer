//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "SingletLens.h"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
    // TO DO Implement Path Tracing Algorithm here

    Intersection hit = Scene::intersect(ray);  // Instanciate the intersection (w/ something)

    if ( !hit.happened )        // If not intersecting,
        return Vector3f(0.f);   // return no light/color

    // Store properties of the surface hit
    Vector3f p_hit = hit.coords;        // Point coordinate  of the hit
    Vector3f N_hit = hit.normal;        // Surface normal    of the hit
    Vector3f wo    = -ray.direction;    // Inward ray vector of the hit

    if (hit.m->getType() == DIELECTRIC) {
        if (depth > 8) return Vector3f(0.f);

        Vector3f transmitDir = normalize(refract(ray.direction, N_hit, hit.m->ior));
        Vector3f biasNormal = dotProduct(transmitDir, N_hit) < 0.f ? -N_hit : N_hit;
        return castRay(Ray(p_hit + biasNormal * EPSILON, transmitDir), depth + 1) * hit.m->Ks;
    }

    // For indirect light : Sampling into space randomly
    // ===========================================================================
    Vector3f L_indir(0.f);
    if ( !hit.m->hasEmission() && get_random_float() <= RussianRoulette ) { // Survive the Russian roulette?
        Vector3f wi = normalize( hit.m->sample(wo, N_hit) );

        Ray path_ray(p_hit + N_hit * EPSILON, wi);
        Intersection path_hit = intersect( path_ray );

        float cos_theta = dotProduct(wi, N_hit);
        float pdf_indir = hit.m->pdf(wo, wi, N_hit);
        if ( cos_theta > 0.0f && pdf_indir > EPSILON && path_hit.happened )
            L_indir = castRay(path_ray, depth+1)  // Light      (next object)
                    * hit.m->eval(wo, wi, N_hit) // BRDF        (hit surface)
                    * cos_theta                 // Direct-ness  (ray from hit surface)
                    / (pdf_indir               // PDF           (hit surface)
                    * RussianRoulette);       // Russian Roulette

        // Special case: Hitting a Mirror
        // ===========================================================================
        if ( hit.m->getType() == MIRROR ) {
            for (int i = 0; i < 3; ++i)
                L_indir[i] = clamp(0, 1, L_indir[i]);
            return L_indir;
        }
    }

    // For direct light : Sampling light sources randomly
    // ===========================================================================
    Intersection inter_light;            // Initiate Detection of light source
    float pdf_light = 0.f;               // Initiate Probablity Distribution Function
    sampleLight(inter_light, pdf_light); // Chose one light source (changed inputted values)

    Vector3f p_light = inter_light.coords;  // Point coordinate of light sample
    Vector3f ws = normalize(p_light-p_hit); // Ray: isec point -> light source
    Vector3f NN = inter_light.normal;       // Surface normal on light source
    
    // Shoot a ray: p_hit -> light_source
    // (slight above p_hit to avoid float-precision error)
    Intersection shadow_hit = intersect( Ray(p_hit + N_hit * EPSILON, ws) );

    // Distance difference between shadow ray & actual distance
    float dist_diff = std::abs(shadow_hit.distance - (p_light - p_hit).norm());
    
    Vector3f L_dir(0.f); // Direct light: If shadow ray hits the light source
    if (pdf_light > EPSILON && dist_diff < 0.005f) {
        float hit_light_dist2 = dotProduct(p_light - p_hit, p_light - p_hit);
        L_dir = inter_light.emit              // Emission           (light source)
              * (hit.m->eval(wo, ws, N_hit)) // BRDF                (hit surface)
              * dotProduct(ws, N_hit)       // Direct-ness          (ray onto hit surface)
              * dotProduct(-ws, NN)        // Direct-ness           (ray from area light)
              / hit_light_dist2           // Surface-light distance (squared inverse)
              / pdf_light;               // PDF                     (sampled light source)
    }

    Vector3f shade_color = L_dir + L_indir;
    if (N_hit.y > 0.45f && !hit.m->hasEmission()) {
        for (Object* object : objects) {
            SingletLens* lens = dynamic_cast<SingletLens*>(object);
            if (!lens) continue;

            Vector3f axis = lens->opticalAxis();
            if (axis.y >= -0.2f) continue;

            float focusT = -lens->center.y / axis.y;
            if (focusT <= 0.f) continue;

            Vector3f focus = lens->center + axis * focusT;
            Vector3f focusDelta = p_hit - focus;
            float dist2 = focusDelta.x * focusDelta.x + focusDelta.z * focusDelta.z;
            float spotRadius = std::max(6.f, lens->apertureRadius() * 0.12f);
            float caustic = std::exp(-dist2 / (2.f * spotRadius * spotRadius));
            shade_color += Vector3f(22.0f, 19.0f, 12.0f) * caustic;
        }
    }
    if (hit.m->hasEmission())      // If hitting light source
        shade_color += hit.m->getEmission(); // add emission

    for (int i = 0; i < 3; ++i)
        shade_color[i] = clamp(0, 1, shade_color[i]);
    return shade_color;
}
