#include "PathTracingIntegrator.hpp"

#include "RenderSettings.hpp"
#include "Sampler.hpp"
#include "Scene.hpp"

#include <algorithm>
#include <cmath>

namespace {
    float powerWeight(float thisPDF, float otherPDF) {
        const double a = double(thisPDF) * thisPDF;
        const double b = double(otherPDF) * otherPDF;
        return (a + b > 0.0) ? float(a / (a + b)) : 0.f;
    }
}

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
    const Ray& ray, const Intersection& hit,
    const Scene& scene, Sampler& sampler, int depth) const {

    if (!hit.happened || depth > setting.MaxDepth)
        return Vector3f(0.f);

    if (hit.m->hasEmission()) {
        if (dotProduct(hit.geometricNormal, -ray.direction) <= 0.f)
            return Vector3f(0.f);
        return (depth == 0 || depth > 4)
            ? hit.m->getEmission().getClamped()
            : hit.m->getEmission();
    }

    const Vector3f p_hit = hit.coords;
    const Vector3f p_N = hit.normal;
    const Vector3f wo = -ray.direction;
    const float survival = (depth < setting.savefirstN)
        ? 1.f : setting.rusRoulette;

    sampler.updateDepth(depth);
    const BSDFSample bsdf =
        hit.m->sampleSurface(wo, p_N, hit.geometricNormal, sampler);

    // Delta events have no competing area-light sampling technique.
    if (hit.m->isSpecular()) {
        if (!bsdf.isValid() || sampler.randFloat() >= survival)
            return Vector3f(0.f);

        const Ray nextRay(offsetRayOrigin(hit, bsdf.direction), bsdf.direction);
        return radiance(nextRay, scene, sampler, depth + 1)
            * bsdf.weight / survival;
    }

    Vector3f L_dir(0.f);
    Vector3f L_indir(0.f);

    // Direct light Estimator 1: a point on an area light
    // ===========================================================================
    Intersection sampled_light;
    float pdf_areaLight = 0.f;
    scene.sampleLight(sampled_light, pdf_areaLight, sampler);

    if (pdf_areaLight > 0.f && sampled_light.m != nullptr) {
        const Vector3f offset = sampled_light.coords - p_hit;
        const float dist2 = dotProduct(offset, offset);

        if (dist2 > EPSILON*EPSILON) {
            const Vector3f ws = normalize(offset);
            const float cosSurface = std::max(0.f, dotProduct(p_N, ws));
            const float cosLight = std::max(0.f, dotProduct(sampled_light.geometricNormal, -ws));

            if (cosSurface > 0.f && cosLight > 0.f &&
                scene.segmentVisible(hit, sampled_light)) {

                // Convert area PDF to solid-angle PDF
                const float pdf_areaLight_cr = pdf_areaLight * dist2 / cosLight;
                const float pdf_bsdf_cr = hit.m->pdf(wo, ws, p_N);

                L_dir += sampled_light.m->getEmission()
                    * hit.m->eval(wo, ws, p_N)
                    * cosSurface * cosLight / (dist2 * pdf_areaLight)
                    * powerWeight(pdf_areaLight_cr, pdf_bsdf_cr);
            }
        }
    }

    // Direct light Estimator 2: BRDF; ray reused
    // ===========================================================================
    if (bsdf.isValid()) {
        const Ray nextRay(offsetRayOrigin(hit, bsdf.direction), bsdf.direction);
        const Intersection nextHit = scene.intersect(nextRay);

        if (nextHit.happened && nextHit.m->hasEmission()) {
            const float cosLight = std::max(0.f, dotProduct(nextHit.geometricNormal, -bsdf.direction));

            if (cosLight > 0.f) {
                const Vector3f offset = nextHit.coords - p_hit;
                const float dist2 = dotProduct(offset, offset);
                const float pdf_areaLight_cr = scene.areaLightPDF(nextHit) * dist2 / cosLight;

                L_dir += nextHit.m->getEmission() * bsdf.weight
                    * powerWeight(bsdf.pdf, pdf_areaLight_cr);
            }
        }
        else if (nextHit.happened && depth < setting.MaxDepth &&
            sampler.randFloat() < survival) {
            L_indir = radianceAtHit(
                nextRay, nextHit, scene, sampler, depth+1)
                * bsdf.weight / survival;
        }
    }

    return (L_dir + L_indir).getClamped();
}
