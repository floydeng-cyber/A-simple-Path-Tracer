#pragma once

#include <cstddef>
#include <vector>

#include "RenderSettings.hpp"
#include "Integrator.hpp"

struct Intersection;

struct Photon {
    Vector3f position;
    Vector3f incoming;
    Vector3f normal;
    Vector3f power;
};

class PhotonMappingIntegrator final : public Integrator {
private:
    void tracePhoton(
        const Ray& ray,
        const Vector3f& power,
        const Scene& scene,
        Sampler& sampler,
        int depth,
        bool hittedDiffuse);

    void buildPhotonTree(std::vector<Photon>& photons,
        std::size_t begin, std::size_t end, int depth);

    Vector3f getPhotonInRadius(const std::vector<Photon>& photons,
        const Intersection& hit) const;

    RenderSettings setting;
    bool trackCausticsOnly;
    std::vector<Photon> causticMap;
    std::vector<Photon> diffuseMap;

    friend class HybridIntegrator;
public:
    explicit PhotonMappingIntegrator(
        const RenderSettings& set,
        bool _trackCausticsOnly = false);

    Vector3f radiance(
        const Ray& ray,
        const Scene& scene,
        Sampler& sampler,
        int depth) const override;

    void buildPhotonMaps(const Scene& scene, Sampler& sampler) override;
};
