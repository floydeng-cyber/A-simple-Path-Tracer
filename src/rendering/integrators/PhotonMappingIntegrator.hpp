#pragma once

#include <cstddef>
#include <vector>

#include "core/RenderSettings.hpp"
#include "rendering/integrators/Integrator.hpp"

struct Intersection;

struct Photon {
    Vector3f position;
    Vector3f incoming;
    Vector3f normal;
    Vector3f power;
};

enum class PhotonMapBuildPolicy {
    Full,
    CausticsOnly
};

class PhotonMap {
public:
    void reserve(std::size_t count);
    void clear();
    void store(const Photon& photon);
    void balance();

    Vector3f estimateIrradiance(
        const Vector3f& position,
        const Vector3f& normal,
        float radius, bool coneFilter = true) const;

    std::size_t size() const { return photons.size(); }

private:
    void balanceRange(std::size_t begin, std::size_t end, int depth);
    void gatherRange(
        std::size_t begin,
        std::size_t end,
        int depth,
        const Vector3f& position,
        const Vector3f& normal,
        float radiusSquared,
        bool coneFilter,
        Vector3f& powerSum) const;

    std::vector<Photon> photons;
};

class PhotonMappingIntegrator final : public Integrator {
public:
    explicit PhotonMappingIntegrator(
        const RenderSettings& settings,
        PhotonMapBuildPolicy buildPolicy = PhotonMapBuildPolicy::Full);

    Vector3f radiance(
        const Ray& ray,
        const Scene& scene,
        Sampler& sampler,
        int depth) const override;

    void preProcess(const Scene& scene, Sampler& sampler) override;

    Vector3f estimateCaustic(const Intersection& hit) const;
    Vector3f estimateGlobal(const Intersection& hit) const;

private:
    void tracePhoton(
        const Ray& ray,
        const Vector3f& power,
        const Scene& scene,
        Sampler& sampler,
        int depth,
        bool passedThroughSpecular,
        bool hadDiffuseBounce);

    RenderSettings settings;
    PhotonMapBuildPolicy buildPolicy;
    PhotonMap causticMap;
    PhotonMap globalMap;
};
