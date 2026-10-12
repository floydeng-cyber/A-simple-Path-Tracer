#pragma once

#include <vector>
#include <memory>
#include <ctime>
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Object.hpp"
#include "Ray.hpp"

class Sampler;

struct BVHBuildNode;

class BVHAccel {

public:
    // BVHAccel Public Types
    enum class SplitMethod { NAIVE, SAH };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::SAH);

    ~BVHAccel() = default;
    
    BVHAccel(const BVHAccel&) = delete;
    BVHAccel& operator=(const BVHAccel&) = delete;

    Intersection getIntersection(const BVHBuildNode* node, const Ray& ray) const;

    std::unique_ptr<BVHBuildNode> root;

    std::unique_ptr<BVHBuildNode> recursiveBuild(std::vector<Object*> objects);

    Intersection Intersect(const Ray &ray) const;    

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> temp_objs;

    void Sample(Intersection& pos, float& pdf, Sampler& sampler);
    void getSample(
        const BVHBuildNode* node,
        float samplePos,
        Intersection& pos,
        float& pdf,
        Sampler& sampler);
};

struct BVHBuildNode {
    Bounds3 bounds;
    std::unique_ptr<BVHBuildNode> left;
    std::unique_ptr<BVHBuildNode> right;

    // Scene or MeshTriangle owns the objects
    // Populated for leaf nodes
    std::vector<Object*> objects;

    float area = 0.f;
    int splitAxis = 0;
};
