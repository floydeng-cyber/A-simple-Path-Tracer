#pragma once

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

class Sampler;

struct BVHBuildNode;
// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel {

public:
    // BVHAccel Public Types
    enum class SplitMethod { NAIVE, SAH };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;

    ~BVHAccel() = default;
    
    BVHAccel(const BVHAccel&) = delete;
    BVHAccel& operator=(const BVHAccel&) = delete;

    Intersection getIntersection(const BVHBuildNode* node, const Ray& ray) const;

    std::unique_ptr<BVHBuildNode> root;

    std::unique_ptr<BVHBuildNode> recursiveBuild(std::vector<Object*> objects);

    Intersection Intersect(const Ray &ray) const;    
    bool IntersectP(const Ray &ray) const;

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;

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

    // Not owning. Let Scene OR MeshTriangle owns the object
    Object* object = nullptr;

    float area = 0.f;
    int splitAxis = 0;
    int firstPrimOffset = 0;
    int nPrimitives = 0;
};