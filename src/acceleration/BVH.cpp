#include <algorithm>
#include <cassert>
#include <cmath>

#include "acceleration/BVH.hpp"
#include "acceleration/SAH.hpp"
#include "core/Sampler.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      temp_objs(std::move(p)) {

    time_t start, stop;
    time(&start);
    if (temp_objs.empty())
        return;

    root = recursiveBuild(temp_objs);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

std::unique_ptr<BVHBuildNode> BVHAccel::recursiveBuild(
    std::vector<Object*> objects) {
    if (objects.empty()) return nullptr;

    auto node = std::make_unique<BVHBuildNode>();
    for (Object* obj : objects) {
        node->bounds = Union(node->bounds, obj->getBounds());
        node->area += obj->getArea();
    }

    if (objects.size() == 1) {  // Only 1 object: create leaf _BVHBuildNode_
        node->objects = std::move(objects);
        return node;
    }

    auto obj_split = objects.end(); // initialize the split ('right'-side split)
    if (splitMethod == SplitMethod::NAIVE) {
        Bounds3 centroidBounds;
        for (Object* obj : objects)
            centroidBounds = Union(centroidBounds, obj->getBounds().Centroid());

        const int widest_axis = centroidBounds.maxExtent();
        obj_split = objects.begin() + objects.size()/2;

        std::nth_element(objects.begin(), obj_split, objects.end(),
            [widest_axis](Object* obj1, Object* obj2) {
                return obj1->getBounds().Centroid()[widest_axis]
                     < obj2->getBounds().Centroid()[widest_axis];
            });
    }
    else {    // SplitMethod::SAH
        obj_split = SAH::axis_split(objects, node->bounds);
    }

    // SAH rejected splitting || partition emptied either side
    if (obj_split == objects.begin() || obj_split == objects.end()) {
        node->objects = std::move(objects);
        return node;
    }

    std::vector<Object*> left_objs(objects.begin(), obj_split);
    std::vector<Object*> right_objs(obj_split, objects.end());

    node->left = recursiveBuild(std::move(left_objs));
    node->right = recursiveBuild(std::move(right_objs));

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const {
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root.get(), ray);
    return isect;
}

Intersection BVHAccel::getIntersection(const BVHBuildNode* node, const Ray& ray) const {
    // Traverse the BVH to find intersection
    Intersection intersect;

    std::array<int, 3> dirIsNeg = {
        ray.direction.x < 0,
        ray.direction.y < 0,
        ray.direction.z < 0,
    };

    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        return intersect;

    if (node->left == nullptr && node->right == nullptr) { // Leaf node
        for (Object* obj : node->objects) {
            const Intersection hit = obj->getIntersection(ray);
            if (hit.happened && hit.distance < intersect.distance)
                intersect = hit;
        }
        return intersect;
    }

    Intersection hitLeft = getIntersection(node->left.get(), ray);
    Intersection hitRight = getIntersection(node->right.get(), ray);

    return (hitLeft.distance < hitRight.distance) ? hitLeft : hitRight;
}


void BVHAccel::getSample(const BVHBuildNode* node,
    float samplePos, Intersection& pos,
    float& pdf, Sampler& sampler) {

    if(node->left == nullptr && node->right == nullptr) {
        Object* selected_obj = nullptr;
        float selected_area = 0.f;
        float cumulative_area = 0.f;

        for (Object* obj : node->objects) {
            const float obj_area = obj->getArea();
            if (!(obj_area > 0.f)) continue;

            // Retain last positive-area object as a rounding fallback
            selected_obj = obj;
            selected_area = obj_area;
            cumulative_area += obj_area;

            if (samplePos < cumulative_area) break;
        }

        if (selected_obj == nullptr) {
            pos = Intersection{};
            pdf = 0.f;
            return;
        }

        selected_obj->Sample(pos, pdf, sampler);
        pdf *= selected_area;
        return;
    }

    if(samplePos < (node->left->area))
        getSample(node->left.get(), samplePos, pos, pdf, sampler);
    else
        getSample(node->right.get(), samplePos - (node->left->area), pos, pdf, sampler);
}

void BVHAccel::Sample(Intersection& pos, float& pdf, Sampler& sampler) {
    if (!root || !(root->area > 0.f) || !std::isfinite(root->area))
        return;

    pos = Intersection{};
    pdf = 0.f;
    const float samplePos = sampler.randFloat() * root->area;

    getSample(root.get(), samplePos, pos, pdf, sampler);
    pdf /= root->area;
}
