#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "Object.hpp"

namespace SAH {

    struct _Object {   // temp. storage of objects (for concise naming)
        Object* object;
        Bounds3 bound;
        Vector3f centroid;
    };
    struct Bin {    // Section splitted in space
        Bounds3 bound;
        std::size_t count = 0;
    };

inline std::vector<Object*>::iterator axis_split(std::vector<Object*>& objects, const Bounds3& bounds) {
    if (objects.size() < 2) return objects.end();   // end splitting if only 1 object

    constexpr int binCount = 15;    // set 15 temp. bins in space for SAH
    
    std::vector<_Object> _objects;    // create a temp. container for objects
    _objects.reserve(objects.size());
    Bounds3 centroid_bound;             // find the boundary of centroid of all objects
    for (Object* obj : objects) {       // loop through all objects
        _objects.emplace_back(_Object( obj, obj->getBounds(), obj->getBounds().Centroid() ));
        centroid_bound = Union(centroid_bound, obj->getBounds().Centroid());
    }

    // Fit selected object into a bin
    const auto bin_idx = [&](const _Object& obj, int axis)->int {
        const float offset = (obj.centroid[axis] - centroid_bound.pMin[axis])
                           / (centroid_bound.pMax[axis] - centroid_bound.pMin[axis]);
        return static_cast<int>(std::clamp(offset * binCount, 0.f, binCount-1.f));
    };

    double splitCost = std::numeric_limits<double>::infinity();
    int    splitAxis = -1;
    int    splitBin = -1;   // on 'right'-side plane
    const double nodeSA = bounds.SurfaceArea();
    constexpr double C_trav = 1;   // Cost coef. of traversal
    constexpr double C_int = 4;    // Cost coef. of intersection
    const double C_leaf = C_int * objects.size();   // Cost coef. of not-splitting
    
    if (nodeSA > 0.0 && std::isfinite(nodeSA)) { // Check for valid node bound
        for (int axis = 0; axis < 3; ++axis) {  // Check every dimension
            const float extent = centroid_bound.pMax[axis] - centroid_bound.pMin[axis];
            if (!(extent > 0.0) || !std::isfinite(extent)) continue;

            std::array<Bin, binCount> bins;       // array of all bins
            for (const _Object& obj : _objects) { // loop through all objects
                Bin& select_bin = bins[bin_idx(obj, axis)];             // Find the best-fit bin
                select_bin.bound = Union(select_bin.bound, obj.bound);  // Union-fy the bound of bin & object
                ++select_bin.count; // Add a count of obj in the best-fit bin
            }

            std::array<Bounds3, binCount> suf_bounds; // To accumulate bin bound from a reversed order
            suf_bounds.back() = bins.back().bound;    // initialize w/last bin bound
            for (int i = binCount - 2; i >= 0; --i)     // Loop through bins from backward
                suf_bounds[i] = Union(bins[i].bound, suf_bounds[i + 1]); // Union-fy/accumulate

            Bounds3 pre_bounds;
            std::size_t leftCnt = 0, rightCnt = 0;
            double temp_cost;
            for (int split_bin = 0; split_bin < binCount-1; ++split_bin) {  // Loop through 'right'-side until 2nd-last bin's
                pre_bounds = Union(pre_bounds, bins[split_bin].bound); // accumulate 'left'-side bound
                leftCnt += bins[split_bin].count;                     // accumulate 'left'-side obj-count
                rightCnt = _objects.size() - leftCnt;
                if (leftCnt == 0 || rightCnt == 0) continue;    // Skip for empty left-/right-side

                temp_cost = pre_bounds.SurfaceArea() * leftCnt
                          + suf_bounds[split_bin+1].SurfaceArea() * rightCnt;
                if (temp_cost < splitCost) {
                    splitCost = temp_cost;
                    splitAxis = axis;
                    splitBin = split_bin;
                }
            }
        }
    }

    if (splitAxis < 0 || C_leaf <= (C_trav + C_int * splitCost/nodeSA))
        return objects.end(); // No usable or cheaper split

    const auto splitObj = std::partition(_objects.begin(), _objects.end(),
            [&](const _Object& obj) {
                return bin_idx(obj, splitAxis) <= splitBin;
            });

    if (splitObj == _objects.begin() || splitObj == _objects.end()) {   // if invalid splt
        return objects.end();
    }

    const auto split_idx = splitObj - _objects.begin();
    std::transform(_objects.begin(), _objects.end(), objects.begin(),
                    [](const _Object& obj) { return obj.object; });
    return objects.begin() + split_idx;
}

} // namespace SAH