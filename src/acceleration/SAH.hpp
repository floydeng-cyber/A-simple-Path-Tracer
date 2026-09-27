#pragma once
#include <vector>
#include <algorithm>

#include "Object.hpp"

namespace SAH {
	struct SAH_bin {
		Bounds3 binBound;
		int objCount = 0;
	};

	template <typename type>
	auto SAH_axis_split(std::vector<type*>& objects, Bounds3& bounds) {

		// Prepare constants
		// =======================================================================
		constexpr float C_trav = 1.f;				// Cost of Traversal
		constexpr float C_insect = 4.f;				// Cost of Intersection
		constexpr int numOfBins = 15;				// Number of Bins (for SAH)
		constexpr int binEndIdx = numOfBins - 1;	// Index of last bin
		const     int numOfObj = objects.size();	// Number of objects

		// Initialize indicators
		// =======================================================================
		float best_cost = std::numeric_limits<float>::infinity(); // lower cost
		int   best_axis = 0;		// axis to split
		int   best_split = 0;		// [object] index to split (split @its right)

		// Preapre object lists along 3D
		// =======================================================================
		std::vector<std::vector<Object*>> objs_in_3axis(3, objects);

		// Loop through 3D to search for lowest split cost
		for (int dim = 0; dim < 3; ++dim) {
			// step 1: Sort objects spatially, along xyz-axes
			std::sort(objs_in_3axis[dim].begin(), objs_in_3axis[dim].end(), [dim](auto f1, auto f2) {
				return f1->getBounds().Centroid()[dim] < f2->getBounds().Centroid()[dim];
				});

			// step 2: Initialize a list of SAH-bins
			std::vector<SAH_bin> bins(numOfBins);

			// step 3: Loop through every object to assign to a SAH-bin each
			float range_of_bins_inv = bounds.pMax[dim] - bounds.pMin[dim];
			if (range_of_bins_inv < EPSILON) continue;
			range_of_bins_inv = 1.f / range_of_bins_inv;

			for (auto& obj : objs_in_3axis[dim]) {
				int fitted_bin_idx = numOfBins
					* (obj->getBounds().Centroid()[dim] - bounds.pMin[dim])
					* range_of_bins_inv;
				fitted_bin_idx = std::clamp(fitted_bin_idx, 0, binEndIdx);

				// Unionize the bin w/obj; Accumulate obj-count per binn
				bins[fitted_bin_idx].binBound = Union(bins[fitted_bin_idx].binBound, obj->getBounds());
				bins[fitted_bin_idx].objCount += 1;
			}

			// step 4: Loop through every bin to compute for cost each
			auto SA_inv = 1 / bounds.SurfaceArea();	   // Prepare inverse of global surface area of bounds
			std::vector<bool> binEmpt(numOfBins);	   // Check empty bins
			std::vector<int> objCnt(numOfBins, 0);	   // Accumulaed object count per bin
			std::vector<int> objCnt_rev(numOfBins, 0); // Accumulated objects in reversed order

			std::vector<Bounds3> prefix_SA(numOfBins);  // Left-side bound SA
			std::vector<Bounds3> suffix_SA(numOfBins);  // Right-side bound SA

			// Initializations:
			binEmpt[0] = (bins[0].objCount == 0);
			objCnt[0] = bins[0].objCount;
			prefix_SA[0] = bins[0].binBound;
			for (int i = 1; i < numOfBins; ++i) {
				binEmpt[i] = (bins[i].objCount == 0);
				objCnt[i] = objCnt[i - 1] + bins[i].objCount;
				prefix_SA[i] = Union(prefix_SA[i - 1], bins[i].binBound);
			}

			objCnt_rev[binEndIdx] = bins[binEndIdx].objCount;
			suffix_SA[binEndIdx] = bins[binEndIdx].binBound;
			for (int i = binEndIdx - 1; i >= 0; --i) {
				objCnt_rev[i] = objCnt_rev[i + 1] + bins[i].objCount;
				suffix_SA[i] = Union(suffix_SA[i + 1], bins[i].binBound);
			}

			// Loop through every possible split (in between existing bins)
			for (int bin_idx = 1; bin_idx < bins.size() - 1; ++bin_idx) {
				if (objCnt[bin_idx] == 0 || objCnt_rev[bin_idx + 1] == 0)
					continue;
				// Cost of current trail split
				float temp_cost = C_trav + C_insect * (
					(prefix_SA[bin_idx].SurfaceArea() * SA_inv) * objCnt[bin_idx]
					+ (suffix_SA[bin_idx + 1].SurfaceArea() * SA_inv) * objCnt_rev[bin_idx+1]
					);

				// Check if current split produces new lowest cost
				if (temp_cost < best_cost) {
					best_cost = temp_cost;
					best_axis = dim;
					best_split = objCnt[bin_idx];
				}
			}
		}
		if (best_split <= 0 || best_split >= numOfObj) {
			best_axis = bounds.maxExtent();
			best_split = numOfObj / 2;
		}
		objects = objs_in_3axis[best_axis];
		return objects.begin() + best_split;
	}
}