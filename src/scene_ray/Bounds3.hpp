#pragma once
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3 {
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3() {
        constexpr float minNum = std::numeric_limits<float>::lowest();
        constexpr float maxNum = std::numeric_limits<float>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p1, const Vector3f p2) {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const {
        Vector3f d = pMax - pMin;
        return 2 * (d.x*d.y + d.x*d.z + d.y*d.z);
    }

    Vector3f Centroid() const { return 0.5f * pMin + 0.5f * pMax; }

    inline const Vector3f& operator[](int i) const {
        return (i == 1) ? pMax : pMin;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const {
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)]

    float t_Min_x = (pMin.x - ray.origin.x) * invDir.x;
    float t_Min_y = (pMin.y - ray.origin.y) * invDir.y;
    float t_Min_z = (pMin.z - ray.origin.z) * invDir.z;
    float t_Max_x = (pMax.x - ray.origin.x) * invDir.x;
    float t_Max_y = (pMax.y - ray.origin.y) * invDir.y;
    float t_Max_z = (pMax.z - ray.origin.z) * invDir.z;

    if (dirIsNeg[0]) {
        float temp = t_Min_x;
        t_Min_x = t_Max_x;
        t_Max_x = temp;
    } if (dirIsNeg[1]) {
        float temp = t_Min_y;
        t_Min_y = t_Max_y;
        t_Max_y = temp;
    } if (dirIsNeg[2]) {
        float temp = t_Min_z;
        t_Min_z = t_Max_z;
        t_Max_z = temp;
    }

    float tEnter = std::max(t_Min_x, std::max(t_Min_y, t_Min_z));
    float tExit =  std::min(t_Max_x, std::min(t_Max_y, t_Max_z));

    if (tExit >= 0 && tEnter <= tExit)
        return true;

    return false;

}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2) {
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p) {
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}
