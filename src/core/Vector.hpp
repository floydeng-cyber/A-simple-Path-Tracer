#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>

class Vector3f {
public:
    float x, y, z;                              // three components
    Vector3f() : x(0), y(0), z(0) {}            // default to 0's
    Vector3f(float val) : x(val), y(val), z(val) {} // default to the identical value
    Vector3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {} // set to defined values

    Vector3f operator * (const float& val) const {
        return Vector3f(x * val, y * val, z * val);   // vector times a scalar
    }
    Vector3f operator / (const float& val) const {
        return Vector3f(x / val, y / val, z / val);   // vector divides by a scalar
    }

    float norm() const { return std::sqrt(x * x + y * y + z * z); } // magnitude
    float norm2() const { return x * x + y * y + z * z; }   // magnitude squared

    Vector3f normalized() {
        float nInv = 1/std::sqrt(x * x + y * y + z * z);
        return Vector3f(x * nInv, y * nInv, z * nInv);      // normalized vector
    }

    Vector3f operator * (const Vector3f &vec) const {       // dot product with another vector
        return Vector3f(x * vec.x, y * vec.y, z * vec.z);
    }
    Vector3f operator - (const Vector3f &v) const {         // minus a vector
        return Vector3f(x - v.x, y - v.y, z - v.z);
    }
    Vector3f operator + (const Vector3f &v) const {         // plus a vector
        return Vector3f(x + v.x, y + v.y, z + v.z);
    }
    Vector3f& operator += (const Vector3f &v) {             // add current vector
        x += v.x, y += v.y, z += v.z; return *this;
    }
    Vector3f operator - () const {      // take the negative
        return Vector3f(-x, -y, -z);
    }

    friend Vector3f operator * (const float &val, const Vector3f &vec) {
        return Vector3f(vec.x * val, vec.y * val, vec.z * val); // now allows 2*vec3f
    }
    friend std::ostream & operator << (std::ostream &os, const Vector3f &v) {
        return os << v.x << ", " << v.y << ", " << v.z;     // allow to be directly printed
    }

    inline float operator[](int index) const { return (&x)[index]; } // access const Vector3f
    inline float& operator[](int index) { return (&x)[index]; }     // overwrite Vector3f

    static Vector3f Min(const Vector3f& vec1, const Vector3f& vec2) {
        return Vector3f(std::min(vec1.x,vec2.x),std::min(vec1.y,vec2.y),std::min(vec1.z,vec2.z));
    }   // minimal value in each of 3 components

    static Vector3f Max(const Vector3f& vec1, const Vector3f& vec2) {
        return Vector3f(std::max(vec1.x, vec2.x),std::max(vec1.y,vec2.y),std::max(vec1.z,vec2.z));
    }   // maximal value in each of 3 components

    Vector3f getClamped() {
        return Vector3f(std::min(1.f, x), std::min(1.f, y), std::min(1.f, z));
    }
};

class Vector2f {    // 2D-version
public:
    float x, y;
    Vector2f() : x(0), y(0) {}
    Vector2f(float val) : x(val), y(val) {}
    Vector2f(float xx, float yy) : x(xx), y(yy) {}
    Vector2f operator * (const float &val) const { return Vector2f(x * val, y * val); }
    Vector2f operator + (const Vector2f &vec) const { return Vector2f(x + vec.x, y + vec.y); }
};

inline Vector3f lerp(const Vector3f &vec1, const Vector3f& vec2, const float &t) {
    return vec1*(1 - t) + vec2*t;
}

inline Vector3f normalize(const Vector3f &vec) {
    float mag2 = vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
    if (mag2 > 0) {
        float invMag = 1 / sqrtf(mag2);
        return Vector3f(vec.x * invMag, vec.y * invMag, vec.z * invMag);
    }

    return vec;
}

inline float dotProduct(const Vector3f& vec1, const Vector3f& vec2) {
    return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z;
}

inline Vector3f crossProduct(const Vector3f& vec1, const Vector3f& vec2) {
    return Vector3f(
            vec1.y * vec2.z - vec1.z * vec2.y,
            vec1.z * vec2.x - vec1.x * vec2.z,
            vec1.x * vec2.y - vec1.y * vec2.x
    );
}