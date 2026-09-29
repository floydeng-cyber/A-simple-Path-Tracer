#pragma once

#include <cassert>
#include <array>

#include "acceleration/BVH.hpp"
#include "core/Sampler.hpp"
#include "geometry/Intersection.hpp"
#include "geometry/Object.hpp"
#include "scene/Material.hpp"
#include "third_party/ObjLoader.hpp"

class Triangle : public Object {
public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Vector3f normal;
    Vector3f n0, n1, n2;
    float area;
    Material* m;
    bool is2sided = false;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2,
             Material* _m = nullptr, bool _twoSided = false)
        : v0(_v0), v1(_v1), v2(_v2), m(_m), is2sided(_twoSided) {
        e1 = v1 - v0;
        e2 = v2 - v0;
        const Vector3f areaVector = crossProduct(e1, e2);
        normal = normalize(areaVector);
        n0 = n1 = n2 = normal;
        area = areaVector.norm() * 0.5f;
    }
    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2,
             Vector3f _n0, Vector3f _n1, Vector3f _n2,
             Material* _m, bool _twoSided)
        : Triangle(_v0, _v1, _v2, _m, _twoSided) {
        n0 = (_n0.norm2() > EPSILON*EPSILON) ? normalize(_n0) : normal;
        n1 = (_n1.norm2() > EPSILON*EPSILON) ? normalize(_n1) : normal;
        n2 = (_n2.norm2() > EPSILON*EPSILON) ? normalize(_n2) : normal;
        if (dotProduct(n0, normal) < 0.f) n0 = -n0;
        if (dotProduct(n1, normal) < 0.f) n1 = -n1;
        if (dotProduct(n2, normal) < 0.f) n2 = -n2;
    }

    Intersection getIntersection(Ray ray) override;

    Bounds3 getBounds() override;

    void Sample(Intersection& pos, float& pdf, Sampler& sampler){
        // u: Distance along triangle (0 @(vertex0), 1 @base(v1-v2))
        // sqrtf() shifts sampling away from corners
        float u = std::sqrtf(sampler.randFloat()),
              v = sampler.randFloat();  // v: Position along base(v1-v1)

        pos.coords = v0 * (1.f - u)
                   + v1 * (u * (1.f - v))
                   + v2 * (u * v);

        pos.normal = this->normal;
        pos.geometricNormal = normal;
        pos.positionError = positionErrorBound(1.f-u, u*(1.f-v), u*v);
        pos.emit = m->getEmission();

        pdf = 1.f / area;
    }

    float getArea() { return area; }
    bool hasEmit() { return m->hasEmission(); }

    Vector3f shadingNormal(float u, float v) const {
        return normalize(n0 * (1.f - u - v) + n1 * u + n2 * v);
    }
    Vector3f positionErrorBound(float a, float b, float c) const {
        // gamma(7), for barycentric reconstruction in float (PBRT section 3.9).
        constexpr float e = std::numeric_limits<float>::epsilon() * 0.5f;
        constexpr float gamma7 = (7.f * e) / (1.f - 7.f * e);
        Vector3f error;
        for (int i=0; i<3; ++i)
            error[i] = gamma7 * (std::abs(a*v0[i]) + std::abs(b*v1[i]) + std::abs(c*v2[i]));
        return error;
    }
};

struct MeshTriangleOptions {
    bool is2sided = false;
    bool smoothShading = false;
};

class MeshTriangle : public Object {
public:
    MeshTriangle(const std::string& filename, Material *mt,
                 MeshTriangleOptions options = {}) {
        objl::Loader loader;
        loader.LoadFile(filename);
        area = 0;
        m = mt;
        assert(loader.LoadedMeshes.size() == 1);
        const auto& mesh = loader.LoadedMeshes[0];
        triangles.reserve(mesh.Indices.size() / 3);

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (std::size_t i = 0; i + 2 < mesh.Indices.size(); i += 3) {
            std::array<Vector3f, 3> face_vertices;
            std::array<Vector3f, 3> face_normals;

            for (int j = 0; j < 3; j++) {
                const auto& sourceVertex = mesh.Vertices[mesh.Indices[i + j]];
                auto vert = Vector3f(sourceVertex.Position.X,
                                     sourceVertex.Position.Y,
                                     sourceVertex.Position.Z);
                face_vertices[j] = vert;
                face_normals[j] = Vector3f(sourceVertex.Normal.X,
                                           sourceVertex.Normal.Y,
                                           sourceVertex.Normal.Z);

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }

            if (options.smoothShading) {
                triangles.emplace_back(face_vertices[0], face_vertices[1], face_vertices[2],
                                       face_normals[0], face_normals[1], face_normals[2],
                                       mt, options.is2sided);
            } else {
                triangles.emplace_back(face_vertices[0], face_vertices[1],
                                       face_vertices[2], mt, options.is2sided);
            }
        }

        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object*> ptrs;
        for (auto& tri : triangles){
            ptrs.push_back(&tri);
            area += tri.area;
        }
        bvh = std::make_unique<BVHAccel>(std::move(ptrs));
    }

    Bounds3 getBounds() override { return bounding_box; }

    //Vector3f evalDiffuseColor(const Vector2f& st) const {
    //    float scale = 5;
    //    float pattern =
    //        (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
    //    return lerp(Vector3f(0.815, 0.235, 0.031),
    //                Vector3f(0.937, 0.937, 0.231), pattern);
    //}

    Intersection getIntersection(Ray ray) override {
        Intersection intersec;

        if (bvh) {
            intersec = bvh->Intersect(ray);
        }

        return intersec;
    }
    
    void Sample(Intersection& pos, float& pdf, Sampler& sampler) override {
        bvh->Sample(pos, pdf, sampler);
        pos.emit = m->getEmission();
    }
    float getArea() override {
        return area;
    }
    bool hasEmit() override {
        return m->hasEmission();
    }

    Bounds3 bounding_box;
    std::vector<Triangle> triangles;

    std::unique_ptr<BVHAccel> bvh;
    float area = 0.f;
    Material* m = nullptr;
};

inline Bounds3 Triangle::getBounds() { return Union(Bounds3(v0, v1), v2); }

inline Intersection Triangle::getIntersection(Ray ray) {
    Intersection inter;

    if (!is2sided && dotProduct(ray.direction, normal) > 0.f)
        return inter;
    double u, v, t_tmp = 0;
    Vector3f pvec = crossProduct(ray.direction, e2);
    double det = dotProduct(e1, pvec);
    if (fabs(det) < EPSILON)
        return inter;

    double det_inv = 1. / det;
    Vector3f tvec = ray.origin - v0;
    u = dotProduct(tvec, pvec) * det_inv;
    if (u < 0 || u > 1)
        return inter;
    Vector3f qvec = crossProduct(tvec, e1);
    v = dotProduct(ray.direction, qvec) * det_inv;
    if (v < 0 || u + v > 1)
        return inter;
    t_tmp = dotProduct(e2, qvec) * det_inv;

    if (t_tmp <= 0) return inter;

    // find ray triangle intersection

    inter.happened = true;
    const float b1 = static_cast<float>(u), b2 = static_cast<float>(v);
    const float b0 = 1.f - b1 - b2;
    inter.coords = v0*b0 + v1*b1 + v2*b2;
    inter.normal = shadingNormal(static_cast<float>(u), static_cast<float>(v));
    inter.geometricNormal = normal;
    inter.positionError = positionErrorBound(b0, b1, b2);
    inter.distance = t_tmp;
    inter.m = m;

    return inter;
}
