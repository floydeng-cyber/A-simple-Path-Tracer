//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType {
    DIFFUSE,
    MICROFACET,
    MIRROR,
    DIELECTRIC
};

class Material {
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        } else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N) {
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)) {
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        } else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

    float DistRough(float& NdotHalf, float& roughness) const {
        float rough2 = roughness * roughness;
        float alpha2 = rough2 * rough2;
        float denom1 = 1.f + (NdotHalf*NdotHalf) * (alpha2-1.f);
        return alpha2 / std::max(EPSILON, M_PI * denom1 * denom1);
    }

    float GeoSmith(float& NdotView, float& NdotLight, float& roughness) {
        auto Schlick = [&](float& Ndot)->float {
            float r = roughness + 1.f;
            float k = (r * r) / 8.f;
            return Ndot / std::max(EPSILON, Ndot * (1.f - k) + k);
            };

        return Schlick(NdotView) * Schlick(NdotLight);
    }

    Vector3f FresnelSchlick(float& cosTheta, const Vector3f& F0) const {
        return F0 + (Vector3f(1.f) - F0) * std::pow(
            1.f - clamp(0, 1, cosTheta),
            5.f);
    }

public:
    float roughness = 0.5f;         // 0.05=mirror-like -> 0.8=rough
    Vector3f F0 = Vector3f(0.04f);  // Reflectance @normal-incidence

    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){ return m_type;}
///Vector3f Material::getColor(){ return m_color;}
Vector3f Material::getEmission() { return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE: {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
            return toWorld(localRay, N);
            break;
        }
        case MICROFACET: {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
            return toWorld(localRay, N);
            break;
        }
        case MIRROR: {
            return reflect(-wi, N);
            break;
        }
        case DIELECTRIC: {
            return normalize(refract(-wi, N, ior));
            break;
        }
    }
    return Vector3f(0.f);
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    if ( !(dotProduct(wo, N) > 0.f) )
        return 0.f;

    switch(m_type){
        case DIFFUSE: {
            // uniform sample probability 1 / (2 * PI)
            return 0.5f / M_PI;
            break;
        }
        case MICROFACET: {
            // uniform sample probability 1 / (2 * PI)
            return 0.5f / M_PI;
            break;
        }
        case MIRROR: {
            return 1.f;
            break;
        }
        case DIELECTRIC: {
            return 1.f;
            break;
        }
    }
    return 0.f;
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    float cosAlpha = dotProduct(N, wo);
    if ( !(cosAlpha > 0.f) )
        return Vector3f(0.f);

    switch(m_type){
        case DIFFUSE: {
            // calculate the contribution of diffuse model
            Vector3f diffuse = Kd / M_PI;
            return diffuse;

            break;
        }
        case MICROFACET: {
            Vector3f half  = normalize(wi + wo);

            float NdotLight   = std::max(0.f, dotProduct(N, wo)); // How direct is light on surface
            float NdotView    = std::max(0.f, dotProduct(N, wi));  // How direct is view on surface
            float NdotHalf    = std::max(0.f, dotProduct(N, half));  // How aligned is facet & normal
            float ViewDotHalf = std::max(0.f, dotProduct(wi, half)); // Angle bet. view & half vector (for Fresnel)

            if (NdotView <= EPSILON || NdotLight <= EPSILON)
                return Vector3f(0.f);

            float    Dist    = DistRough(NdotHalf, roughness);
            float    Geo     = GeoSmith(NdotView, NdotLight, roughness);
            Vector3f Fresnel = FresnelSchlick(ViewDotHalf, F0);

            Vector3f specular = Fresnel * Dist * Geo
                              / std::max(EPSILON, 4.f * NdotView * NdotLight);
            return Kd / M_PI + specular;

            break;
        }
        case MIRROR: {
            Vector3f reflectDir = normalize( reflect(-wi, N) );
            float align = dotProduct(normalize(wo), reflectDir);

            if (align > 1.f - EPSILON)
                return Ks / std::max(EPSILON, dotProduct(N, wo));

            return Vector3f(0.f);

            break;
        }
        case DIELECTRIC: {
            return Ks;
            break;
        }
    }
    return Vector3f(0.f);
}

#endif //RAYTRACING_MATERIAL_H
