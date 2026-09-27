#pragma once

#include "Vector.hpp"
#include "Sampler.hpp"

enum class ScatterEvent {
    Invalid,
    Diffuse,
    Glossy,
    Reflection,
    Transmission
};

// bidirectional scattering distribution function
struct BSDFSample {
    Vector3f direction = Vector3f(0.f);
    Vector3f weight = Vector3f(0.f);
    float pdf = 0.f;
    ScatterEvent event = ScatterEvent::Invalid;

    bool isValid() const {
        return event != ScatterEvent::Invalid
            && pdf > 0.f
            && direction.norm() > EPSILON;
    }
    
    //bool isSpecular() const {
    //    return event == ScatterEvent::Reflection
    //        || event == ScatterEvent::Transmission;
    //}
};

enum MaterialType {
    DIFFUSE,
    MICROFACET,
    MIRROR,
    DIELECTRIC
};

enum class TransportMode { Radiance, Importance };
struct DielectricBranches {
    BSDFSample reflection;
    BSDFSample transmission;
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

    Vector3f toWorld(const Vector3f &a, const Vector3f &N) const {
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

    float DistRough(float NdotHalf, float roughness) const {
        float rough2 = roughness * roughness;
        float alpha2 = rough2 * rough2;
        float denom1 = 1.f + (NdotHalf*NdotHalf) * (alpha2-1.f);
        return alpha2 / std::max(EPSILON, M_PI * denom1 * denom1);
    }

    float GeoSmith(float NdotView, float NdotLight, float roughness) const {
        auto Schlick = [roughness](float Ndot)->float {
            float r = roughness + 1.f;
            float k = (r * r) / 8.f;
            return Ndot / std::max(EPSILON, Ndot * (1.f - k) + k);
            };

        return Schlick(NdotView) * Schlick(NdotLight);
    }

    Vector3f FresnelSchlick(float cosTheta, const Vector3f& F0) const {
        return F0 + (Vector3f(1.f) - F0) * std::pow(
            1.f - clamp(0, 1, cosTheta),
            5.f);
    }

public:
    DielectricBranches dielectricBranches(const Vector3f& wi, Vector3f n,
        const Vector3f& geometricNormal, TransportMode mode) const {
        // Enter/exit classification must agree with the actual surface.
        if (dotProduct(wi,n)*dotProduct(wi,geometricNormal) <= 0.f)
            n = geometricNormal;
        float f;
        fresnel(-wi,n,ior,f);
        Vector3f r = normalize(reflect(-wi,n));
        Vector3f t = refract(-wi,n,ior);
        // Smooth normals may send a ray through the wrong geometric hemisphere.
        // Use the geometric interface for the entire event in that case.
        const float side = dotProduct(wi,geometricNormal);
        if (side*dotProduct(r,geometricNormal) <= 0.f ||
            (f < 1.f && side*dotProduct(t,geometricNormal) >= 0.f)) {
            n = geometricNormal;
            fresnel(-wi,n,ior,f);
            r = normalize(reflect(-wi,n));
            t = refract(-wi,n,ior);
        }
        const float eta = side > 0.f ? 1.f/ior : ior;
        const float transmissionScale = mode == TransportMode::Radiance ? eta*eta : 1.f;
        return {
            {r, Ks*f, f, ScatterEvent::Reflection},
            {normalize(t), Ks*((1.f-f)*transmissionScale), 1.f-f, ScatterEvent::Transmission}
        };
    }

    BSDFSample sampleSurface(const Vector3f& wi, const Vector3f& n,
        const Vector3f& geometricNormal, Sampler& sampler,
        TransportMode mode = TransportMode::Radiance) const {
        if (m_type != DIELECTRIC) return sample(wi,n,sampler);
        const auto branches = dielectricBranches(wi,n,geometricNormal,mode);
        BSDFSample result = sampler.randFloat() < branches.reflection.pdf
            ? branches.reflection : branches.transmission;
        if (result.pdf > 0.f) result.weight = result.weight / result.pdf;
        return result;
    }

    float roughness = 0.5f;         // 0.05=mirror-like -> 0.8=rough
    Vector3f F0 = Vector3f(0.04f);  // Reflectance @normal-incidence

    MaterialType m_type = DIFFUSE;
    Vector3f m_emission = Vector3f(0.f);
    float ior = 1.5f;
    Vector3f Kd = Vector3f(0.f), Ks = Vector3f(0.f);

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType() const;

    inline bool isSpecular() const {
        return m_type == MIRROR || m_type == DIELECTRIC;
    }

    inline BSDFSample sample(   // sample a ray by Material properties
        const Vector3f& wi,
        const Vector3f& N,
        Sampler& sampler) const;

    inline float pdf(   // given a ray, calculate the PDF of this ray
        const Vector3f& wi,
        const Vector3f& wo,
        const Vector3f& N) const;

    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(
        const Vector3f& wi,
        const Vector3f& wo,
        const Vector3f& N) const;

    inline Vector3f getEmission() const;
    inline bool hasEmission() const;
};

Material::Material(MaterialType type, Vector3f emit)
    : m_type(type), m_emission(emit) {}

MaterialType Material::getType() const { return m_type;}
Vector3f Material::getEmission() const { return m_emission;}
bool Material::hasEmission() const {
    return (m_emission.norm() > EPSILON);
}

BSDFSample Material::sample(const Vector3f& wi, const Vector3f& N, Sampler& sampler) const {
    switch(m_type){
        case DIFFUSE: {
            const float u = sampler.randFloat(), phi = 2.f*M_PI*sampler.randFloat();
            const float r = std::sqrt(u), z = std::sqrt(1.f-u);
            const Vector3f direction = normalize(toWorld(Vector3f(r*std::cos(phi),r*std::sin(phi),z),N));
            return {direction, Kd, z/M_PI, ScatterEvent::Diffuse};
        }
        case MICROFACET: {
            // uniform sample on the hemisphere
            const float x_1 = sampler.randFloat(),
                        x_2 = sampler.randFloat();
            const float z = std::fabs(1.0f - 2.0f * x_1);
            const float r = std::sqrt(1.0f - z * z);
            const float phi = 2.f * M_PI * x_2;

            const Vector3f woLocal(r * std::cos(phi), r * std::sin(phi), z);
            const Vector3f wo = normalize(toWorld(woLocal, N));

            const float samplePDF = pdf(wi, wo, N);
            if (samplePDF <= EPSILON) return {};

            const float cosTheta = std::max(0.f, dotProduct(N, wo));
            const Vector3f bsdfVal = eval(wi, wo, N);
                
            return BSDFSample{
                wo,
                bsdfVal * cosTheta / samplePDF,
                samplePDF,
                (m_type == DIFFUSE) ? ScatterEvent::Diffuse : ScatterEvent::Glossy
            };
        }
        case MIRROR: {
            return BSDFSample{
                normalize(reflect(-wi, N)),
                Ks,
                1.f,
                ScatterEvent::Reflection
            };
        }
        case DIELECTRIC: {
            float PR_reflect = 0.f;
            fresnel(-wi, N, ior, PR_reflect);

            if (sampler.randFloat() < PR_reflect) { // light reflected
                return BSDFSample{
                    normalize(reflect(-wi, N)),
                    Ks,
                    std::max(EPSILON, PR_reflect),
                    ScatterEvent::Reflection
                };
            }

            const Vector3f transmitted = refract(-wi, N, ior);
            if (transmitted.norm2() <= EPSILON*EPSILON) return {};

            return BSDFSample{
                normalize(transmitted),
                Ks,
                std::max(EPSILON, 1.f - PR_reflect),
                ScatterEvent::Transmission
            };
        }
    }
    return {};
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) const {
    if ( !(dotProduct(wo, N) > 0.f) )
        return 0.f;

    switch(m_type){
        case DIFFUSE: {
            // Cosine-weighted hemisphere density.
            return dotProduct(wo,N) / M_PI;
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

Vector3f Material::eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) const {
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

            float Dist = DistRough(NdotHalf, roughness);
            float Geo  = GeoSmith(NdotView, NdotLight, roughness);
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
