#pragma once

#include "Sampler.hpp"
#include "Vector.hpp"
#include "global.hpp"

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
            && pdf > EPSILON
            && direction.norm2() > EPSILON*EPSILON;
    }
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
    Vector3f reflect(const Vector3f& wi, const Vector3f& N) const {
        return wi - 2 * dotProduct(wi, N) * N;
    }

    // Compute refraction direction based on Snell's law
    //    - Ray inside the object   :   Invert the normal & refractive index
    //    - Ray outside the object  :   Invert the normal
    Vector3f refract(const Vector3f& wi, const Vector3f& N, const float& ior) const {
        float cosi = clamp(-1, 1, dotProduct(wi, N));
        float n1 = 1, n2 = ior;
        Vector3f surface_normal = N;
        if (cosi < 0) {
            cosi = -cosi;
        } else {
            std::swap(n1, n2); surface_normal= -N;
        }
        float eta = n1 / n2;
        float k = 1 - eta*eta * (1 - cosi*cosi);    // discriminator
        return (k < 0) ? reflect(wi, surface_normal)  // total internal reflection
                       : eta * wi + (eta * cosi - sqrtf(k)) * surface_normal;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    // \param N is the normal at the intersection point
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const {
        float cosi = clamp(-1.f, 1.f, dotProduct(I, N));
        float n0 = 1, n1 = ior;
        if (cosi > 0.f)   // incoming ray comes beneath surface
            std::swap(n0, n1);

        // Compute sini using Snell's law
        float sint = n0 / n1 * sqrtf(std::max(0.f, 1.f - cosi * cosi));
        // Total internal reflection
        if (sint >= 1.f) {
            kr = 1;
        } else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = std::abs(cosi);
            float Rs = ((n1 * cosi) - (n0 * cost)) / ((n1 * cosi) + (n0 * cost));
            float Rp = ((n0 * cosi) - (n1 * cost)) / ((n0 * cosi) + (n1 * cost));
            kr = (Rs * Rs + Rp * Rp) / 2.f;
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
    DielectricBranches dielectricBranches(
        const Vector3f& wi,
        Vector3f normal,
        const Vector3f& geometricNormal,
        TransportMode mode) const {

        // Enter/exit classification must agree with the actual surface.
        if (dotProduct(wi,normal)*dotProduct(wi,geometricNormal) <= 0.f)
            normal = geometricNormal;
        float reflect_coef;
        fresnel(-wi,normal,ior,reflect_coef);
        Vector3f refl_dir = normalize(reflect(-wi,normal));
        Vector3f trans_dir = refract(-wi,normal,ior);

        // Smooth normals may send a ray through the wrong geometric hemisphere.
        // Use the geometric interface for the entire event in that case.
        const float side = dotProduct(wi,geometricNormal);
        if (side*dotProduct(refl_dir,geometricNormal) <= 0.f ||
           (reflect_coef < 1.f && side*dotProduct(trans_dir,geometricNormal) >= 0.f)) {
            normal = geometricNormal;
            fresnel(-wi,normal,ior,reflect_coef);
            refl_dir = normalize(reflect(-wi,normal));
            trans_dir = refract(-wi,normal,ior);
        }
        const float n = (side > 0.f) ? 1.f/ior : ior;
        const float transmissionScale = (mode == TransportMode::Radiance) ? n*n : 1.f;
        return {
            {refl_dir, Ks*reflect_coef, reflect_coef, ScatterEvent::Reflection},
            {normalize(trans_dir), Ks*((1.f-reflect_coef)*transmissionScale), 1.f-reflect_coef, ScatterEvent::Transmission}
        };
    }

    BSDFSample sampleSurface(const Vector3f& wi,              const Vector3f& normal,
                             const Vector3f& geometricNormal, Sampler& sampler,
                             TransportMode mode = TransportMode::Radiance) const {

        if (m_type != DIELECTRIC)
            return sample(wi,normal,sampler);
        const auto branches = dielectricBranches(wi,normal,geometricNormal,mode);

        const float reflect_coef = branches.reflection.pdf;
        const float sample_reflect = (reflect_coef < EPSILON || reflect_coef > 1.f-EPSILON)
                                   ? reflect_coef
                                   : clamp(0.1f, 0.9f, reflect_coef);

        BSDFSample result = branches.transmission;
        result.pdf = 1.f - sample_reflect;

        if (sampler.randFloat() < sample_reflect) {
            result = branches.reflection;
            result.pdf = sample_reflect;
        }

        if (result.pdf > EPSILON)
            result.weight = result.weight / result.pdf;
        return result;
    }

    float roughness = 0.2f;         // 0.05=mirror-like -> 0.8=rough
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
    return (m_emission.norm2() > EPSILON*EPSILON);
}

BSDFSample Material::sample(const Vector3f& wi, const Vector3f& N, Sampler& sampler) const {
    switch(m_type){
        case DIFFUSE: {
            const Vector2f gridSample = sampler.randDiffuse2D();
            const float phi     = 2.f * M_PI * gridSample.y,
                        radius  = std::sqrtf(gridSample.x),
                        z       = std::sqrtf(1.f - gridSample.x);
            const Vector3f sample_dir = normalize(toWorld(
                Vector3f(radius*std::cos(phi), radius*std::sin(phi), z), N
            ));
            return { sample_dir, Kd, z/M_PI, ScatterEvent::Diffuse };
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
            if (samplePDF < EPSILON) return {};

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
            Vector3f diffuse = Kd / M_PI;
            return diffuse;
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
        }
        case MIRROR: {
            Vector3f reflectDir = normalize( reflect(-wi, N) );
            float align = dotProduct(normalize(wo), reflectDir);

            if (align > 1.f - EPSILON)
                return Ks / std::max(EPSILON, dotProduct(N, wo));

            return Vector3f(0.f);
        }
        case DIELECTRIC: {
            return Ks;
        }
    }
    return Vector3f(0.f);
}
