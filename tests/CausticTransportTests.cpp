#include <iostream>
#include <stdexcept>
#include "geometry/Sphere.hpp"
#include "geometry/Triangle.hpp"
#include "rendering/integrators/PathTracingIntegrator.hpp"
#include "scene/Scene.hpp"
extern const float EPSILON=1e-5f;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
struct Moments{
    double sum=0,square=0;
    void add(float v){sum+=v;square+=double(v)*v;}
    double mean(int n)const{return sum/n;}
    double variance(int n)const{return square/n-mean(n)*mean(n);}
};
int main(){
    Scene scene;
    auto& floor=scene.createMaterial(DIFFUSE);floor.Kd=Vector3f(0.7f);
    auto& glass=scene.createMaterial(DIELECTRIC);glass.Ks=Vector3f(1.f);
    auto& light=scene.createMaterial(DIFFUSE,Vector3f(8.f));
    scene.addObj<Triangle>(Vector3f(-10,0,-10),Vector3f(0,0,10),Vector3f(10,0,-10),&floor);
    scene.addObj<Sphere>(Vector3f(0,3,0),1.5f,&glass);
    scene.addObj<Sphere>(Vector3f(0,6,0),0.5f,&light);scene.buildBVH();
    const Ray camera(Vector3f(0,1,-4),normalize(Vector3f(0,-1,4)));
    require(scene.intersect(camera).m==&floor,"camera hits receiver");
    RenderSettings settings;
    settings.guideCaustics=false;PathTracingIntegrator baseline(settings);
    settings.guideCaustics=true;PathTracingIntegrator targeted(settings);
    constexpr int n=150000;Moments moments[2];
    for(int mode=0;mode<2;++mode){
        Sampler rng(42);
        for(int i=0;i<n;++i){
            auto v=(mode?targeted:baseline).radiance(camera,scene,rng,0);
            require(std::isfinite(v.x)&&v.x>=0.f,"finite nonnegative transport");moments[mode].add(v.x);
        }
    }
    double error=std::sqrt((moments[0].variance(n)+moments[1].variance(n))/n);
    require(std::abs(moments[0].mean(n)-moments[1].mean(n))<6*error,"mean preserved, no double counting");
    require(moments[1].variance(n)<moments[0].variance(n)*0.25,"caustic variance reduced");
    const Ray lightRay(Vector3f(0,6,-2),Vector3f(0,0,1));
    Sampler a(11),b(11);
    require((baseline.radiance(lightRay,scene,a,0)-targeted.radiance(lightRay,scene,b,0)).norm2()==0,"primary light preserved");
    glass.m_type=DIFFUSE;glass.Kd=Vector3f(0.7f);scene.buildBVH();
    for(int i=0;i<1000;++i)
        require((baseline.radiance(camera,scene,a,0)-targeted.radiance(camera,scene,b,0)).norm2()==0,"exact no-specular fallback");
    std::cout<<"Transport tests passed; means="<<moments[0].mean(n)<<","<<moments[1].mean(n)
             <<" standard error="<<error<<" variance ratio="<<moments[1].variance(n)/moments[0].variance(n)<<"\n";

    // A second, overlapping target and a reflective wall exercise the path
    // partition for multiple specular objects and specular-diffuse-specular paths.
    glass.m_type=DIELECTRIC;
    auto& mirror=scene.createMaterial(MIRROR);mirror.Ks=Vector3f(0.9f);
    scene.addObj<Sphere>(Vector3f(1.8f,2.8f,0),0.8f,&mirror);
    scene.addObj<Triangle>(Vector3f(-10,0,8),Vector3f(10,0,8),Vector3f(0,15,8),&floor);
    scene.buildBVH();
    for(int depth:{1,2,5,12}) {
        settings.MaxDepth=depth;
        settings.guideCaustics=false;PathTracingIntegrator reference(settings);
        settings.guideCaustics=true;PathTracingIntegrator candidate(settings);
        Moments check[2];constexpr int count=60000;
        for(int mode=0;mode<2;++mode){
            Sampler rng(571+mode);
            for(int i=0;i<count;++i)
                check[mode].add((mode?candidate:reference).radiance(camera,scene,rng,0).x);
        }
        double se=std::sqrt((check[0].variance(count)+check[1].variance(count))/count);
        require(std::abs(check[0].mean(count)-check[1].mean(count))<6*se+1e-6,
                "multiple targets and depth limits must preserve the mean");
        std::cout<<"Depth "<<depth<<": means="<<check[0].mean(count)<<","<<check[1].mean(count)
                 <<" standard error="<<se<<"\n";
    }
}
