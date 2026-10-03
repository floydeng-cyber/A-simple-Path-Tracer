#include <cmath>
#include <iostream>
#include <stdexcept>
#include "geometry/Sphere.hpp"
#include "scene/Scene.hpp"
extern const float EPSILON = 1e-5f;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){
    Scene empty;
    Sampler rng(1729);
    require(!empty.hasSpecularObjects(),"empty scene");
    require(empty.specularDirectionPdf(Vector3f(0),Vector3f(0,0,1))==0.f,"empty PDF");
    Scene scene;
    auto& glass=scene.createMaterial(DIELECTRIC);glass.Ks=Vector3f(1.f);
    scene.addObj<Sphere>(Vector3f(0,0,4),1.f,&glass);scene.buildBVH();
    const Vector3f origin(0.f),north(0,0,1);
    const float expected=1.f/(2.f*M_PI*(1.f-std::sqrt(15.f/16.f)));
    require(std::abs(scene.specularDirectionPdf(origin,north)/expected-1.f)<1e-5f,"solid angle PDF");
    scene.addObj<Sphere>(Vector3f(0,0,8),2.f,&glass);scene.buildBVH();
    require(std::abs(scene.specularDirectionPdf(origin,north)/expected-1.f)<1e-5f,"overlap PDF");
    scene.addObj<Sphere>(Vector3f(4,0,0),2.f,&glass);scene.buildBVH();
    constexpr int samples=400000;
    double integral=0,furnace=0;
    Material diffuse(DIFFUSE);diffuse.Kd=Vector3f(0.7f);
    for(int i=0;i<samples;++i){
        auto direction=scene.sampleSpecularDirection(origin,rng,i%8,8);
        require(std::abs(direction.norm2()-1.f)<1e-5f &&
                scene.specularDirectionPdf(origin,direction)>0.f,"sample/PDF agreement");
        float z=1.f-2.f*rng.randFloat(),phi=2.f*M_PI*rng.randFloat();
        float r=std::sqrt(std::max(0.f,1.f-z*z));
        integral+=scene.specularDirectionPdf(origin,Vector3f(r*std::cos(phi),r*std::sin(phi),z))*4.f*M_PI;
        direction=rng.randFloat()<0.5f?scene.sampleSpecularDirection(origin,rng):diffuse.sample(north,north,rng).direction;
        float p=0.5f*diffuse.pdf(north,direction,north)+0.5f*scene.specularDirectionPdf(origin,direction);
        furnace+=0.7f*std::max(0.f,direction.z)/(M_PI*p);
    }
    require(std::abs(integral/samples-1.)<0.025,"PDF normalization");
    require(std::abs(furnace/samples-0.7)<0.005,"constant radiance preserved");
    Scene inside;auto& mirror=inside.createMaterial(MIRROR);
    inside.addObj<Sphere>(origin,2.f,&mirror);inside.buildBVH();
    require(std::abs(inside.specularDirectionPdf(origin,north)*4.f*M_PI-1.f)<1e-6f,"inside sphere");
    Scene scaled;auto& m=scaled.createMaterial(DIELECTRIC);
    scaled.addObj<Sphere>(Vector3f(0,0,4000),1000.f,&m);scaled.buildBVH();
    require(std::abs(scaled.specularDirectionPdf(origin,north)/expected-1.f)<1e-5f,"scene scale invariance");
    std::cout<<"Camera proposal tests passed; integral="<<integral/samples<<", furnace="<<furnace/samples<<"\n";
}
