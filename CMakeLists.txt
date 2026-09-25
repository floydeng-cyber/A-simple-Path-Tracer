cmake_minimum_required(VERSION 3.10)
project(Path_Tracing_rev)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Path_Tracing_rev
    src/main.cpp

    src/core/Bounds3.hpp
    src/core/global.hpp
    src/core/Ray.hpp
    src/core/RenderSettings.hpp
    src/core/Sampler.hpp
    src/core/Vector.hpp

    src/scene/AreaLight.hpp
    src/scene/Intersection.hpp
    src/scene/Light.hpp
    src/scene/Material.hpp
    src/scene/Scene.cpp
    src/scene/Scene.hpp

    src/geometry/Object.hpp
    src/geometry/Sphere.hpp
    src/geometry/Triangle.hpp
    src/OBJ_Loader.hpp

    src/acceleration/BVH.cpp
    src/acceleration/BVH.hpp

    src/rendering/Camera.cpp
    src/rendering/Camera.hpp
    src/rendering/Renderer.cpp
    src/rendering/Renderer.hpp

    src/rendering/integrators/Integrator.hpp
    src/rendering/integrators/PathTracingIntegrator.cpp
    src/rendering/integrators/PathTracingIntegrator.hpp
    src/rendering/integrators/PhotonMappingIntegrator.cpp
    src/rendering/integrators/PhotonMappingIntegrator.hpp
    src/rendering/integrators/HybridIntegrator.cpp
    src/rendering/integrators/HybridIntegrator.hpp
)

target_include_directories(Path_Tracing_rev PRIVATE
    src
    src/core
    src/scene
    src/geometry
    src/acceleration
    src/rendering
    src/rendering/integrators
)

if(MSVC)
    target_compile_options(Path_Tracing_rev PRIVATE /W4)
else()
    target_compile_options(Path_Tracing_rev PRIVATE -Wall -Wextra -Wpedantic)
endif()
