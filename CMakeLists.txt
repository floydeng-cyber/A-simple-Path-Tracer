cmake_minimum_required(VERSION 3.10)
project(Path_Tracing_rev)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Path_Tracing_rev
    src/main.cpp
    src/scene_ray/Camera.cpp
    src/scene_ray/Scene.cpp
    src/acceleration/BVH.cpp
    src/rendering/Renderer.cpp
    src/rendering/integrators/PathTracingIntegrator.cpp
    src/rendering/integrators/PhotonMappingIntegrator.cpp
    src/rendering/integrators/HybridIntegrator.cpp
    src/rendering/integrators/CelluloidIntegrator.cpp
)

target_include_directories(Path_Tracing_rev PRIVATE
    src
    src/setting_math
    src/geometry_material
    src/scene_ray
    src/acceleration
    src/rendering
    src/rendering/integrators
)

if(MSVC)
    target_compile_options(Path_Tracing_rev PRIVATE /W4)
else()
    target_compile_options(Path_Tracing_rev PRIVATE -Wall -Wextra -Wpedantic)
endif()
