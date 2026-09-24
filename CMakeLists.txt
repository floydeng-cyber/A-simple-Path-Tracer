cmake_minimum_required(VERSION 3.10)
project(Path_Tracing_rev)

set(CMAKE_CXX_STANDARD 20)

add_executable(Path_Tracing_rev 
	main.cpp	Object.hpp	Vector.cpp	Vector.hpp	Sphere.hpp
	global.hpp	Triangle.hpp	Scene.cpp	Scene.hpp	Light.hpp
	AreaLight.hpp	BVH.cpp		BVH.hpp		Bounds3.hpp	Ray.hpp
	Material.hpp	Intersection.hpp
        Renderer.cpp	Renderer.hpp	Camera.cpp	Camera.hpp	Integrator.hpp
	PathTracingIntegrator.hpp	PathTracingIntegrator.cpp)

if(MSVC)
    target_compile_options(Path_Tracing_rev PRIVATE /W4)
else()
    target_compile_options(Path_Tracing_rev PRIVATE -Wall -Wextra -Wpedantic)
endif()