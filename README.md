# Path Tracing and Photon Mapping

This project renders the Cornell-box scene with path tracing, photon mapping,
or a configurable hybrid of both methods.

## Build

```powershell
cmake -S . -B build -G Ninja
cmake --build build --config Release
```

Run commands from the project root so the renderer can locate `assets/models`.

## Render Modes

Select the renderer in `src/setting_math/RenderSettings.hpp`:

```cpp
RenderMode renderMode = RenderMode::PathTracing;
// RenderMode renderMode = RenderMode::PhotonMapping;
// RenderMode renderMode = RenderMode::Hybrid;
```

Set `outputFile` in the same file, then run the executable without arguments:

```powershell
.\build\Path_Tracing_rev.exe
```

Image quality and photon-map parameters are also configured in
`src/setting_math/RenderSettings.hpp`. Hybrid mode uses path tracing for general
illumination and photon mapping for caustics.

Each run reports scene/BVH setup, integrator preprocessing, camera integration,
image writing, render-pipeline total, and end-to-end wall-clock time. Photon-map
construction is included in the render-pipeline total for fair comparisons.


## Layout

```text
assets/models/              Scene geometry
src/main.cpp                Program entry point
src/setting_math/           Shared math, sampling, and settings
src/geometry_material/     Shapes and materials
src/acceleration/           BVH construction and traversal
src/scene_ray/              Scene, camera, bounds, rays, and intersections
src/ObjLoader.hpp           OBJ loader
src/rendering/              Renderer and base integrator
src/rendering/integrators/  Path, photon, and hybrid integrators
render_img/                 Generated images (ignored by Git)
```
