# VoxNav

Voxel-based 3D navigation library with multi-gravity support. VoxNav converts arbitrary triangle meshes into navigable surfaces through voxelization, agent clearance dilation, and surface extraction, then builds a navigation mesh with A* pathfinding.

## Architecture

The library is organized into these core components:

- **Voxelizer** (`voxelizer.h`) - Converts triangle meshes into binary voxel grids using triangle-AABB intersection tests
- **Clearance** (`clearance.h`) - Morphological dilation of voxel grids to account for agent radius
- **Mesh Builder** (`mesh_builder.h`) - Marching Cubes surface extraction from voxel grids to produce simplified triangle meshes
- **Nav Mesh** (`nav_mesh.h`) - Navigation mesh construction with slope-based face classification, multi-gravity zone support, and A* pathfinding over walkable face graphs
- **OBJ Loader** (`obj_loader.h`) - Wavefront OBJ file import/export
- **Build Request** (`build_request.h`) - Async build system with thread pool for background voxelization and nav mesh construction
- **Math Types** (`math_types.h`) - Core types: `Vec3`, `AABB`, `Triangle`, `Mesh`, `SimplifiedMesh`

### Multi-Gravity Support

VoxNav supports gravity boxes -- axis-aligned regions where gravity direction differs from the default. Faces inside gravity boxes are classified as walkable based on their local gravity direction, enabling navigation on ceilings, walls, or arbitrary surfaces.

## Build Instructions

### Requirements

- CMake 3.14+
- C++17 compiler
- pthreads
- SDL2 + OpenGL 3.3 (optional, for sample app)

### Building

```bash
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)
```

If SDL2 is not found, the sample app is skipped and only the library + tests are built.

## Usage

```cpp
#include "voxnav/math_types.h"
#include "voxnav/voxelizer.h"
#include "voxnav/clearance.h"
#include "voxnav/mesh_builder.h"
#include "voxnav/nav_mesh.h"

using namespace voxnav;

// 1. Load or create a mesh
Mesh mesh = /* your mesh */;
AABB region = mesh.computeBounds();

// 2. Voxelize
VoxelGrid grid = voxelize(mesh, region, 0.25f);

// 3. Dilate for agent clearance
VoxelGrid dilated = dilate(grid, 0.5f);

// 4. Extract simplified surface
SimplifiedMesh surface = extractSurface(dilated);

// 5. Build navigation mesh
NavConfig config;
config.maxSlopeAngle = 45.0f;
config.defaultGravity = {0, -1, 0};

// Optional: add gravity boxes for multi-gravity zones
GravityBox gb;
gb.box = {{-5, 5, -5}, {5, 10, 5}};
gb.gravityDir = {0, 1, 0};  // inverted gravity
config.gravityBoxes.push_back(gb);

NavMesh nav;
nav.build(surface, config);

// 6. Find paths
auto path = nav.findPath({0, 0, 0}, {10, 0, 10});
```

### Async Builds

```cpp
BuildSystem sys(4);  // 4-thread pool
BuildParams params;
params.region = region;
params.sourceMesh = &mesh;
params.voxelSize = 0.25f;
params.agentRadius = 0.5f;
params.navConfig = config;

BuildHandle handle = sys.requestBuild(params);

// Poll for completion
while (!sys.isComplete(handle)) { /* wait */ }

BuildResult result = sys.getResult(handle);
```

## Tests

```bash
cd build
./tests/voxnav_tests
```

Runs 17 tests covering voxelization, clearance dilation, Marching Cubes surface extraction, navigation/pathfinding, raycast accuracy, and thread safety.

## Sample App

The sample app provides an interactive 3D visualization of the full VoxNav pipeline.

### Controls

| Input | Action |
|-------|--------|
| Right-click drag | Orbit camera |
| Middle-click drag | Pan camera |
| Scroll wheel | Zoom |
| Tab | Toggle FPS camera mode |
| W/A/S/D | FPS movement (when in FPS mode) |
| Space / Ctrl | FPS up / down |
| Shift | Sprint (FPS mode) |

### UI Panels

- **Build Parameters** - Adjust voxel size, agent radius, max slope, select mesh, rebuild
- **Visualization** - Toggle original mesh, simplified mesh, walkable faces, path, voxel debug view
- **Gravity Boxes** - Add/remove gravity zones with configurable bounds and gravity direction
- **Navigation Test** - Set start/end points and find paths

<!-- Screenshot placeholder: ![VoxNav Sample](screenshot.png) -->
