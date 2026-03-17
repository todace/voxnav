#pragma once
#include "voxnav/math_types.h"
#include "voxnav/voxelizer.h"

namespace voxnav {

// Extract an isosurface from a binary voxel grid using Marching Cubes.
SimplifiedMesh extractSurface(const VoxelGrid& grid);

} // namespace voxnav
