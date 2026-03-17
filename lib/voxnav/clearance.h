#pragma once
#include "voxnav/voxelizer.h"

namespace voxnav {

// Dilate the solid region of a voxel grid by the given agent radius.
// Returns a new grid where solid voxels have been expanded outward.
VoxelGrid dilate(const VoxelGrid& grid, float agentRadius);

} // namespace voxnav
