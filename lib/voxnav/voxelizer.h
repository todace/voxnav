#pragma once
#include "voxnav/math_types.h"
#include <vector>
#include <cstdint>

namespace voxnav {

struct VoxelGrid {
    AABB bounds;
    int nx = 0, ny = 0, nz = 0;
    float voxelSize = 0;
    std::vector<uint8_t> data; // byte per voxel: 1=solid, 0=empty

    bool get(int x, int y, int z) const;
    void set(int x, int y, int z, bool val);
    int solidCount() const;

    // Initialize grid from bounds and voxel size
    void init(const AABB& bounds, float voxelSize);
};

// Voxelize a mesh into a binary voxel grid
VoxelGrid voxelize(const Mesh& mesh, const AABB& region, float voxelSize);

} // namespace voxnav
