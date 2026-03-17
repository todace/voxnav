#include "voxnav/clearance.h"
#include <cmath>
#include <algorithm>

namespace voxnav {

VoxelGrid dilate(const VoxelGrid& grid, float agentRadius) {
    int erodeVoxels = static_cast<int>(std::ceil(agentRadius / grid.voxelSize));
    if (erodeVoxels <= 1) {
        return grid; // voxels already agent-sized
    }

    VoxelGrid result;
    result.bounds = grid.bounds;
    result.nx = grid.nx;
    result.ny = grid.ny;
    result.nz = grid.nz;
    result.voxelSize = grid.voxelSize;
    result.data.assign(grid.data.size(), 0);

    float r2 = static_cast<float>(erodeVoxels * erodeVoxels);

    for (int z = 0; z < grid.nz; ++z) {
        for (int y = 0; y < grid.ny; ++y) {
            for (int x = 0; x < grid.nx; ++x) {
                if (!grid.get(x, y, z)) continue;

                // This voxel is solid — mark all voxels within erodeVoxels radius
                int xmin = std::max(0, x - erodeVoxels);
                int xmax = std::min(grid.nx - 1, x + erodeVoxels);
                int ymin = std::max(0, y - erodeVoxels);
                int ymax = std::min(grid.ny - 1, y + erodeVoxels);
                int zmin = std::max(0, z - erodeVoxels);
                int zmax = std::min(grid.nz - 1, z + erodeVoxels);

                for (int dz = zmin; dz <= zmax; ++dz) {
                    for (int dy = ymin; dy <= ymax; ++dy) {
                        for (int dx = xmin; dx <= xmax; ++dx) {
                            float dist2 = static_cast<float>((dx-x)*(dx-x) + (dy-y)*(dy-y) + (dz-z)*(dz-z));
                            if (dist2 <= r2) {
                                result.set(dx, dy, dz, true);
                            }
                        }
                    }
                }
            }
        }
    }
    return result;
}

} // namespace voxnav
