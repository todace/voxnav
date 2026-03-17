#include "voxnav/voxelizer.h"
#include <cmath>
#include <algorithm>

namespace voxnav {

void VoxelGrid::init(const AABB& b, float vs) {
    bounds = b;
    voxelSize = vs;
    Vec3 ext = b.extent();
    nx = std::max(1, static_cast<int>(std::ceil(ext.x / vs)));
    ny = std::max(1, static_cast<int>(std::ceil(ext.y / vs)));
    nz = std::max(1, static_cast<int>(std::ceil(ext.z / vs)));
    data.assign(static_cast<size_t>(nx) * ny * nz, 0);
}

bool VoxelGrid::get(int x, int y, int z) const {
    if (x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz) return false;
    return data[static_cast<size_t>(z) * nx * ny + static_cast<size_t>(y) * nx + x] != 0;
}

void VoxelGrid::set(int x, int y, int z, bool val) {
    if (x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz) return;
    data[static_cast<size_t>(z) * nx * ny + static_cast<size_t>(y) * nx + x] = val ? 1 : 0;
}

int VoxelGrid::solidCount() const {
    int count = 0;
    for (auto v : data) count += v;
    return count;
}

// Triangle-AABB intersection test (separating axis theorem)
namespace {

// Project triangle onto axis and return [min, max]
void projectTriangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& axis,
                     float& pmin, float& pmax) {
    float p0 = axis.dot(v0);
    float p1 = axis.dot(v1);
    float p2 = axis.dot(v2);
    pmin = std::min({p0, p1, p2});
    pmax = std::max({p0, p1, p2});
}

// Project AABB onto axis and return [min, max]
void projectAABB(const Vec3& center, const Vec3& halfExt, const Vec3& axis,
                 float& pmin, float& pmax) {
    float r = std::abs(axis.x) * halfExt.x + std::abs(axis.y) * halfExt.y + std::abs(axis.z) * halfExt.z;
    float c = axis.dot(center);
    pmin = c - r;
    pmax = c + r;
}

bool overlapsOnAxis(const Vec3& v0, const Vec3& v1, const Vec3& v2,
                    const Vec3& boxCenter, const Vec3& halfExt, const Vec3& axis) {
    float l = axis.lengthSq();
    if (l < 1e-12f) return true; // degenerate axis
    float tmin, tmax, bmin, bmax;
    projectTriangle(v0, v1, v2, axis, tmin, tmax);
    projectAABB(boxCenter, halfExt, axis, bmin, bmax);
    constexpr float eps = 1e-4f;
    return tmin <= bmax + eps && tmax >= bmin - eps;
}

bool triangleAABBIntersect(const Triangle& tri, const Vec3& boxCenter, const Vec3& halfExt) {
    // Translate triangle so box center is at origin
    Vec3 v0 = tri.v0 - boxCenter;
    Vec3 v1 = tri.v1 - boxCenter;
    Vec3 v2 = tri.v2 - boxCenter;

    Vec3 e0 = v1 - v0, e1 = v2 - v1, e2 = v0 - v2;

    // 3 face normals of AABB
    Vec3 axes[3] = {{1,0,0},{0,1,0},{0,0,1}};
    for (auto& a : axes) {
        if (!overlapsOnAxis(v0, v1, v2, {0,0,0}, halfExt, a)) return false;
    }

    // Triangle normal
    Vec3 tn = e0.cross(e1);
    if (!overlapsOnAxis(v0, v1, v2, {0,0,0}, halfExt, tn)) return false;

    // 9 cross product axes
    Vec3 edges[3] = {e0, e1, e2};
    for (auto& e : edges) {
        for (auto& a : axes) {
            Vec3 cross = e.cross(a);
            if (!overlapsOnAxis(v0, v1, v2, {0,0,0}, halfExt, cross)) return false;
        }
    }

    return true;
}

} // anonymous namespace

VoxelGrid voxelize(const Mesh& mesh, const AABB& region, float voxelSize) {
    VoxelGrid grid;
    grid.init(region, voxelSize);

    Vec3 halfVoxel = {voxelSize * 0.5f, voxelSize * 0.5f, voxelSize * 0.5f};

    for (size_t t = 0; t < mesh.triangleCount(); ++t) {
        Triangle tri = mesh.getTriangle(t);
        AABB tb = tri.bounds();

        // Map triangle bounds to voxel range
        int x0 = std::max(0, static_cast<int>((tb.min.x - region.min.x) / voxelSize));
        int y0 = std::max(0, static_cast<int>((tb.min.y - region.min.y) / voxelSize));
        int z0 = std::max(0, static_cast<int>((tb.min.z - region.min.z) / voxelSize));
        int x1 = std::min(grid.nx - 1, static_cast<int>((tb.max.x - region.min.x) / voxelSize));
        int y1 = std::min(grid.ny - 1, static_cast<int>((tb.max.y - region.min.y) / voxelSize));
        int z1 = std::min(grid.nz - 1, static_cast<int>((tb.max.z - region.min.z) / voxelSize));

        for (int z = z0; z <= z1; ++z) {
            for (int y = y0; y <= y1; ++y) {
                for (int x = x0; x <= x1; ++x) {
                    if (grid.get(x, y, z)) continue;
                    Vec3 center = {
                        region.min.x + (x + 0.5f) * voxelSize,
                        region.min.y + (y + 0.5f) * voxelSize,
                        region.min.z + (z + 0.5f) * voxelSize
                    };
                    if (triangleAABBIntersect(tri, center, halfVoxel)) {
                        grid.set(x, y, z, true);
                    }
                }
            }
        }
    }
    return grid;
}

} // namespace voxnav
