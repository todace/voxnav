#include "voxnav/math_types.h"
#include "voxnav/voxelizer.h"
#include "voxnav/clearance.h"
#include "voxnav/mesh_builder.h"
#include "voxnav/nav_mesh.h"
#include "voxnav/obj_loader.h"
#include "voxnav/build_request.h"

#include <iostream>
#include <cassert>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

using namespace voxnav;

static int testsPassed = 0;
static int testsFailed = 0;

#define TEST(name) \
    static void test_##name(); \
    struct Register_##name { \
        Register_##name() { tests().push_back({#name, test_##name}); } \
    } reg_##name; \
    static void test_##name()

#define ASSERT_TRUE(expr) \
    do { if (!(expr)) { \
        std::cerr << "  FAIL: " << #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
        throw std::runtime_error("assertion failed"); \
    }} while(0)

#define ASSERT_EQ(a, b) ASSERT_TRUE((a) == (b))
#define ASSERT_NEAR(a, b, tol) ASSERT_TRUE(std::abs((a) - (b)) <= (tol))
#define ASSERT_GT(a, b) ASSERT_TRUE((a) > (b))
#define ASSERT_GE(a, b) ASSERT_TRUE((a) >= (b))
#define ASSERT_LT(a, b) ASSERT_TRUE((a) < (b))
#define ASSERT_LE(a, b) ASSERT_TRUE((a) <= (b))

struct TestEntry {
    std::string name;
    void (*func)();
};

static std::vector<TestEntry>& tests() {
    static std::vector<TestEntry> t;
    return t;
}

// ============================================================================
// Helper: create a unit cube mesh (axis-aligned, from -0.5 to 0.5)
// ============================================================================
static Mesh makeUnitCube() {
    Mesh m;
    m.vertices = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f},
        {-0.5f, -0.5f,  0.5f}, {0.5f, -0.5f,  0.5f}, {0.5f, 0.5f,  0.5f}, {-0.5f, 0.5f,  0.5f}
    };
    // 6 faces, 2 triangles each = 12 triangles
    m.indices = {
        // -Z face
        0,1,2, 0,2,3,
        // +Z face
        4,6,5, 4,7,6,
        // -Y face
        0,5,1, 0,4,5,
        // +Y face
        2,7,3, 2,6,7,
        // -X face
        0,3,7, 0,7,4,
        // +X face
        1,5,6, 1,6,2
    };
    return m;
}

// Create a flat floor quad (Y=0, from -5 to 5 in XZ)
static Mesh makeFloorQuad() {
    Mesh m;
    m.vertices = {
        {-5, 0, -5}, {5, 0, -5}, {5, 0, 5}, {-5, 0, 5}
    };
    m.indices = {0, 1, 2, 0, 2, 3};
    return m;
}

// Create two parallel planes along X-axis, separated by a gap
static Mesh makeTwoPlanes(float gap) {
    Mesh m;
    // Plane 1 at Y=0
    m.vertices = {
        {-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}
    };
    // Plane 2 at Y=gap
    m.vertices.push_back({-1, gap, -1});
    m.vertices.push_back({ 1, gap, -1});
    m.vertices.push_back({ 1, gap,  1});
    m.vertices.push_back({-1, gap,  1});

    m.indices = {
        0,1,2, 0,2,3,   // bottom
        4,5,6, 4,6,7    // top
    };
    return m;
}

// Create a simple corridor mesh (floor + walls)
static Mesh makeCorridor(float length, float width, float wallHeight) {
    Mesh m;
    float hw = width / 2.0f;

    // Floor: Y=0
    m.vertices.push_back({0, 0, -hw});
    m.vertices.push_back({length, 0, -hw});
    m.vertices.push_back({length, 0, hw});
    m.vertices.push_back({0, 0, hw});

    // Left wall: Z=-hw, from Y=0 to wallHeight
    m.vertices.push_back({0, 0, -hw});
    m.vertices.push_back({length, 0, -hw});
    m.vertices.push_back({length, wallHeight, -hw});
    m.vertices.push_back({0, wallHeight, -hw});

    // Right wall: Z=hw
    m.vertices.push_back({0, 0, hw});
    m.vertices.push_back({length, 0, hw});
    m.vertices.push_back({length, wallHeight, hw});
    m.vertices.push_back({0, wallHeight, hw});

    m.indices = {
        0,1,2, 0,2,3,     // floor
        4,6,5, 4,7,6,     // left wall
        8,9,10, 8,10,11   // right wall
    };
    return m;
}

// Create two disconnected platforms
static Mesh makeTwoPlatforms() {
    Mesh m;
    // Platform 1 at Y=0, X=[-5,-1]
    m.vertices = {
        {-5, 0, -2}, {-1, 0, -2}, {-1, 0, 2}, {-5, 0, 2}
    };
    // Platform 2 at Y=0, X=[1, 5]
    m.vertices.push_back({1, 0, -2});
    m.vertices.push_back({5, 0, -2});
    m.vertices.push_back({5, 0,  2});
    m.vertices.push_back({1, 0,  2});

    m.indices = {
        0,1,2, 0,2,3,
        4,5,6, 4,6,7
    };
    return m;
}

// ============================================================================
// 1. Voxelization Tests
// ============================================================================

TEST(voxelize_axis_aligned_box) {
    Mesh cube = makeUnitCube();
    AABB region = {{-1, -1, -1}, {1, 1, 1}};
    VoxelGrid grid = voxelize(cube, region, 0.25f);

    int count = grid.solidCount();
    // A unit cube (edge length 1) voxelized at 0.25 resolution
    // Surface voxels should be in the dozens to low hundreds range
    ASSERT_GT(count, 20);
    ASSERT_LT(count, 500);
}

TEST(voxelize_empty_region) {
    Mesh cube = makeUnitCube();
    // Region that doesn't contain the cube
    AABB region = {{10, 10, 10}, {12, 12, 12}};
    VoxelGrid grid = voxelize(cube, region, 0.25f);
    ASSERT_EQ(grid.solidCount(), 0);
}

TEST(voxelize_resolution_scaling) {
    Mesh cube = makeUnitCube();
    AABB region = {{-1, -1, -1}, {1, 1, 1}};

    VoxelGrid coarse = voxelize(cube, region, 0.5f);
    VoxelGrid fine = voxelize(cube, region, 0.25f);

    // Finer grid should have more voxels (surface ~ resolution^2 scaling)
    ASSERT_GT(fine.solidCount(), coarse.solidCount());
    // Expect roughly 4x more (surface area scales with res^2)
    float ratio = static_cast<float>(fine.solidCount()) / coarse.solidCount();
    ASSERT_GT(ratio, 2.0f);
    ASSERT_LT(ratio, 8.0f);
}

TEST(voxelize_rotated_box) {
    // Rotate the cube 45 degrees around Y — create rotated vertices manually
    Mesh m;
    float c = std::cos(3.14159f / 4.0f);
    float s = std::sin(3.14159f / 4.0f);

    auto rot = [c, s](Vec3 v) -> Vec3 {
        return {c * v.x + s * v.z, v.y, -s * v.x + c * v.z};
    };

    Mesh cube = makeUnitCube();
    m.vertices.resize(cube.vertices.size());
    for (size_t i = 0; i < cube.vertices.size(); ++i) {
        m.vertices[i] = rot(cube.vertices[i]);
    }
    m.indices = cube.indices;

    AABB region = {{-1.5f, -1.5f, -1.5f}, {1.5f, 1.5f, 1.5f}};
    VoxelGrid gridRot = voxelize(m, region, 0.25f);

    // Should still produce solid voxels
    ASSERT_GT(gridRot.solidCount(), 20);
}

// ============================================================================
// 2. Clearance / Dilation Tests
// ============================================================================

TEST(dilation_closes_narrow_gap) {
    Mesh planes = makeTwoPlanes(0.3f);
    AABB region = {{-2, -1, -2}, {2, 2, 2}};
    VoxelGrid grid = voxelize(planes, region, 0.1f);

    VoxelGrid dilated = dilate(grid, 0.2f);

    // After dilation, the gap (0.3 apart) with agent radius 0.2 should be closed.
    // Check the midpoint between the two planes.
    float midY = 0.15f;
    int vy = static_cast<int>((midY - region.min.y) / 0.1f);
    int vx = static_cast<int>((0.0f - region.min.x) / 0.1f);
    int vz = static_cast<int>((0.0f - region.min.z) / 0.1f);

    ASSERT_TRUE(dilated.get(vx, vy, vz));
}

TEST(dilation_preserves_wide_gap) {
    Mesh planes = makeTwoPlanes(2.0f);
    AABB region = {{-2, -1, -2}, {2, 4, 2}};
    VoxelGrid grid = voxelize(planes, region, 0.1f);
    VoxelGrid dilated = dilate(grid, 0.2f);

    // Midpoint at Y=1.0 should still be empty
    float midY = 1.0f;
    int vy = static_cast<int>((midY - region.min.y) / 0.1f);
    int vx = static_cast<int>((0.0f - region.min.x) / 0.1f);
    int vz = static_cast<int>((0.0f - region.min.z) / 0.1f);

    ASSERT_TRUE(!dilated.get(vx, vy, vz));
}

TEST(no_dilation_when_voxel_large) {
    Mesh cube = makeUnitCube();
    AABB region = {{-2, -2, -2}, {2, 2, 2}};
    VoxelGrid grid = voxelize(cube, region, 1.0f);
    VoxelGrid dilated = dilate(grid, 0.5f);

    // With voxelSize=1.0 and agentRadius=0.5, erodeVoxels=1, so no dilation
    ASSERT_EQ(grid.solidCount(), dilated.solidCount());
}

// ============================================================================
// 3. Mesh Builder (Marching Cubes) Tests
// ============================================================================

TEST(mc_sphere_topology) {
    // Create a spherical region of solid voxels
    VoxelGrid grid;
    AABB bounds = {{-5, -5, -5}, {5, 5, 5}};
    grid.init(bounds, 0.5f);

    float r = 3.0f;
    for (int z = 0; z < grid.nz; ++z) {
        for (int y = 0; y < grid.ny; ++y) {
            for (int x = 0; x < grid.nx; ++x) {
                Vec3 pos = {
                    bounds.min.x + (x + 0.5f) * grid.voxelSize,
                    bounds.min.y + (y + 0.5f) * grid.voxelSize,
                    bounds.min.z + (z + 0.5f) * grid.voxelSize
                };
                if (pos.lengthSq() <= r * r) {
                    grid.set(x, y, z, true);
                }
            }
        }
    }

    SimplifiedMesh mesh = extractSurface(grid);

    // Should produce a closed mesh with many triangles
    ASSERT_GT(mesh.triangleCount(), 100u);
    ASSERT_GT(mesh.vertices.size(), 50u);

    // Verify watertight: each edge should be shared by exactly 2 faces
    std::unordered_map<uint64_t, int> edgeCounts;
    for (size_t i = 0; i < mesh.triangleCount(); ++i) {
        uint32_t i0 = mesh.indices[i*3], i1 = mesh.indices[i*3+1], i2 = mesh.indices[i*3+2];
        auto ek = [](uint32_t a, uint32_t b) -> uint64_t {
            if (a > b) std::swap(a, b);
            return (static_cast<uint64_t>(a) << 32) | b;
        };
        edgeCounts[ek(i0, i1)]++;
        edgeCounts[ek(i1, i2)]++;
        edgeCounts[ek(i2, i0)]++;
    }

    int boundaryEdges = 0;
    for (auto& [k, count] : edgeCounts) {
        if (count != 2) boundaryEdges++;
    }
    // A sphere extracted via MC on a binary grid should be mostly watertight
    // Allow small fraction of boundary edges due to grid boundaries
    float boundaryRatio = static_cast<float>(boundaryEdges) / edgeCounts.size();
    ASSERT_LT(boundaryRatio, 0.05f);
}

TEST(mc_cube_face_count) {
    VoxelGrid grid;
    AABB bounds = {{-2, -2, -2}, {2, 2, 2}};
    grid.init(bounds, 0.25f);

    // Fill a cube from (-1,-1,-1) to (1,1,1)
    for (int z = 0; z < grid.nz; ++z) {
        for (int y = 0; y < grid.ny; ++y) {
            for (int x = 0; x < grid.nx; ++x) {
                Vec3 pos = {
                    bounds.min.x + (x + 0.5f) * grid.voxelSize,
                    bounds.min.y + (y + 0.5f) * grid.voxelSize,
                    bounds.min.z + (z + 0.5f) * grid.voxelSize
                };
                if (std::abs(pos.x) <= 1.0f && std::abs(pos.y) <= 1.0f && std::abs(pos.z) <= 1.0f) {
                    grid.set(x, y, z, true);
                }
            }
        }
    }

    SimplifiedMesh mesh = extractSurface(grid);
    ASSERT_GT(mesh.triangleCount(), 10u);

    // Check that normals cluster around 6 directions (cube faces)
    int counts[6] = {}; // +x, -x, +y, -y, +z, -z
    for (size_t i = 0; i < mesh.triangleCount(); ++i) {
        Triangle tri = mesh.getTriangle(i);
        Vec3 n = tri.normal();
        float ax = std::abs(n.x), ay = std::abs(n.y), az = std::abs(n.z);
        if (ax > ay && ax > az) {
            counts[n.x > 0 ? 0 : 1]++;
        } else if (ay > az) {
            counts[n.y > 0 ? 2 : 3]++;
        } else {
            counts[n.z > 0 ? 4 : 5]++;
        }
    }
    // All 6 directions should have some faces
    for (int i = 0; i < 6; ++i) {
        ASSERT_GT(counts[i], 0);
    }
}

// ============================================================================
// 4. Navigation Tests
// ============================================================================

TEST(flat_floor_walkable) {
    // Voxelize a floor, extract surface, build navmesh
    Mesh floor = makeFloorQuad();
    AABB region = {{-6, -1, -6}, {6, 1, 6}};
    VoxelGrid grid = voxelize(floor, region, 0.5f);
    VoxelGrid dilated = dilate(grid, 0.1f);
    SimplifiedMesh smesh = extractSurface(dilated);

    NavConfig config;
    config.maxSlopeAngle = 45.0f;
    config.defaultGravity = {0, -1, 0};

    NavMesh nav;
    nav.build(smesh, config);

    // Should have some walkable faces
    ASSERT_GT(static_cast<int>(nav.getWalkableFaces().size()), 0);
}

TEST(gravity_box_ceiling_walk) {
    // Create a box shape, use reversed gravity to make ceiling walkable
    Mesh cube = makeUnitCube();
    AABB region = {{-2, -2, -2}, {2, 2, 2}};
    VoxelGrid grid = voxelize(cube, region, 0.2f);
    SimplifiedMesh smesh = extractSurface(grid);

    NavConfig config;
    config.maxSlopeAngle = 45.0f;
    config.defaultGravity = {0, -1, 0};

    // Add gravity box covering top half with Y-up gravity (inverted)
    GravityBox gb;
    gb.box = {{-2, 0, -2}, {2, 2, 2}};
    gb.gravityDir = {0, 1, 0}; // gravity pointing up
    config.gravityBoxes.push_back(gb);

    NavMesh nav;
    nav.build(smesh, config);

    // Should have walkable faces (both bottom faces from default gravity
    // and top faces from inverted gravity box)
    ASSERT_GT(static_cast<int>(nav.getWalkableFaces().size()), 0);
}

TEST(path_simple_corridor) {
    Mesh corridor = makeCorridor(10.0f, 2.0f, 2.0f);
    AABB region = {{-1, -1, -2}, {11, 4, 2}};
    VoxelGrid grid = voxelize(corridor, region, 0.5f);
    VoxelGrid dilated = dilate(grid, 0.1f);
    SimplifiedMesh smesh = extractSurface(dilated);

    NavConfig config;
    config.maxSlopeAngle = 45.0f;
    config.defaultGravity = {0, -1, 0};

    NavMesh nav;
    nav.build(smesh, config);

    Vec3 start = {1.0f, 0.5f, 0.0f};
    Vec3 end = {9.0f, 0.5f, 0.0f};
    auto path = nav.findPath(start, end);

    // Should find a path
    ASSERT_GT(static_cast<int>(path.size()), 1);

    // Path length should be reasonable
    float pathLen = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        pathLen += (path[i] - path[i-1]).length();
    }
    float directDist = (end - start).length();
    // Path should be within 3x direct distance (generous for voxelized path)
    ASSERT_LT(pathLen, directDist * 3.0f);
}

TEST(path_blocked) {
    Mesh platforms = makeTwoPlatforms();
    AABB region = {{-6, -1, -3}, {6, 1, 3}};
    VoxelGrid grid = voxelize(platforms, region, 0.5f);
    VoxelGrid dilated = dilate(grid, 0.1f);
    SimplifiedMesh smesh = extractSurface(dilated);

    NavConfig config;
    config.maxSlopeAngle = 45.0f;
    config.defaultGravity = {0, -1, 0};

    NavMesh nav;
    nav.build(smesh, config);

    Vec3 start = {-3.0f, 0.5f, 0.0f};
    Vec3 end = {3.0f, 0.5f, 0.0f};
    auto path = nav.findPath(start, end);

    // Platforms are disconnected, so no path
    ASSERT_TRUE(path.empty());
}

TEST(path_optimality) {
    // Open floor — path should be close to direct
    Mesh floor = makeFloorQuad();
    AABB region = {{-6, -1, -6}, {6, 1, 6}};
    VoxelGrid grid = voxelize(floor, region, 0.5f);
    VoxelGrid dilated = dilate(grid, 0.1f);
    SimplifiedMesh smesh = extractSurface(dilated);

    NavConfig config;
    config.maxSlopeAngle = 45.0f;
    config.defaultGravity = {0, -1, 0};

    NavMesh nav;
    nav.build(smesh, config);

    Vec3 start = {-3.0f, 0.5f, -3.0f};
    Vec3 end = {3.0f, 0.5f, 3.0f};
    auto path = nav.findPath(start, end);

    if (!path.empty()) {
        float pathLen = 0;
        for (size_t i = 1; i < path.size(); ++i) {
            pathLen += (path[i] - path[i-1]).length();
        }
        float directDist = (end - start).length();
        ASSERT_LE(pathLen, directDist * 1.5f);
    }
}

// ============================================================================
// 5. Raycast Error Test
// ============================================================================

// Moller-Trumbore ray-triangle intersection
static bool rayTriIntersect(const Vec3& orig, const Vec3& dir,
                            const Vec3& v0, const Vec3& v1, const Vec3& v2,
                            float& t) {
    const float EPSILON = 1e-7f;
    Vec3 e1 = v1 - v0;
    Vec3 e2 = v2 - v0;
    Vec3 h = dir.cross(e2);
    float a = e1.dot(h);
    if (a > -EPSILON && a < EPSILON) return false;
    float f = 1.0f / a;
    Vec3 s = orig - v0;
    float u = f * s.dot(h);
    if (u < 0.0f || u > 1.0f) return false;
    Vec3 q = s.cross(e1);
    float v = f * dir.dot(q);
    if (v < 0.0f || u + v > 1.0f) return false;
    t = f * e2.dot(q);
    return t > EPSILON;
}

// Raycast against a mesh, return closest hit distance (negative if no hit)
static float raycastMesh(const Vec3& orig, const Vec3& dir,
                         const std::vector<Vec3>& verts,
                         const std::vector<uint32_t>& indices) {
    float closest = -1.0f;
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        float t;
        if (rayTriIntersect(orig, dir, verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]], t)) {
            if (closest < 0 || t < closest) closest = t;
        }
    }
    return closest;
}

// Create a UV sphere mesh procedurally
static Mesh makeUVSphere(float radius, int slices, int stacks) {
    Mesh m;
    // Generate vertices
    for (int j = 0; j <= stacks; ++j) {
        float phi = 3.14159265f * j / stacks;
        float sp = std::sin(phi), cp = std::cos(phi);
        for (int i = 0; i <= slices; ++i) {
            float theta = 2.0f * 3.14159265f * i / slices;
            float st = std::sin(theta), ct = std::cos(theta);
            m.vertices.push_back({radius * sp * ct, radius * cp, radius * sp * st});
        }
    }
    // Generate indices
    for (int j = 0; j < stacks; ++j) {
        for (int i = 0; i < slices; ++i) {
            uint32_t a = j * (slices + 1) + i;
            uint32_t b = a + slices + 1;
            m.indices.push_back(a);
            m.indices.push_back(b);
            m.indices.push_back(a + 1);
            m.indices.push_back(a + 1);
            m.indices.push_back(b);
            m.indices.push_back(b + 1);
        }
    }
    return m;
}

TEST(simplified_vs_original_raycast) {
    // Create a unit sphere, voxelize, extract simplified surface
    Mesh sphere = makeUVSphere(2.0f, 24, 16);
    AABB region = {{-3, -3, -3}, {3, 3, 3}};
    float vs = 0.2f;
    VoxelGrid grid = voxelize(sphere, region, vs);
    SimplifiedMesh simplified = extractSurface(grid);

    ASSERT_GT(simplified.triangleCount(), 10u);

    // Cast random rays from outside the sphere toward the origin
    // Use a simple pseudo-random generator for reproducibility
    int numRays = 200;
    int hitsOrig = 0, hitsSimp = 0, hitsBoth = 0;
    float sumSqError = 0;

    auto pseudoRand = [](int seed) -> float {
        // Simple hash-based pseudo-random [0,1]
        unsigned int h = static_cast<unsigned int>(seed);
        h = ((h >> 16) ^ h) * 0x45d9f3b;
        h = ((h >> 16) ^ h) * 0x45d9f3b;
        h = (h >> 16) ^ h;
        return static_cast<float>(h & 0xFFFF) / 65535.0f;
    };

    for (int i = 0; i < numRays; ++i) {
        // Random direction on sphere surface
        float u = pseudoRand(i * 3 + 0) * 2.0f - 1.0f;
        float v = pseudoRand(i * 3 + 1) * 2.0f * 3.14159265f;
        float s = std::sqrt(1.0f - u * u);
        Vec3 dir = {s * std::cos(v), u, s * std::sin(v)};
        dir = dir.normalized();

        // Origin far away from sphere, pointing toward center
        Vec3 orig = dir * (-6.0f);  // 6 units away from center
        Vec3 rayDir = dir;          // pointing inward

        float tOrig = raycastMesh(orig, rayDir, sphere.vertices, sphere.indices);
        float tSimp = raycastMesh(orig, rayDir, simplified.vertices, simplified.indices);

        if (tOrig > 0) hitsOrig++;
        if (tSimp > 0) hitsSimp++;
        if (tOrig > 0 && tSimp > 0) {
            hitsBoth++;
            float err = tOrig - tSimp;
            sumSqError += err * err;
        }
    }

    // Both meshes should be hit by most rays
    ASSERT_GT(hitsOrig, numRays / 2);
    ASSERT_GT(hitsSimp, numRays / 2);
    ASSERT_GT(hitsBoth, numRays / 4);

    // RMS error should be below threshold (related to voxel size)
    float rmsError = std::sqrt(sumSqError / std::max(hitsBoth, 1));
    // Error should be within a few voxel sizes
    float errorThreshold = vs * 4.0f;
    ASSERT_LT(rmsError, errorThreshold);
}

// ============================================================================
// 6. Thread Safety / Async Tests
// ============================================================================

TEST(async_build_completes) {
    BuildSystem sys(2);
    Mesh cube = makeUnitCube();
    BuildParams params;
    params.region = {{-2, -2, -2}, {2, 2, 2}};
    params.sourceMesh = &cube;
    params.voxelSize = 0.5f;
    params.agentRadius = 0.1f;
    params.navConfig.maxSlopeAngle = 45.0f;
    params.navConfig.defaultGravity = {0, -1, 0};

    BuildHandle h = sys.requestBuild(params);

    // Wait for completion (with timeout)
    auto start = std::chrono::steady_clock::now();
    while (!sys.isComplete(h)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto elapsed = std::chrono::steady_clock::now() - start;
        ASSERT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(), 30);
    }

    BuildResult result = sys.getResult(h);
    ASSERT_GT(result.voxelGrid.solidCount(), 0);
    ASSERT_GT(result.simplifiedMesh.triangleCount(), 0u);
}

TEST(multiple_concurrent_builds) {
    BuildSystem sys(4);
    Mesh cube = makeUnitCube();

    std::vector<BuildHandle> handles;
    for (int i = 0; i < 4; ++i) {
        BuildParams params;
        params.region = {{-2, -2, -2}, {2, 2, 2}};
        params.sourceMesh = &cube;
        params.voxelSize = 0.5f + i * 0.1f;
        params.agentRadius = 0.1f;
        params.navConfig.maxSlopeAngle = 45.0f;
        params.navConfig.defaultGravity = {0, -1, 0};

        handles.push_back(sys.requestBuild(params));
    }

    // Wait for all
    auto start = std::chrono::steady_clock::now();
    for (auto h : handles) {
        while (!sys.isComplete(h)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            auto elapsed = std::chrono::steady_clock::now() - start;
            ASSERT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(), 60);
        }
        BuildResult result = sys.getResult(h);
        ASSERT_GT(result.voxelGrid.solidCount(), 0);
    }
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "Running VoxNav tests...\n\n";

    for (auto& t : tests()) {
        std::cout << "  " << t.name << "... ";
        try {
            t.func();
            std::cout << "OK\n";
            testsPassed++;
        } catch (const std::exception& e) {
            std::cout << "FAILED: " << e.what() << "\n";
            testsFailed++;
        }
    }

    std::cout << "\n" << testsPassed << " passed, " << testsFailed << " failed\n";
    return testsFailed > 0 ? 1 : 0;
}
