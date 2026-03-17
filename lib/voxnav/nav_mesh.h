#pragma once
#include "voxnav/math_types.h"
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace voxnav {

struct GravityBox {
    AABB box;
    Vec3 gravityDir; // Normalized gravity direction
};

struct NavConfig {
    float maxSlopeAngle = 45.0f;     // Degrees
    Vec3 defaultGravity = {0, -1, 0};
    std::vector<GravityBox> gravityBoxes;
};

class NavMesh {
public:
    void build(const SimplifiedMesh& mesh, const NavConfig& config);

    // Find path between two world-space points
    std::vector<Vec3> findPath(const Vec3& start, const Vec3& end) const;

    // Find nearest walkable face to a point
    int findNearestFace(const Vec3& point) const;

    // Get walkable face indices
    const std::vector<int>& getWalkableFaces() const { return walkableFaces_; }

    // Get the simplified mesh used
    const SimplifiedMesh& getMesh() const { return mesh_; }

    // Get gravity direction for a face
    Vec3 getGravityForFace(int faceIdx) const;

private:
    SimplifiedMesh mesh_;
    NavConfig config_;
    std::vector<bool> faceWalkable_;
    std::vector<int> walkableFaces_;
    // Adjacency: for each face, list of adjacent face indices
    std::vector<std::vector<int>> adjacency_;

    void buildAdjacency();
    void classifyFaces();
};

} // namespace voxnav
