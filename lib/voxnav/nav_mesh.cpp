#include "voxnav/nav_mesh.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <limits>

namespace voxnav {

namespace {

constexpr float kPi = 3.14159265358979323846f;

// Create a canonical edge key for adjacency lookup
uint64_t edgeKey(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(a) << 32) | b;
}

} // anonymous namespace

Vec3 NavMesh::getGravityForFace(int faceIdx) const {
    Triangle tri = mesh_.getTriangle(static_cast<size_t>(faceIdx));
    Vec3 center = tri.center();

    for (auto& gb : config_.gravityBoxes) {
        if (gb.box.contains(center)) {
            return gb.gravityDir;
        }
    }
    return config_.defaultGravity;
}

void NavMesh::buildAdjacency() {
    size_t numFaces = mesh_.triangleCount();
    adjacency_.assign(numFaces, {});

    // Build edge -> face map
    std::unordered_map<uint64_t, std::vector<int>> edgeFaces;
    for (size_t i = 0; i < numFaces; ++i) {
        uint32_t i0 = mesh_.indices[i * 3];
        uint32_t i1 = mesh_.indices[i * 3 + 1];
        uint32_t i2 = mesh_.indices[i * 3 + 2];

        edgeFaces[edgeKey(i0, i1)].push_back(static_cast<int>(i));
        edgeFaces[edgeKey(i1, i2)].push_back(static_cast<int>(i));
        edgeFaces[edgeKey(i2, i0)].push_back(static_cast<int>(i));
    }

    // Build adjacency from shared edges
    for (auto& [key, faces] : edgeFaces) {
        for (size_t a = 0; a < faces.size(); ++a) {
            for (size_t b = a + 1; b < faces.size(); ++b) {
                adjacency_[faces[a]].push_back(faces[b]);
                adjacency_[faces[b]].push_back(faces[a]);
            }
        }
    }
}

void NavMesh::classifyFaces() {
    size_t numFaces = mesh_.triangleCount();
    faceWalkable_.assign(numFaces, false);
    walkableFaces_.clear();

    float maxSlopeRad = config_.maxSlopeAngle * kPi / 180.0f;
    float cosSlopeThreshold = std::cos(maxSlopeRad);

    for (size_t i = 0; i < numFaces; ++i) {
        Triangle tri = mesh_.getTriangle(i);
        Vec3 normal = tri.normal();
        Vec3 gravity = getGravityForFace(static_cast<int>(i));
        Vec3 up = Vec3{0, 0, 0} - gravity; // up = -gravity

        float d = normal.dot(up);
        if (d >= cosSlopeThreshold) {
            faceWalkable_[i] = true;
            walkableFaces_.push_back(static_cast<int>(i));
        }
    }
}

void NavMesh::build(const SimplifiedMesh& mesh, const NavConfig& config) {
    mesh_ = mesh;
    config_ = config;
    buildAdjacency();
    classifyFaces();
}

int NavMesh::findNearestFace(const Vec3& point) const {
    int best = -1;
    float bestDist = std::numeric_limits<float>::max();
    for (int idx : walkableFaces_) {
        Vec3 center = mesh_.getTriangle(static_cast<size_t>(idx)).center();
        float d = (center - point).lengthSq();
        if (d < bestDist) {
            bestDist = d;
            best = idx;
        }
    }
    return best;
}

std::vector<Vec3> NavMesh::findPath(const Vec3& start, const Vec3& end) const {
    int startFace = findNearestFace(start);
    int endFace = findNearestFace(end);

    if (startFace < 0 || endFace < 0) return {};
    if (startFace == endFace) {
        return {start, end};
    }

    // A* over walkable faces
    struct Node {
        int face;
        float g, f;
        bool operator>(const Node& o) const { return f > o.f; }
    };

    Vec3 goalCenter = mesh_.getTriangle(static_cast<size_t>(endFace)).center();

    std::unordered_map<int, float> gScore;
    std::unordered_map<int, int> cameFrom;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    gScore[startFace] = 0;
    float h = (mesh_.getTriangle(static_cast<size_t>(startFace)).center() - goalCenter).length();
    open.push({startFace, 0, h});

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        if (current.face == endFace) {
            // Reconstruct path
            std::vector<Vec3> path;
            int f = endFace;
            while (f != startFace) {
                path.push_back(mesh_.getTriangle(static_cast<size_t>(f)).center());
                f = cameFrom[f];
            }
            path.push_back(mesh_.getTriangle(static_cast<size_t>(startFace)).center());
            std::reverse(path.begin(), path.end());
            // Replace first and last with actual start/end
            path.front() = start;
            path.back() = end;
            return path;
        }

        if (current.g > gScore.count(current.face) ? std::numeric_limits<float>::max() : gScore[current.face] + 1e-5f) {
            // Stale entry
            if (gScore.count(current.face) && current.g > gScore[current.face] + 1e-5f)
                continue;
        }

        Vec3 curCenter = mesh_.getTriangle(static_cast<size_t>(current.face)).center();

        for (int neighbor : adjacency_[current.face]) {
            if (!faceWalkable_[neighbor]) continue;

            Vec3 nCenter = mesh_.getTriangle(static_cast<size_t>(neighbor)).center();
            float tentG = current.g + (nCenter - curCenter).length();

            auto it = gScore.find(neighbor);
            if (it == gScore.end() || tentG < it->second) {
                gScore[neighbor] = tentG;
                cameFrom[neighbor] = current.face;
                float hh = (nCenter - goalCenter).length();
                open.push({neighbor, tentG, tentG + hh});
            }
        }
    }

    return {}; // no path found
}

} // namespace voxnav
