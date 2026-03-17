#pragma once
#include <cmath>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <limits>

namespace voxnav {

struct Vec3 {
    float x = 0, y = 0, z = 0;

    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }

    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    float lengthSq() const { return x * x + y * y + z * z; }
    Vec3 normalized() const {
        float l = length();
        return l > 1e-8f ? Vec3{x / l, y / l, z / l} : Vec3{0, 0, 0};
    }

    static Vec3 min(const Vec3& a, const Vec3& b) {
        return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
    }
    static Vec3 max(const Vec3& a, const Vec3& b) {
        return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
    }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

struct AABB {
    Vec3 min, max;

    Vec3 center() const { return (min + max) * 0.5f; }
    Vec3 extent() const { return max - min; }

    bool contains(const Vec3& p) const {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }

    bool intersects(const AABB& o) const {
        return min.x <= o.max.x && max.x >= o.min.x &&
               min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }
};

struct Triangle {
    Vec3 v0, v1, v2;

    Vec3 normal() const {
        return (v1 - v0).cross(v2 - v0).normalized();
    }
    Vec3 center() const {
        return (v0 + v1 + v2) / 3.0f;
    }
    AABB bounds() const {
        return {Vec3::min(Vec3::min(v0, v1), v2), Vec3::max(Vec3::max(v0, v1), v2)};
    }
};

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<uint32_t> indices; // 3 per triangle

    size_t triangleCount() const { return indices.size() / 3; }

    Triangle getTriangle(size_t i) const {
        return {vertices[indices[i * 3]], vertices[indices[i * 3 + 1]], vertices[indices[i * 3 + 2]]};
    }

    AABB computeBounds() const {
        AABB b;
        b.min = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        b.max = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};
        for (auto& v : vertices) {
            b.min = Vec3::min(b.min, v);
            b.max = Vec3::max(b.max, v);
        }
        return b;
    }
};

struct SimplifiedMesh {
    std::vector<Vec3> vertices;
    std::vector<uint32_t> indices; // 3 per triangle

    size_t triangleCount() const { return indices.size() / 3; }

    Triangle getTriangle(size_t i) const {
        return {vertices[indices[i * 3]], vertices[indices[i * 3 + 1]], vertices[indices[i * 3 + 2]]};
    }
};

} // namespace voxnav
