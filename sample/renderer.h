#pragma once
#include "voxnav/math_types.h"
#include <vector>
#include <cstdint>

struct GLMeshBuffer {
    unsigned int vao = 0, vbo = 0, ebo = 0;
    int indexCount = 0;

    void upload(const std::vector<float>& verts, const std::vector<uint32_t>& indices);
    void draw() const;
    void destroy();
};

struct Renderer {
    unsigned int shaderProgram = 0;
    int uMVP = -1, uColor = -1, uLightDir = -1;

    void init();
    void destroy();
    void setMVP(const float mvp[16]);
    void setColor(float r, float g, float b, float a);
    void setLightDir(float x, float y, float z);
    void use();

    // Build vertex buffer with normals from mesh
    static void buildBuffer(const voxnav::SimplifiedMesh& mesh, GLMeshBuffer& buf);
    static void buildBuffer(const voxnav::Mesh& mesh, GLMeshBuffer& buf);
    static void buildLineBuffer(const std::vector<voxnav::Vec3>& points, GLMeshBuffer& buf);
};

void makePerspective(float out[16], float fovDeg, float aspect, float near, float far);
void matMul(float out[16], const float a[16], const float b[16]);
