#define GL_GLEXT_PROTOTYPES
#include "renderer.h"
#include <SDL2/SDL_opengl.h>
#include <cmath>
#include <cstring>
#include <cstdio>

static const char* vertSrc = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 uMVP;
out vec3 vNormal;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vNormal = aNormal;
}
)";

static const char* fragSrc = R"(
#version 330 core
in vec3 vNormal;
uniform vec4 uColor;
uniform vec3 uLightDir;
out vec4 FragColor;
void main() {
    vec3 n = normalize(vNormal);
    float diff = max(dot(n, normalize(uLightDir)), 0.0) * 0.7 + 0.3;
    FragColor = vec4(uColor.rgb * diff, uColor.a);
}
)";

static unsigned int compileShader(GLenum type, const char* src) {
    unsigned int s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    int ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetShaderInfoLog(s, 512, nullptr, log);
        fprintf(stderr, "Shader error: %s\n", log);
    }
    return s;
}

void Renderer::init() {
    unsigned int vs = compileShader(GL_VERTEX_SHADER, vertSrc);
    unsigned int fs = compileShader(GL_FRAGMENT_SHADER, fragSrc);
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vs);
    glAttachShader(shaderProgram, fs);
    glLinkProgram(shaderProgram);
    glDeleteShader(vs);
    glDeleteShader(fs);
    uMVP = glGetUniformLocation(shaderProgram, "uMVP");
    uColor = glGetUniformLocation(shaderProgram, "uColor");
    uLightDir = glGetUniformLocation(shaderProgram, "uLightDir");
}

void Renderer::destroy() {
    if (shaderProgram) glDeleteProgram(shaderProgram);
}

void Renderer::use() { glUseProgram(shaderProgram); }
void Renderer::setMVP(const float mvp[16]) { glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp); }
void Renderer::setColor(float r, float g, float b, float a) { glUniform4f(uColor, r, g, b, a); }
void Renderer::setLightDir(float x, float y, float z) { glUniform3f(uLightDir, x, y, z); }

void GLMeshBuffer::upload(const std::vector<float>& verts, const std::vector<uint32_t>& indices) {
    if (!vao) { glGenVertexArrays(1, &vao); glGenBuffers(1, &vbo); glGenBuffers(1, &ebo); }
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    indexCount = static_cast<int>(indices.size());
    glBindVertexArray(0);
}

void GLMeshBuffer::draw() const {
    if (!vao || indexCount == 0) return;
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void GLMeshBuffer::destroy() {
    if (vao) { glDeleteVertexArrays(1, &vao); glDeleteBuffers(1, &vbo); glDeleteBuffers(1, &ebo); vao = 0; }
}

static void addTriWithNormal(std::vector<float>& verts, std::vector<uint32_t>& idx,
                              const voxnav::Vec3& v0, const voxnav::Vec3& v1, const voxnav::Vec3& v2) {
    voxnav::Vec3 n = (v1 - v0).cross(v2 - v0).normalized();
    uint32_t base = static_cast<uint32_t>(verts.size() / 6);
    auto push = [&](const voxnav::Vec3& v) {
        verts.push_back(v.x); verts.push_back(v.y); verts.push_back(v.z);
        verts.push_back(n.x); verts.push_back(n.y); verts.push_back(n.z);
    };
    push(v0); push(v1); push(v2);
    idx.push_back(base); idx.push_back(base+1); idx.push_back(base+2);
}

void Renderer::buildBuffer(const voxnav::SimplifiedMesh& mesh, GLMeshBuffer& buf) {
    std::vector<float> verts;
    std::vector<uint32_t> idx;
    for (size_t i = 0; i < mesh.triangleCount(); ++i) {
        auto t = mesh.getTriangle(i);
        addTriWithNormal(verts, idx, t.v0, t.v1, t.v2);
    }
    buf.upload(verts, idx);
}

void Renderer::buildBuffer(const voxnav::Mesh& mesh, GLMeshBuffer& buf) {
    std::vector<float> verts;
    std::vector<uint32_t> idx;
    for (size_t i = 0; i < mesh.triangleCount(); ++i) {
        auto t = mesh.getTriangle(i);
        addTriWithNormal(verts, idx, t.v0, t.v1, t.v2);
    }
    buf.upload(verts, idx);
}

void Renderer::buildLineBuffer(const std::vector<voxnav::Vec3>& points, GLMeshBuffer& buf) {
    std::vector<float> verts;
    std::vector<uint32_t> idx;
    for (size_t i = 0; i < points.size(); ++i) {
        verts.push_back(points[i].x); verts.push_back(points[i].y); verts.push_back(points[i].z);
        verts.push_back(0); verts.push_back(1); verts.push_back(0); // dummy normal
        idx.push_back(static_cast<uint32_t>(i));
    }
    buf.upload(verts, idx);
}

void makePerspective(float out[16], float fovDeg, float aspect, float near, float far) {
    std::memset(out, 0, 16 * sizeof(float));
    float f = 1.0f / std::tan(fovDeg * 3.14159265f / 360.0f);
    out[0] = f / aspect;
    out[5] = f;
    out[10] = (far + near) / (near - far);
    out[11] = -1;
    out[14] = 2.0f * far * near / (near - far);
}

void matMul(float out[16], const float a[16], const float b[16]) {
    float tmp[16];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            tmp[j * 4 + i] = 0;
            for (int k = 0; k < 4; ++k)
                tmp[j * 4 + i] += a[k * 4 + i] * b[j * 4 + k];
        }
    std::memcpy(out, tmp, sizeof(tmp));
}
