#include "camera.h"
#include <cmath>
#include <cstring>

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kDeg2Rad = kPi / 180.0f;

void Camera::orbit(float dx, float dy) {
    yaw += dx * 0.3f;
    pitch += dy * 0.3f;
    if (pitch > 89) pitch = 89;
    if (pitch < -89) pitch = -89;
}

void Camera::pan(float dx, float dy) {
    float yr = yaw * kDeg2Rad;
    float rightX = std::cos(yr), rightZ = -std::sin(yr);
    float upX = 0, upY = 1, upZ = 0;
    float scale = distance * 0.002f;
    targetX += (-rightX * dx + upX * dy) * scale;
    targetY += (upY * dy) * scale;
    targetZ += (-rightZ * dx + upZ * dy) * scale;
}

void Camera::zoom(float delta) {
    distance -= delta * distance * 0.1f;
    if (distance < 0.5f) distance = 0.5f;
    if (distance > 500) distance = 500;
}

void Camera::fpsMove(float forward, float right, float up, float dt) {
    float yr = yaw * kDeg2Rad, pr = pitch * kDeg2Rad;
    float fx = -std::sin(yr) * std::cos(pr);
    float fy = std::sin(pr);
    float fz = -std::cos(yr) * std::cos(pr);
    float rx = std::cos(yr), rz = -std::sin(yr);
    float speed = 10.0f * dt;
    posX += (fx * forward + rx * right) * speed;
    posY += (fy * forward + up) * speed;
    posZ += (fz * forward + rz * right) * speed;
}

void Camera::fpsLook(float dx, float dy) {
    yaw += dx * 0.1f;
    pitch += dy * 0.1f;
    if (pitch > 89) pitch = 89;
    if (pitch < -89) pitch = -89;
}

void Camera::getPosition(float& x, float& y, float& z) const {
    if (fps) {
        x = posX; y = posY; z = posZ;
    } else {
        float yr = yaw * kDeg2Rad, pr = pitch * kDeg2Rad;
        x = targetX + distance * std::cos(pr) * std::sin(yr);
        y = targetY + distance * std::sin(pr);
        z = targetZ + distance * std::cos(pr) * std::cos(yr);
    }
}

void Camera::getViewMatrix(float out[16]) const {
    float ex, ey, ez;
    getPosition(ex, ey, ez);
    float tx, ty, tz;
    if (fps) {
        float yr = yaw * kDeg2Rad, pr = pitch * kDeg2Rad;
        tx = ex - std::sin(yr) * std::cos(pr);
        ty = ey + std::sin(pr);
        tz = ez - std::cos(yr) * std::cos(pr);
    } else {
        tx = targetX; ty = targetY; tz = targetZ;
    }

    // lookAt
    float fx = tx - ex, fy = ty - ey, fz = tz - ez;
    float fl = std::sqrt(fx*fx + fy*fy + fz*fz);
    if (fl < 1e-6f) fl = 1;
    fx /= fl; fy /= fl; fz /= fl;

    float ux = 0, uy = 1, uz = 0;
    float sx = fy * uz - fz * uy;
    float sy = fz * ux - fx * uz;
    float sz = fx * uy - fy * ux;
    float sl = std::sqrt(sx*sx + sy*sy + sz*sz);
    if (sl < 1e-6f) sl = 1;
    sx /= sl; sy /= sl; sz /= sl;

    float uux = sy * fz - sz * fy;
    float uuy = sz * fx - sx * fz;
    float uuz = sx * fy - sy * fx;

    std::memset(out, 0, 16 * sizeof(float));
    out[0] = sx;  out[4] = sy;  out[8]  = sz;  out[12] = -(sx*ex + sy*ey + sz*ez);
    out[1] = uux; out[5] = uuy; out[9]  = uuz; out[13] = -(uux*ex + uuy*ey + uuz*ez);
    out[2] = -fx; out[6] = -fy; out[10] = -fz; out[14] = (fx*ex + fy*ey + fz*ez);
    out[3] = 0;   out[7] = 0;   out[11] = 0;   out[15] = 1;
}
