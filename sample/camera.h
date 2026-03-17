#pragma once

struct Camera {
    float posX = 0, posY = 5, posZ = 10;
    float targetX = 0, targetY = 0, targetZ = 0;
    float yaw = 0, pitch = -30;
    float distance = 15;
    bool fps = false;

    void orbit(float dx, float dy);
    void pan(float dx, float dy);
    void zoom(float delta);
    void fpsMove(float forward, float right, float up, float dt);
    void fpsLook(float dx, float dy);
    void getViewMatrix(float out[16]) const;
    void getPosition(float& x, float& y, float& z) const;
};
