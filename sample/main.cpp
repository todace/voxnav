#define GL_GLEXT_PROTOTYPES
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"

#include "camera.h"
#include "renderer.h"
#include "voxnav/math_types.h"
#include "voxnav/obj_loader.h"
#include "voxnav/build_request.h"

#include <string>
#include <vector>
#include <chrono>
#include <cstdio>

static const char* kMeshFiles[] = {"meshes/nav_test.obj", "meshes/dungeon.obj", "meshes/undulating.obj"};
static const int kNumMeshes = 3;

int main(int, char**) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* window = SDL_CreateWindow("VoxNav Sample",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplSDL2_InitForOpenGL(window, gl);
    ImGui_ImplOpenGL3_Init("#version 330");

    Renderer renderer;
    renderer.init();

    Camera camera;
    camera.distance = 20;

    voxnav::BuildSystem buildSys(2);
    voxnav::Mesh sourceMesh;
    voxnav::BuildResult buildResult;
    bool hasBuildResult = false;
    voxnav::BuildHandle currentBuild = 0;
    bool buildPending = false;
    float buildTimeMs = 0;

    GLMeshBuffer origBuf, simplifiedBuf, walkableBuf, pathBuf;
    bool showOrig = true, showSimplified = true, showWalkable = true, showPath = true;

    float voxelSize = 0.25f;
    float agentRadius = 0.5f;
    float maxSlope = 45.0f;
    int meshIdx = 0;

    // Navigation test
    voxnav::Vec3 pathStart = {0, 0, 0}, pathEnd = {0, 0, 0};
    std::vector<voxnav::Vec3> pathPoints;
    bool hasPath = false;

    // Gravity boxes
    std::vector<voxnav::GravityBox> gravityBoxes;

    // Load initial mesh
    auto loadMesh = [&](int idx) {
        sourceMesh = {};
        voxnav::loadOBJ(kMeshFiles[idx], sourceMesh);
    };
    loadMesh(0);

    auto doRebuild = [&]() {
        if (sourceMesh.vertices.empty()) return;
        voxnav::BuildParams params;
        params.region = sourceMesh.computeBounds();
        // Expand region slightly
        params.region.min = params.region.min - voxnav::Vec3{1,1,1};
        params.region.max = params.region.max + voxnav::Vec3{1,1,1};
        params.sourceMesh = &sourceMesh;
        params.voxelSize = voxelSize;
        params.agentRadius = agentRadius;
        params.navConfig.maxSlopeAngle = maxSlope;
        params.navConfig.defaultGravity = {0, -1, 0};
        params.navConfig.gravityBoxes = gravityBoxes;
        currentBuild = buildSys.requestBuild(params);
        buildPending = true;
        hasBuildResult = false;
    };

    auto rebuildBuffers = [&]() {
        Renderer::buildBuffer(sourceMesh, origBuf);
        Renderer::buildBuffer(buildResult.simplifiedMesh, simplifiedBuf);

        // Build walkable faces buffer
        auto& wf = buildResult.navMesh.getWalkableFaces();
        auto& sm = buildResult.navMesh.getMesh();
        voxnav::SimplifiedMesh walkMesh;
        for (int fi : wf) {
            auto tri = sm.getTriangle(fi);
            uint32_t base = static_cast<uint32_t>(walkMesh.vertices.size());
            walkMesh.vertices.push_back(tri.v0);
            walkMesh.vertices.push_back(tri.v1);
            walkMesh.vertices.push_back(tri.v2);
            walkMesh.indices.push_back(base);
            walkMesh.indices.push_back(base + 1);
            walkMesh.indices.push_back(base + 2);
        }
        Renderer::buildBuffer(walkMesh, walkableBuf);
    };

    doRebuild();

    auto startTime = std::chrono::steady_clock::now();
    bool running = true;

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            ImGui_ImplSDL2_ProcessEvent(&ev);
            if (ev.type == SDL_QUIT) running = false;
            if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_TAB)
                camera.fps = !camera.fps;
            if (!ImGui::GetIO().WantCaptureMouse) {
                if (ev.type == SDL_MOUSEWHEEL) camera.zoom(static_cast<float>(ev.wheel.y));
                if (ev.type == SDL_MOUSEMOTION) {
                    if (ev.motion.state & SDL_BUTTON_RMASK)
                        camera.orbit(static_cast<float>(ev.motion.xrel), static_cast<float>(ev.motion.yrel));
                    if (ev.motion.state & SDL_BUTTON_MMASK)
                        camera.pan(static_cast<float>(ev.motion.xrel), static_cast<float>(ev.motion.yrel));
                }
            }
            if (!ImGui::GetIO().WantCaptureKeyboard && camera.fps) {
                // FPS handled below
            }
        }

        // FPS movement
        if (camera.fps) {
            const Uint8* ks = SDL_GetKeyboardState(nullptr);
            float fwd = 0, rht = 0, up = 0;
            if (ks[SDL_SCANCODE_W]) fwd += 1;
            if (ks[SDL_SCANCODE_S]) fwd -= 1;
            if (ks[SDL_SCANCODE_D]) rht += 1;
            if (ks[SDL_SCANCODE_A]) rht -= 1;
            if (ks[SDL_SCANCODE_SPACE]) up += 1;
            if (ks[SDL_SCANCODE_LCTRL]) up -= 1;
            float speed = ks[SDL_SCANCODE_LSHIFT] ? 3.0f : 1.0f;
            camera.fpsMove(fwd * speed, rht * speed, up * speed, 1.0f / 60.0f);
        }

        // Check build completion
        if (buildPending && buildSys.isComplete(currentBuild)) {
            auto endTime = std::chrono::steady_clock::now();
            buildTimeMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();
            buildResult = buildSys.getResult(currentBuild);
            hasBuildResult = true;
            buildPending = false;
            rebuildBuffers();
        }

        // ImGui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Build Parameters");
        ImGui::SliderFloat("Voxel Size", &voxelSize, 0.05f, 2.0f);
        ImGui::SliderFloat("Agent Radius", &agentRadius, 0.1f, 5.0f);
        ImGui::SliderFloat("Max Slope", &maxSlope, 0.0f, 90.0f);
        if (ImGui::Combo("Mesh", &meshIdx, "nav_test.obj\0dungeon.obj\0undulating.obj\0")) {
            loadMesh(meshIdx);
        }
        if (ImGui::Button("Rebuild")) {
            startTime = std::chrono::steady_clock::now();
            doRebuild();
        }
        if (buildPending) ImGui::Text("Building...");
        else ImGui::Text("Build time: %.1f ms", buildTimeMs);
        ImGui::End();

        ImGui::Begin("Visualization");
        ImGui::Checkbox("Original Mesh", &showOrig);
        ImGui::Checkbox("Simplified Mesh", &showSimplified);
        ImGui::Checkbox("Walkable Faces", &showWalkable);
        ImGui::Checkbox("Path", &showPath);
        ImGui::End();

        ImGui::Begin("Gravity Boxes");
        for (size_t i = 0; i < gravityBoxes.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            ImGui::Text("Box %zu: grav=(%.1f,%.1f,%.1f)", i,
                gravityBoxes[i].gravityDir.x, gravityBoxes[i].gravityDir.y, gravityBoxes[i].gravityDir.z);
            ImGui::SameLine();
            if (ImGui::Button("Delete")) {
                gravityBoxes.erase(gravityBoxes.begin() + i);
                --i;
            }
            ImGui::PopID();
        }
        ImGui::End();

        ImGui::Begin("Navigation Test");
        ImGui::Text("Start: (%.2f, %.2f, %.2f)", pathStart.x, pathStart.y, pathStart.z);
        ImGui::Text("End: (%.2f, %.2f, %.2f)", pathEnd.x, pathEnd.y, pathEnd.z);
        ImGui::InputFloat3("Start##nav", &pathStart.x);
        ImGui::InputFloat3("End##nav", &pathEnd.x);
        if (ImGui::Button("Find Path") && hasBuildResult) {
            pathPoints = buildResult.navMesh.findPath(pathStart, pathEnd);
            hasPath = !pathPoints.empty();
            if (hasPath) Renderer::buildLineBuffer(pathPoints, pathBuf);
        }
        if (hasPath) {
            float len = 0;
            for (size_t i = 1; i < pathPoints.size(); ++i)
                len += (pathPoints[i] - pathPoints[i-1]).length();
            ImGui::Text("Path length: %.2f (%zu points)", len, pathPoints.size());
        } else {
            ImGui::Text("No path");
        }
        ImGui::End();

        // Render
        int w, h;
        SDL_GetWindowSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        float proj[16], view[16], mvp[16];
        makePerspective(proj, 60.0f, static_cast<float>(w) / std::max(h, 1), 0.1f, 500.0f);
        camera.getViewMatrix(view);
        matMul(mvp, proj, view);

        renderer.use();
        renderer.setMVP(mvp);
        renderer.setLightDir(0.3f, 1.0f, 0.5f);

        if (showOrig && hasBuildResult) {
            renderer.setColor(0.6f, 0.6f, 0.6f, 0.3f);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            origBuf.draw();
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        if (showSimplified && hasBuildResult) {
            renderer.setColor(0.3f, 0.5f, 0.8f, 0.5f);
            simplifiedBuf.draw();
        }

        if (showWalkable && hasBuildResult) {
            renderer.setColor(0.2f, 0.9f, 0.3f, 0.7f);
            walkableBuf.draw();
        }

        if (showPath && hasPath) {
            renderer.setColor(1.0f, 1.0f, 0.0f, 1.0f);
            glLineWidth(3.0f);
            if (pathBuf.vao) {
                glBindVertexArray(pathBuf.vao);
                glDrawElements(GL_LINE_STRIP, pathBuf.indexCount, GL_UNSIGNED_INT, nullptr);
                glBindVertexArray(0);
            }
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    origBuf.destroy();
    simplifiedBuf.destroy();
    walkableBuf.destroy();
    pathBuf.destroy();
    renderer.destroy();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
