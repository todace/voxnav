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

// Gravity direction presets
static const char* kGravityDirNames[] = {"Y-down", "Y-up", "X+", "X-", "Z+", "Z-"};
static const voxnav::Vec3 kGravityDirs[] = {
    {0, -1, 0}, {0, 1, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}
};

// Gravity zone colors
static void getGravityZoneColor(const voxnav::Vec3& grav, float& r, float& g, float& b) {
    float ax = std::abs(grav.x), ay = std::abs(grav.y), az = std::abs(grav.z);
    if (ay >= ax && ay >= az) {
        if (grav.y < 0) { r = 0.2f; g = 0.9f; b = 0.3f; }       // Y-down: green
        else             { r = 0.9f; g = 0.2f; b = 0.9f; }       // Y-up: magenta
    } else if (ax >= az) {
        r = 0.2f; g = 0.3f; b = 0.9f;                            // X+/X-: blue
    } else {
        r = 0.9f; g = 0.6f; b = 0.2f;                            // Z+/Z-: orange
    }
}

// Build a wireframe box (12 edges = 24 line vertices)
static void buildWireframeBox(const voxnav::AABB& box, GLMeshBuffer& buf) {
    voxnav::Vec3 mn = box.min, mx = box.max;
    // 8 corners
    voxnav::Vec3 c[8] = {
        {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z}, {mx.x, mx.y, mn.z}, {mn.x, mx.y, mn.z},
        {mn.x, mn.y, mx.z}, {mx.x, mn.y, mx.z}, {mx.x, mx.y, mx.z}, {mn.x, mx.y, mx.z}
    };
    // 12 edges as pairs of corner indices
    static const int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7}
    };
    std::vector<float> verts;
    std::vector<uint32_t> idx;
    for (auto& e : edges) {
        uint32_t base = static_cast<uint32_t>(verts.size() / 6);
        for (int vi = 0; vi < 2; ++vi) {
            verts.push_back(c[e[vi]].x); verts.push_back(c[e[vi]].y); verts.push_back(c[e[vi]].z);
            verts.push_back(0); verts.push_back(1); verts.push_back(0); // dummy normal
        }
        idx.push_back(base); idx.push_back(base + 1);
    }
    buf.upload(verts, idx);
}

// Build voxel debug point cloud (solid voxels as GL_POINTS)
static void buildVoxelPointCloud(const voxnav::VoxelGrid& grid, GLMeshBuffer& buf) {
    std::vector<float> verts;
    std::vector<uint32_t> idx;
    uint32_t count = 0;
    for (int z = 0; z < grid.nz; ++z) {
        for (int y = 0; y < grid.ny; ++y) {
            for (int x = 0; x < grid.nx; ++x) {
                if (!grid.get(x, y, z)) continue;
                // Check if surface voxel (has at least one empty neighbor)
                bool surface = false;
                for (int d = 0; d < 6 && !surface; ++d) {
                    int nx = x + (d == 0 ? 1 : d == 1 ? -1 : 0);
                    int ny = y + (d == 2 ? 1 : d == 3 ? -1 : 0);
                    int nz = z + (d == 4 ? 1 : d == 5 ? -1 : 0);
                    if (nx < 0 || nx >= grid.nx || ny < 0 || ny >= grid.ny || nz < 0 || nz >= grid.nz)
                        surface = true;
                    else if (!grid.get(nx, ny, nz))
                        surface = true;
                }
                if (!surface) continue;
                float px = grid.bounds.min.x + (x + 0.5f) * grid.voxelSize;
                float py = grid.bounds.min.y + (y + 0.5f) * grid.voxelSize;
                float pz = grid.bounds.min.z + (z + 0.5f) * grid.voxelSize;
                verts.push_back(px); verts.push_back(py); verts.push_back(pz);
                verts.push_back(0); verts.push_back(1); verts.push_back(0); // dummy normal
                idx.push_back(count++);
            }
        }
    }
    buf.upload(verts, idx);
}

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

    GLMeshBuffer origBuf, simplifiedBuf, pathBuf, voxelBuf;
    // Per-gravity-zone walkable face buffers
    GLMeshBuffer walkBufGreen, walkBufMagenta, walkBufBlue, walkBufOrange;
    // Gravity box wireframe buffers
    std::vector<GLMeshBuffer> gravBoxBufs;

    bool showOrig = true, showSimplified = true, showWalkable = true, showPath = true;
    bool showVoxels = false;

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

    // New gravity box input fields
    float newBoxMin[3] = {-1, -1, -1};
    float newBoxMax[3] = {1, 1, 1};
    int newGravDirIdx = 0;

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

    auto rebuildGravBoxBufs = [&]() {
        for (auto& b : gravBoxBufs) b.destroy();
        gravBoxBufs.resize(gravityBoxes.size());
        for (size_t i = 0; i < gravityBoxes.size(); ++i) {
            buildWireframeBox(gravityBoxes[i].box, gravBoxBufs[i]);
        }
    };

    auto rebuildBuffers = [&]() {
        Renderer::buildBuffer(sourceMesh, origBuf);
        Renderer::buildBuffer(buildResult.simplifiedMesh, simplifiedBuf);

        // Build per-gravity-zone walkable face buffers (Issue 3)
        auto& wf = buildResult.navMesh.getWalkableFaces();
        auto& sm = buildResult.navMesh.getMesh();

        voxnav::SimplifiedMesh meshGreen, meshMagenta, meshBlue, meshOrange;
        for (int fi : wf) {
            auto tri = sm.getTriangle(fi);
            voxnav::Vec3 grav = buildResult.navMesh.getGravityForFace(fi);
            float cr, cg, cb;
            getGravityZoneColor(grav, cr, cg, cb);

            voxnav::SimplifiedMesh* target;
            if (cr > 0.5f && cg < 0.5f && cb > 0.5f) target = &meshMagenta;
            else if (cr < 0.5f && cg < 0.5f && cb > 0.5f) target = &meshBlue;
            else if (cr > 0.5f && cg > 0.5f && cb < 0.5f) target = &meshOrange;
            else target = &meshGreen;

            uint32_t base = static_cast<uint32_t>(target->vertices.size());
            target->vertices.push_back(tri.v0);
            target->vertices.push_back(tri.v1);
            target->vertices.push_back(tri.v2);
            target->indices.push_back(base);
            target->indices.push_back(base + 1);
            target->indices.push_back(base + 2);
        }
        Renderer::buildBuffer(meshGreen, walkBufGreen);
        Renderer::buildBuffer(meshMagenta, walkBufMagenta);
        Renderer::buildBuffer(meshBlue, walkBufBlue);
        Renderer::buildBuffer(meshOrange, walkBufOrange);

        // Build voxel debug visualization (Issue 4)
        buildVoxelPointCloud(buildResult.dilatedGrid, voxelBuf);

        rebuildGravBoxBufs();
    };

    doRebuild();

    auto startTime = std::chrono::steady_clock::now();
    bool running = true;

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            ImGui_ImplSDL2_ProcessEvent(&ev);
            if (ev.type == SDL_QUIT) running = false;
            if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_TAB) {
                camera.fps = !camera.fps;
                // Issue 2: Toggle mouse capture for FPS mode
                SDL_SetRelativeMouseMode(camera.fps ? SDL_TRUE : SDL_FALSE);
            }
            if (!ImGui::GetIO().WantCaptureMouse) {
                if (ev.type == SDL_MOUSEWHEEL) camera.zoom(static_cast<float>(ev.wheel.y));
                if (ev.type == SDL_MOUSEMOTION) {
                    // Issue 2: FPS mouse look when captured
                    if (camera.fps) {
                        camera.fpsLook(static_cast<float>(ev.motion.xrel),
                                       static_cast<float>(ev.motion.yrel));
                    } else {
                        if (ev.motion.state & SDL_BUTTON_RMASK)
                            camera.orbit(static_cast<float>(ev.motion.xrel), static_cast<float>(ev.motion.yrel));
                        if (ev.motion.state & SDL_BUTTON_MMASK)
                            camera.pan(static_cast<float>(ev.motion.xrel), static_cast<float>(ev.motion.yrel));
                    }
                }
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
        ImGui::Checkbox("Show Voxels", &showVoxels);
        ImGui::End();

        // Issue 1: Gravity Box editor with add functionality
        ImGui::Begin("Gravity Boxes");
        for (size_t i = 0; i < gravityBoxes.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            auto& gb = gravityBoxes[i];
            ImGui::Text("Box %zu: (%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f) grav=(%.1f,%.1f,%.1f)",
                i, gb.box.min.x, gb.box.min.y, gb.box.min.z,
                gb.box.max.x, gb.box.max.y, gb.box.max.z,
                gb.gravityDir.x, gb.gravityDir.y, gb.gravityDir.z);
            ImGui::SameLine();
            if (ImGui::Button("Delete")) {
                gravityBoxes.erase(gravityBoxes.begin() + i);
                --i;
            }
            ImGui::PopID();
        }
        ImGui::Separator();
        ImGui::Text("Add New Gravity Box:");
        ImGui::InputFloat3("Min Corner", newBoxMin);
        ImGui::InputFloat3("Max Corner", newBoxMax);
        ImGui::Combo("Gravity Dir", &newGravDirIdx, "Y-down\0Y-up\0X+\0X-\0Z+\0Z-\0");
        if (ImGui::Button("Add Gravity Box")) {
            voxnav::GravityBox gb;
            gb.box.min = {newBoxMin[0], newBoxMin[1], newBoxMin[2]};
            gb.box.max = {newBoxMax[0], newBoxMax[1], newBoxMax[2]};
            gb.gravityDir = kGravityDirs[newGravDirIdx];
            gravityBoxes.push_back(gb);
            rebuildGravBoxBufs();
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

        // Issue 3: Draw walkable faces colored by gravity zone
        if (showWalkable && hasBuildResult) {
            renderer.setColor(0.2f, 0.9f, 0.3f, 0.7f);
            walkBufGreen.draw();
            renderer.setColor(0.9f, 0.2f, 0.9f, 0.7f);
            walkBufMagenta.draw();
            renderer.setColor(0.2f, 0.3f, 0.9f, 0.7f);
            walkBufBlue.draw();
            renderer.setColor(0.9f, 0.6f, 0.2f, 0.7f);
            walkBufOrange.draw();
        }

        // Issue 4: Voxel debug visualization
        if (showVoxels && hasBuildResult) {
            renderer.setColor(1.0f, 0.4f, 0.1f, 0.6f);
            glEnable(GL_PROGRAM_POINT_SIZE);
            glPointSize(3.0f);
            if (voxelBuf.vao) {
                glBindVertexArray(voxelBuf.vao);
                glDrawElements(GL_POINTS, voxelBuf.indexCount, GL_UNSIGNED_INT, nullptr);
                glBindVertexArray(0);
            }
        }

        // Issue 1: Render gravity box wireframes
        for (size_t i = 0; i < gravBoxBufs.size(); ++i) {
            float cr, cg, cb;
            getGravityZoneColor(gravityBoxes[i].gravityDir, cr, cg, cb);
            renderer.setColor(cr, cg, cb, 1.0f);
            glLineWidth(2.0f);
            if (gravBoxBufs[i].vao) {
                glBindVertexArray(gravBoxBufs[i].vao);
                glDrawElements(GL_LINES, gravBoxBufs[i].indexCount, GL_UNSIGNED_INT, nullptr);
                glBindVertexArray(0);
            }
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
    walkBufGreen.destroy();
    walkBufMagenta.destroy();
    walkBufBlue.destroy();
    walkBufOrange.destroy();
    pathBuf.destroy();
    voxelBuf.destroy();
    for (auto& b : gravBoxBufs) b.destroy();
    renderer.destroy();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
