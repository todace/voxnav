#pragma once
#include "voxnav/math_types.h"
#include "voxnav/voxelizer.h"
#include "voxnav/nav_mesh.h"
#include "voxnav/thread_pool.h"
#include <future>
#include <memory>
#include <atomic>

namespace voxnav {

struct BuildParams {
    AABB region;
    const Mesh* sourceMesh = nullptr;
    float voxelSize = 0.25f;
    float agentRadius = 0.5f;
    NavConfig navConfig;
};

struct BuildResult {
    VoxelGrid voxelGrid;
    VoxelGrid dilatedGrid;
    SimplifiedMesh simplifiedMesh;
    NavMesh navMesh;
};

using BuildHandle = size_t;

class BuildSystem {
public:
    explicit BuildSystem(size_t numThreads = 2);

    BuildHandle requestBuild(const BuildParams& params);
    bool isComplete(BuildHandle handle) const;
    BuildResult getResult(BuildHandle handle);
    void cancelBuild(BuildHandle handle);

private:
    ThreadPool pool_;
    mutable std::mutex mutex_;
    size_t nextHandle_ = 0;

    struct Job {
        std::future<BuildResult> future;
        std::atomic<bool> cancelled{false};
    };
    std::unordered_map<size_t, std::shared_ptr<Job>> jobs_;
};

} // namespace voxnav
