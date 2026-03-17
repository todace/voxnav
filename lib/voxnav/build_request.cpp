#include "voxnav/build_request.h"
#include "voxnav/clearance.h"
#include "voxnav/mesh_builder.h"

namespace voxnav {

BuildSystem::BuildSystem(size_t numThreads) : pool_(numThreads) {}

BuildHandle BuildSystem::requestBuild(const BuildParams& params) {
    std::lock_guard<std::mutex> lk(mutex_);
    BuildHandle handle = nextHandle_++;

    auto job = std::make_shared<Job>();

    // Copy params by value for the lambda (except sourceMesh pointer — must remain valid)
    BuildParams p = params;
    auto cancelledPtr = &job->cancelled;

    job->future = pool_.submit([p, cancelledPtr]() -> BuildResult {
        BuildResult result;

        // Step 1: Voxelize
        result.voxelGrid = voxelize(*p.sourceMesh, p.region, p.voxelSize);

        if (cancelledPtr->load()) return result;

        // Step 2: Dilate for agent clearance
        result.dilatedGrid = dilate(result.voxelGrid, p.agentRadius);

        if (cancelledPtr->load()) return result;

        // Step 3: Extract surface via Marching Cubes
        result.simplifiedMesh = extractSurface(result.dilatedGrid);

        if (cancelledPtr->load()) return result;

        // Step 4: Build navigation mesh
        result.navMesh.build(result.simplifiedMesh, p.navConfig);

        return result;
    });

    jobs_[handle] = job;
    return handle;
}

bool BuildSystem::isComplete(BuildHandle handle) const {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = jobs_.find(handle);
    if (it == jobs_.end()) return true;
    return it->second->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

BuildResult BuildSystem::getResult(BuildHandle handle) {
    std::shared_ptr<Job> job;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = jobs_.find(handle);
        if (it == jobs_.end()) return {};
        job = it->second;
    }
    BuildResult r = job->future.get();
    {
        std::lock_guard<std::mutex> lk(mutex_);
        jobs_.erase(handle);
    }
    return r;
}

void BuildSystem::cancelBuild(BuildHandle handle) {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = jobs_.find(handle);
    if (it != jobs_.end()) {
        it->second->cancelled.store(true);
    }
}

} // namespace voxnav
