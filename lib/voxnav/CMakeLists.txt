add_library(voxnav STATIC
    voxelizer.cpp
    clearance.cpp
    mesh_builder.cpp
    nav_mesh.cpp
    obj_loader.cpp
    build_request.cpp
)

target_include_directories(voxnav PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_include_directories(voxnav PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_features(voxnav PUBLIC cxx_std_17)

find_package(Threads REQUIRED)
target_link_libraries(voxnav PUBLIC Threads::Threads)
