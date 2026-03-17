# Sample app - requires SDL2, OpenGL, and vendored ImGui
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

# ImGui sources (vendored)
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

add_executable(voxnav_sample
    main.cpp
    camera.cpp
    renderer.cpp
    ${IMGUI_SOURCES}
)

target_include_directories(voxnav_sample PRIVATE
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(voxnav_sample PRIVATE
    voxnav
    ${SDL2_LIBRARIES}
    OpenGL::GL
)

# ImGui uses its own built-in GL loader (imgui_impl_opengl3_loader.h)
