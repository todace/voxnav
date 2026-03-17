cmake_minimum_required(VERSION 3.14)
project(voxnav VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(lib/voxnav)
add_subdirectory(tests)

find_package(SDL2 QUIET)
if(SDL2_FOUND)
    add_subdirectory(sample)
else()
    message(STATUS "SDL2 not found — skipping sample app")
endif()
