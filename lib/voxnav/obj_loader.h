#pragma once
#include "voxnav/math_types.h"
#include <string>

namespace voxnav {

// Load an OBJ file (vertices + triangulated faces only)
bool loadOBJ(const std::string& path, Mesh& mesh);

// Save a mesh as OBJ
bool saveOBJ(const std::string& path, const Mesh& mesh);

} // namespace voxnav
