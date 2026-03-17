#include "voxnav/obj_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace voxnav {

bool loadOBJ(const std::string& path, Mesh& mesh) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    mesh.vertices.clear();
    mesh.indices.clear();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            Vec3 v;
            iss >> v.x >> v.y >> v.z;
            mesh.vertices.push_back(v);
        } else if (prefix == "f") {
            // Parse face — may have v, v/vt, v/vt/vn, or v//vn
            std::vector<uint32_t> faceIndices;
            std::string token;
            while (iss >> token) {
                // Extract vertex index (before first '/')
                size_t slash = token.find('/');
                std::string idxStr = (slash != std::string::npos) ? token.substr(0, slash) : token;
                int idx = std::stoi(idxStr);
                // OBJ indices are 1-based
                faceIndices.push_back(static_cast<uint32_t>(idx - 1));
            }
            // Triangulate fan
            for (size_t i = 1; i + 1 < faceIndices.size(); ++i) {
                mesh.indices.push_back(faceIndices[0]);
                mesh.indices.push_back(faceIndices[i]);
                mesh.indices.push_back(faceIndices[i + 1]);
            }
        }
    }
    return !mesh.vertices.empty();
}

bool saveOBJ(const std::string& path, const Mesh& mesh) {
    std::ofstream file(path);
    if (!file.is_open()) return false;

    for (auto& v : mesh.vertices) {
        file << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        file << "f " << (mesh.indices[i] + 1) << " "
             << (mesh.indices[i + 1] + 1) << " "
             << (mesh.indices[i + 2] + 1) << "\n";
    }
    return true;
}

} // namespace voxnav
