#pragma once
#include <vector>
#include <string>
#include "Engine/SurfaceMesh.h"

namespace VCX::Labs::GeometryProcessing {

struct IntrinsicContent {
    static std::vector<Engine::SurfaceMesh> Meshes;
    static std::vector<std::string> Names;
};

}
