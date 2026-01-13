#pragma once

#include <vector>
#include <string>
#include "Engine/SurfaceMesh.h"

namespace VCX::Labs::GeometryProcessing {

struct IntrinsicContent {
    struct MeshItem {
        std::string name;
        Engine::SurfaceMesh mesh;
    };

    // 所有 intrinsic meshes（从文件夹加载）
    static std::vector<MeshItem> Models;
};

}
