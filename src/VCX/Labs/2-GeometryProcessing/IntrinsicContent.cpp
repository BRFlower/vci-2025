#include "Labs/2-GeometryProcessing/IntrinsicContent.h"
#include "Engine/loader.h"

#include <filesystem>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace VCX::Labs::GeometryProcessing {

static std::vector<IntrinsicContent::MeshItem> LoadIntrinsicModels() {
    std::vector<IntrinsicContent::MeshItem> models;

    // ⚠️ 你自己的 mesh 文件夹
    // 放在 assets 下，xmake 会自动拷贝
    const fs::path folder = "assets/intrinsic_meshes";

    if (!fs::exists(folder)) {
        spdlog::warn("IntrinsicContent: folder {} does not exist", folder.string());
        return models;
    }

    for (auto const& entry : fs::directory_iterator(folder)) {
        if (!entry.is_regular_file()) continue;

        auto ext = entry.path().extension().string();
        if (ext != ".obj" && ext != ".ply") continue;

        IntrinsicContent::MeshItem item;
        item.name = entry.path().filename().string();
        item.mesh = Engine::LoadSurfaceMesh(entry.path().string(), true);
        item.mesh.NormalizePositions();

        models.push_back(std::move(item));
    }

    if (models.empty()) {
        spdlog::warn("IntrinsicContent: no valid mesh found in {}", folder.string());
    }

    return models;
}

// 静态初始化
std::vector<IntrinsicContent::MeshItem> IntrinsicContent::Models = LoadIntrinsicModels();

}
