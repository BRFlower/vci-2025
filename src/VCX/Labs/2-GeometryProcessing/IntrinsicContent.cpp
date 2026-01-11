#include "IntrinsicContent.h"
#include "Engine/loader.h"

#include <filesystem>

namespace fs = std::filesystem;

namespace VCX::Labs::GeometryProcessing {

static std::vector<Engine::SurfaceMesh> LoadMeshes() {
    std::vector<Engine::SurfaceMesh> meshes;

    std::string folder = "assets/intrinsic_meshes"; // 你自己的目录

    for (auto const & entry : fs::directory_iterator(folder)) {
        if (!entry.is_regular_file()) continue;

        auto ext = entry.path().extension().string();
        if (ext != ".obj" && ext != ".ply") continue;

        auto mesh = Engine::LoadSurfaceMesh(entry.path().string(), true);
        mesh.NormalizePositions();
        meshes.push_back(std::move(mesh));
    }

    return meshes;
}

static std::vector<std::string> LoadNames() {
    std::vector<std::string> names;
    std::string folder = "assets/intrinsic_meshes";

    for (auto const & entry : fs::directory_iterator(folder)) {
        if (!entry.is_regular_file()) continue;
        auto ext = entry.path().extension().string();
        if (ext != ".obj" && ext != ".ply") continue;
        names.push_back(entry.path().filename().string());
    }
    return names;
}

std::vector<Engine::SurfaceMesh> IntrinsicContent::Meshes = LoadMeshes();
std::vector<std::string> IntrinsicContent::Names  = LoadNames();

}
