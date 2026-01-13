#include <algorithm>
#include <iostream>

#include "Labs/2-GeometryProcessing/tasks.h"
#include "Labs/2-GeometryProcessing/CaseIntrinsicTriangulation.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp" 
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::GeometryProcessing {

CaseIntrinsicTriangulation::CaseIntrinsicTriangulation(
    Viewer& viewer,
    const std::vector<IntrinsicContent::MeshItem>& models
)
    : _viewer(viewer)
    , _models(models)
    , _cameraManager(glm::vec3(-1, 1, 1))
{
    // Camera
    _camera.Eye = glm::vec3(-1, 1, 1);
    _cameraManager.EnablePan       = false;
    _cameraManager.AutoRotateSpeed = 0.f;

    // Geometry-friendly default rendering
    _options.Ambient = 0.3f;     
    _options.Flat = true;

    _options.LightDirection =
        glm::vec3(
            glm::cos(glm::radians(_options.LightDirScalar)),
            -1.0f,
            glm::sin(glm::radians(_options.LightDirScalar))
        );
}

// ============================================================
// UI
// ============================================================

void CaseIntrinsicTriangulation::OnSetupPropsUI() {
    Common::ImGuiHelper::SaveImage(_viewer.GetTexture(), _viewer.GetSize(), true);
    ImGui::Spacing();

    // ---- Model ----
    if (ImGui::BeginCombo("Model", GetModelName(_modelIdx))) {
        for (std::size_t i = 0; i < _models.size(); ++i) {
            bool selected = (i == _modelIdx);
            if (ImGui::Selectable(GetModelName(i), selected)) {
                _modelIdx  = i;
                _recompute = true;
            }
            if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::Separator();

    // ---- Display mode ----
    if (ImGui::Checkbox("Mesh Only (Wireframe)", &_showInitialMesh)) {
        _showUV = false;
        _recompute = true;
    }

    if (ImGui::Checkbox("Show Distance (UV)", &_showUV)) {
        _showInitialMesh = false;
        _recompute = true;
    }

    // Intrinsic Delaunay 开关
    ImGui::Separator();
    if (ImGui::Checkbox("Enable Intrinsic Delaunay", &_useFlipping)) {
        _recompute = true;
    }

    ImGui::Separator();

    if (ImGui::Button("Recompute"))
        _recompute = true;

    if (_running) {
        static const std::string t = "Running.....";
        ImGui::Text(
            t.substr(0, 7 + (static_cast<int>(ImGui::GetTime() / 0.1f) % 6)).c_str()
        );
    } else {
        ImGui::NewLine();
    }

    ImGui::Spacing();
    Viewer::SetupRenderOptionsUI(_options, _cameraManager);
}

// ============================================================
// Render + Compute
// ============================================================

Common::CaseRenderResult
CaseIntrinsicTriangulation::OnRender(
    std::pair<std::uint32_t, std::uint32_t> const desiredSize
) {
    if (_showInitialMesh) {
        _options.Wireframe = true;
    } else {
        _options.Wireframe = false;
    }

    if (_recompute) {
        _recompute = false;
        _running   = true;

        std::size_t modelIdx = _modelIdx;
        bool showMesh = _showInitialMesh;
        bool showUV   = _showUV;
        bool useFlipping = _useFlipping;

        _task.Emplace([&, modelIdx, showMesh, showUV, useFlipping]() {
            std::cout << "[TASK] Start\n";

            // 1. 获取原始网格
            Engine::SurfaceMesh inputMesh = GetModelMesh(modelIdx);
            Engine::SurfaceMesh output = inputMesh; 

            // 2. 构建原始数据的 DCEL 和 IntrinsicData
            DCEL G_orig(inputMesh);
            IntrinsicData data_orig = CalculateEdgeLength(G_orig, inputMesh);

            // 3. 准备用于计算的引用 (默认指向原始数据)
            // 使用指针或引用来指向当前应该使用的数据集
            DCEL* activeG = &G_orig;
            IntrinsicData* activeData = &data_orig;

            // 如果需要翻转，我们创建一份副本进行操作
            // 注意：这里需要 DCEL 和 IntrinsicData 支持拷贝构造函数 (默认的通常就够用)
            DCEL G_flipped(inputMesh); 
            IntrinsicData data_flipped = data_orig;

            if (useFlipping) {
                std::cout << "Computing Intrinsic Delaunay Triangulation...\n";
                
                // 对副本进行翻转操作
                DoDelaunayFlipping(G_flipped, data_flipped);

                // 更新输出网格的拓扑结构
                output = G_flipped.ExportMesh();
                output.Positions = inputMesh.Positions; 
                output.TexCoords = inputMesh.TexCoords;

                // 将活跃指针指向翻转后的数据
                activeG = &G_flipped;
                activeData = &data_flipped;
            }

            // 4. 计算距离场
            if (showUV) {
                std::cout << "Computing Distance Map...\n";
                std::vector<int> sources { 0 };
                
                // 传入当前活跃的数据 (可能是原始的，也可能是翻转后的)
                DistanceMap(output, *activeG, *activeData, sources);
            }

            std::cout << "Computation done.\n";
            return output;
        });
    }

    if (_running && _task.HasValue()) {
        _running = false;
        _modelObject.ReplaceMesh(_task.Value());
    }

    return _viewer.Render(
        _options,
        _modelObject,
        _camera,
        _cameraManager,
        desiredSize
    );
}

// ============================================================
// Input
// ============================================================

void CaseIntrinsicTriangulation::OnProcessInput(ImVec2 const& pos) {
    _cameraManager.ProcessInput(_camera, pos);
}

} // namespace VCX::Labs::GeometryProcessing