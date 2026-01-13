#include <algorithm>
#include <iostream>

#include "Labs/2-GeometryProcessing/tasks.h"
#include "Labs/2-GeometryProcessing/CaseIntrinsicTriangulation.h"
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
    _options.Ambient = 0.3f;     // 强环境光，避免阴影过重
    // _options.Wireframe = true;        // Mesh Only 时会开启
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
    // Render options depend on mode
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

        _task.Emplace([&, modelIdx, showMesh, showUV]() {
            std::cout << "[TASK] Start\n";

            Engine::SurfaceMesh output = GetModelMesh(modelIdx);

            if (showUV) {
                // Use intrinsic distance -> TexCoords
                std::vector<int> sources { 0 };
                DistanceMap(GetModelMesh(modelIdx), output, sources);
            }

            std::cout << "Distance computation done.\n";
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
    // 不拦截，保证滚轮缩放正常
    _cameraManager.ProcessInput(_camera, pos);
}

} // namespace VCX::Labs::GeometryProcessing
