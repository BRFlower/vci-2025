// final project
#include <algorithm>
#include <array>

#include "Labs/2-GeometryProcessing/tasks.h"
#include "Labs/2-GeometryProcessing/CaseIntrinsicTriangulation.h"
#include "Labs/Common/ImGuiHelper.h"
namespace VCX::Labs::GeometryProcessing {

CaseIntrinsicTriangulation::CaseIntrinsicTriangulation(
    Viewer & viewer,
    std::initializer_list<Assets::ExampleModel> && models
)
    : _models(models)
    , _viewer(viewer) {}

void CaseIntrinsicTriangulation::OnSetupPropsUI() {
    // ---- Model selection ----
    if (ImGui::BeginCombo("Model", GetModelName(_modelIdx))) {
        for (std::size_t i = 0; i < _models.size(); ++i) {
            bool selected = (i == _modelIdx);
            if (ImGui::Selectable(GetModelName(i), selected)) {
                _modelIdx = i;
                _recompute = true;
            }
            if (selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    // ---- Intrinsic options ----
    if (ImGui::Checkbox("Use Delaunay Flipping", &_useFlipping)) {
        _recompute = true;
    }

    // ---- Debug: show original mesh ----
    ImGui::Checkbox("Show Initial Mesh", &_showInitialMesh);

    // ---- Recompute ----
    if (ImGui::Button("Recompute")) {
        _recompute = true;
    }
}

Common::CaseRenderResult
CaseIntrinsicTriangulation::OnRender(
    std::pair<std::uint32_t, std::uint32_t> const desiredSize
) {
    if (_recompute) {
        _recompute = false;
        _running = true;

        _task.Emplace([&]() {
            // ⚠️ 现在只是占位：返回原 mesh
            // 之后你会在这里：
            // 1. 复制 extrinsic mesh
            // 2. 构建 intrinsic triangulation
            // 3. flipping / split / etc.
            return GetModelMesh(_modelIdx);
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


void CaseIntrinsicTriangulation::OnProcessInput(ImVec2 const & pos) {
    _cameraManager.ProcessInput(_camera, pos);
}

} // namespace VCX::Labs::GeometryProcessing
