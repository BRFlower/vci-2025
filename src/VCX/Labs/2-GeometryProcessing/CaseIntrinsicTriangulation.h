#pragma once

#include "Engine/Async.hpp"
#include "Labs/2-GeometryProcessing/Content.h"
#include "Labs/2-GeometryProcessing/Viewer.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::GeometryProcessing {

class CaseIntrinsicTriangulation : public Common::ICase {
public:
    CaseIntrinsicTriangulation(
        Viewer & viewer,
        std::initializer_list<Assets::ExampleModel> && models
    );

    virtual std::string_view const GetName() override {
        return "Intrinsic Triangulation";
    }

    virtual void OnSetupPropsUI() override;
    virtual Common::CaseRenderResult OnRender(
        std::pair<std::uint32_t, std::uint32_t> const desiredSize
    ) override;
    virtual void OnProcessInput(ImVec2 const & pos) override;

private:
    // ===== Models =====
    std::vector<Assets::ExampleModel> const _models;
    std::size_t _modelIdx { 0 };

    // ===== Compute state =====
    Engine::Async<Engine::SurfaceMesh> _task;
    bool _recompute { true };
    bool _running   { false };

    // ===== Viewer / Camera =====
    Viewer & _viewer;
    Engine::Camera _camera { .Eye = glm::vec3(-1, 1, 1) };
    Common::OrbitCameraManager _cameraManager { glm::vec3(-1, 1, 1) };

    // ===== Render =====
    ModelObject   _modelObject;
    RenderOptions _options;

    // ===== UI Parameters =====
    bool _useFlipping     { true };
    bool _showInitialMesh { false };   // 👈 你要的调试按钮

private:
    char const * GetModelName(std::size_t const i) const {
        return Content::ModelNames[std::size_t(_models[i])].c_str();
    }

    Engine::SurfaceMesh const & GetModelMesh(std::size_t const i) const {
        return Content::ModelMeshes[std::size_t(_models[i])];
    }
};

} // namespace VCX::Labs::GeometryProcessing
