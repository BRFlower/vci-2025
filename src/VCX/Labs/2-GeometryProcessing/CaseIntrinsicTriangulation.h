#pragma once

#include "Labs/Common/ICase.h"
#include "Labs/2-GeometryProcessing/IntrinsicContent.h"
#include "Labs/2-GeometryProcessing/Viewer.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Engine/Async.hpp"

namespace VCX::Labs::GeometryProcessing {

class CaseIntrinsicTriangulation : public Common::ICase {
public:
    CaseIntrinsicTriangulation(
        Viewer& viewer,
        std::vector<IntrinsicContent::MeshItem> const& models
    );

    std::string_view const GetName() override {
        return "Intrinsic Triangulation";
    }

    void OnSetupPropsUI() override;
    Common::CaseRenderResult OnRender(
        std::pair<std::uint32_t, std::uint32_t> const desiredSize
    ) override;
    void OnProcessInput(ImVec2 const& pos) override;

private:
    Viewer& _viewer;

    std::vector<IntrinsicContent::MeshItem> const& _models;
    std::size_t _modelIdx { 0 };

    Engine::Async<Engine::SurfaceMesh> _task;
    bool _recompute { true };
    bool _running   { false };

    bool _useFlipping     { false };
    bool _showInitialMesh { true };   // Mesh Only
    bool _showUV          { false };  // Distance / UV view

    Engine::Camera _camera;
    Common::OrbitCameraManager _cameraManager;

    ModelObject   _modelObject;
    RenderOptions _options;

    char const* GetModelName(std::size_t i) const {
        return _models[i].name.c_str();
    }

    Engine::SurfaceMesh const& GetModelMesh(std::size_t i) const {
        return _models[i].mesh;
    }
};

} // namespace VCX::Labs::GeometryProcessing
