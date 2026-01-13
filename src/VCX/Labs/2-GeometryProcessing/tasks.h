#pragma once

#include "Labs/2-GeometryProcessing/ModelObject.h"
#include "DCEL.hpp"
namespace VCX::Labs::GeometryProcessing {
    struct IntrinsicData {
            std::vector<double> edge_length;
        };
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations);
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations);
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio);
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight);
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n);
    
    IntrinsicData CalculateEdgeLength(DCEL const& G, Engine::SurfaceMesh const & extr_mesh)
    void DistanceMap(Engine::SurfaceMesh const& input,
                    DCEL const& G,
                    IntrinsicData const& intrinsic,
                    std::vector<int> const& sources);
    void DoDelaunayFlipping(DCEL& G,IntrinsicData& intrinsic_data);
}
