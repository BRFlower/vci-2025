#include <unordered_map>
#include <iostream>
#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();

                // your code here:
                //calculate average neighbors
                int n = neighbors.size();
                glm::vec3 E1(0.0f);
                for (auto v_neighbor : neighbors){
                    E1 += prev_mesh.Positions[v_neighbor];
                }
                float u = (n == 3) ? 3.0/16 : 3.0 / 8 / n;
                // glm::Vec3 E2(0.0f);
                // auto faces = v->Faces();
                // for (auto f : faces){
                //     E2 += prev_mesh.Positions[f->Center()];
                // }
                // from F' = 1/n [ F + 2 average(edges midpoint) + (n-3) average(face centerpoint use gravity center)],
                auto new_position = prev_mesh.Positions[i] * (-u * n + 1) + E1 * u;
                curr_mesh.Positions.push_back(new_position);
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                       = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    auto f1 = e->From(), f2 = e->To();
                    auto mid = (curr_mesh.Positions[f1] + curr_mesh.Positions[f2]) / 2.0f;
                    curr_mesh.Positions.push_back(mid);
                
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    auto f1 = e->From(), f2 = e->To();
                    auto mid = (curr_mesh.Positions[f1] + curr_mesh.Positions[f2]) / 2.0f;
                    curr_mesh.Positions.push_back(mid);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    {v0,m2,m1},
                    {v1,m0,m2},
                    {v2,m1,m0},
                    {m0,m1,m2}
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0.5,0.5 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        std::unordered_map<uint32_t, uint32_t> boundary_vertices; 
        uint32_t begin_point;
        for (auto edge_i : G.Edges()) {
            if (!(edge_i -> TwinEdgeOr(nullptr))) {
                //is boundary
                boundary_vertices[edge_i -> From()] = edge_i -> To();
                begin_point = edge_i -> From();
            }
        }
        // auto bp = begin_point;
        uint32_t s = boundary_vertices.size();
        // uint32_t begin_point = boundary_vertices.front()->first;
        
        const float pi = 3.14159265358979323846;
        for (int i = 0;i < s; i ++){
            output.TexCoords[begin_point] = glm::vec2(std::cos(i*2*pi/s) * 0.5 + 0.5, std::sin(i*2*pi/s) * 0.5 + 0.5);
            begin_point = boundary_vertices[begin_point];
        }
        // if (bp != begin_point){
        //     std::cout << "error" << std::endl;
        // }
        // Solve equation via Gauss-Seidel Iterative Method.
        float lambda = 1.0f;
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            for (int i = 0; i < input.Positions.size(); ++i)
                if (boundary_vertices.find(i) == boundary_vertices.end()){
                    glm::vec2 g = glm::vec2(0);
                    for (auto neighbor : G.Vertex(i) -> Neighbors()){
                        g +=  output.TexCoords[neighbor];
                    }
                    g /= G.Vertex(i) -> Neighbors().size();
                    output.TexCoords[i] = (1-lambda) * output.TexCoords[i] + lambda * g;
                }
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Kp matrix of the face f.
        auto UpdateQ {
            [&G, &output, &input] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Kp;
                // your code here:
                auto v0 = output.Positions[f -> VertexIndex(0)];
                auto v1 = output.Positions[f -> VertexIndex(1)];
                auto v2 = output.Positions[f -> VertexIndex(2)];

                // 计算两条边
                glm::vec3 e1 = v1 - v0;
                glm::vec3 e2 = v2 - v0;
                
                // 计算法向量
                glm::vec3 normal = glm::cross(e1, e2)/*/ float(input.Positions.size())*/;
                
                float d = -glm::dot(normal, v0);
                glm::vec4 n(normal, d);
                return glm::outerProduct(n, n);
            }
        };

        // The struct to record contraction info.
        struct ContractionPair {
            DCEL::HalfEdge const * edge;            // which edge to contract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ContractionPair struct.
        static constexpr auto MakePair {
            [] (DCEL::HalfEdge const * edge,
                glm::vec3 const & p1,
                glm::vec3 const & p2,
                glm::mat4 const & Q
            ) -> ContractionPair {
                // your code here:
                auto p1_ = glm::vec4(p1, 1.0f);
                auto p2_ = glm::vec4(p2, 1.0f);
                // glm::mat4 Q_prime = Q;
                // Q_prime[3] = glm::vec4(0, 0, 0, 1); // 最后一行改为[0 0 0 1]
                glm::mat4 Q_prime = glm::mat4{
                    Q[0][0], Q[1][0],Q[2][0], 0,
                    Q[0][1], Q[1][1],Q[2][1], 0,
                    Q[0][2], Q[1][2],Q[2][2], 0,
                    Q[0][3], Q[1][3],Q[2][3], 1
                };
                glm::vec4 rhs = glm::vec4(0, 0, 0, 1);
                // 求解线性方程组 Q' * v = rhs
                
                glm::vec4 vprime = glm::inverse(Q_prime) * rhs;

                // 如果矩阵不可逆，使用中点
                // if (glm::isnan(vprime.x) || glm::isnan(vprime.y) || glm::isnan(vprime.z)) {
                if (std::abs(glm::determinant(Q_prime) < 1e-3)){
                // if (true){
                    vprime = (p1_ + p2_) * 0.5f;
                    // compare p1_, p2_, vprime
                    auto d_p1 = glm::dot(p1_, Q * p1_), d_p2 = glm::dot(p2_, Q * p2_), d_mid = glm::dot(vprime, Q * vprime);
                    if (d_p1 < d_p2 && d_p1 < d_mid) {
                        vprime = p1_;
                    } else if (d_p2 < d_p1 && d_p2 < d_mid) {
                        vprime = p2_;
                    }
                                    //lambda p1 + (1-lambda) p2;
                    // sum dis^2 = a l^2 + 2 * b l + c
                    // auto a = glm::dot(p1_ - p2_, Q * (p1_ - p2_));
                    // auto b = glm::dot(p1_, Q * (p1_ - p2_)); // Q^T = Q always
                    // auto c = glm::dot(p2_, Q * p2_);
                    // auto lambda_best = 0.5f;
                    // if (abs(a) > 1e-6)
                    //     auto lambda_best = -b / a;
                    // vprime = lambda_best * p1_ + (1 - lambda_best) * p2_; //齐次坐标
                    // auto cost = c - b*b/a;
                }
                else
                    vprime = glm::inverse(Q_prime) * rhs;
                    // vprime /= vprime.w;
                float cost = glm::dot(vprime, Q * vprime);
                // if (vprime[3] == 0)
                //     std::cout << "error" << std::endl;
                // glm::vec4 p1_ = glm::vec4(p1, 1.0f);
                // glm::vec4 p2_ = glm::vec4(p2, 1.0f);


                
                return {edge, vprime, cost};
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ContractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Kf:       $Kf[idx]$ is the Kp matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map; 
        std::vector<ContractionPair>                  pairs; 
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Kf(G.NumOfFaces(),    glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q                 = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Kf[G.IndexOf(f)]       = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the contractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsContractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the contractable pair with minimal cost.

            // // 清理无效的pair
            // pairs.erase(
            //     std::remove_if(pairs.begin(), pairs.end(), 
            //         [](const ContractionPair& p) { return p.edge == nullptr; }), 
            //     pairs.end()
            // );

            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsContractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the contractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the contract result
            // ring:   the edge ring of vertex v1
            ContractionPair & top    = pairs[min_idx];
            auto               v1     = top.edge->From();
            auto               v2     = top.edge->To();
            auto               result = G.Contract(top.edge);
            auto               ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The contraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Kf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Kf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Kp matrix for $e->Face()$.
                //     2. According to the difference between the old Kp (in $Kf$) and the new Kp (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                //     4. Update $Kf$.
                auto new_kp = UpdateQ(e->Face());
                // for (int i = 0; i < 3; i ++)
                //     if (e->Face()->VertexIndex(i) != v1)
                //         Qv[e->Face()->VertexIndex(i)] += (new_kp - Kf[G.IndexOf(e->Face())]);
                Qv[e->From()] += (new_kp - Kf[G.IndexOf(e->Face())]);
                Qv[e->To()] += (new_kp - Kf[G.IndexOf(e->Face())]);
                Qv[v1] += new_kp;
                Kf[G.IndexOf(e->Face())] = new_kp;
            }

            // Finally, as the Q matrix changed, we should update the relative $ContractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:
            // 遍历 v1 的所有相邻节点
            //需要改写的是所有与v1相邻点有关的边

            for (auto e : ring) {
                // 更新与v1相关的边
                auto v = e -> From();
                // for (int i = 0; i < pairs.size(); i++){
                //     if (!pairs[i].edge) continue;
                //     auto vf = pairs[i].edge->From(), vt = pairs[i].edge->To();
                //     if (v == vf || v == vt) {
                //         pairs[i] = MakePair(pairs[i].edge, output.Positions[vf], output.Positions[vt], Qv[vf] + Qv[vt]);
                //     }
                // }
                for (auto e1 : G.Vertex(v)->Ring()){
                    auto e2 = e1->NextEdge();
                    if (!G.IsContractable(e2)){
                        pairs[pair_map[G.IndexOf(e2)]].edge = nullptr;
                    }
                    else {
                        auto v2 = e1->To();
                        auto pair = MakePair(e2, output.Positions[v], output.Positions[v2], Qv[v] + Qv[v2]);
                        pairs[pair_map[G.IndexOf(e2)]].targetPosition = pair.targetPosition;
                        pairs[pair_map[G.IndexOf(e2)]].cost = pair.cost;
                    }
                }
            }
        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                v1 -= vAngle;
                v2 -= vAngle;

                const float eps = 1e-6f;
                float l1 = glm::length(v1);
                float l2 = glm::length(v2);

                // if (l1 < eps || l2 < eps) {
                //     return 0.0f;
                // }

                // glm::vec3 u1 = v1 / l1;
                // glm::vec3 u2 = v2 / l2;

                float cosine = glm::dot(v1,v2);
                float sine = glm::sqrt(glm::dot(glm::cross(v1,v2),glm::cross(v1,v2)));
                if (sine < eps * glm::abs(cosine)){
                    return (cosine > 0)?1e2 : -1e2;
                }
                float cot = cosine / sine;
                return cot;
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            auto get_else = [&] (uint32_t a, uint32_t b, const DCEL::Triangle* face){
                if (face == nullptr)
                    return a;
                auto v1 = face -> VertexIndex(0),v2 = face->VertexIndex(1),v3=face->VertexIndex(2);
                if (v1 != a && v1 != b){
                    return v1;
                }
                else if (v2 != a && v2 != b){
                    return v2;
                }
                else if (v3 != a && v3 != b){
                    return v3;
                }
                else{
                    throw std::runtime_error("VCX::Labs::GeometryProcessing::SmoothMesh: Invalid triangle vertex configuration");
                }
            };
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
                
                float sum_w = 0;
                auto surround = glm::vec3(0);
                for (auto m : G.Vertex(i)->Ring()){
                    auto v = m -> To();
                    auto f = m -> Face();
                    auto angle_point = get_else(i,v,f);
                    float cot;
                    if (useUniformWeight){
                        cot = 1;
                        // sum_w += 1;
                    }
                    else{
                        auto x1 = prev_mesh.Positions[angle_point]-prev_mesh.Positions[i], x2 = prev_mesh.Positions[angle_point] - prev_mesh.Positions[v];
                        // sum_w += glm::sqrt(glm::dot(glm::cross(x1,x2),glm::cross(x1,x2))) / 3;
                        cot = -GetCotangent(prev_mesh.Positions[angle_point],prev_mesh.Positions[v],prev_mesh.Positions[i]);
                        // auto twin = m->TwinEdgeOr(nullptr);
                        f = m->OppositeFace();
                        if (f != nullptr) {
                            angle_point = get_else(i,v,f);
                            cot += -GetCotangent(prev_mesh.Positions[angle_point],prev_mesh.Positions[v],prev_mesh.Positions[i]);
                            // cot /= 2;
                        }
                        if (cot < 0)
                            cot = -cot;
                        // sum_w += cot;
                    }
                    sum_w += cot;
                    surround += cot * (prev_mesh.Positions[v]);
                }
                if (glm::abs(sum_w) < 1e-2f)
                    curr_mesh.Positions[i] = prev_mesh.Positions[i];
                else{
                    surround /= sum_w;
                    curr_mesh.Positions[i] = prev_mesh.Positions[i] * (1-lambda) + surround * lambda; 
                }
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here:
        output.Positions.clear();
        output.Indices.clear();

        struct ArrayHash {
            std::size_t operator()(const std::array<int, 3>& arr) const {
                std::size_t h1 = std::hash<int>{}(arr[0]);
                std::size_t h2 = std::hash<int>{}(arr[1]);
                std::size_t h3 = std::hash<int>{}(arr[2]);
                return h1 ^ (h2 << 1) ^ (h3 << 2);
            }
        };
        std::unordered_map<std::array<int,3>, uint32_t, ArrayHash> pos_to_idx;
        uint32_t cnt = 0;
        auto get_vertex_index = [&sdf, &pos_to_idx, &cnt, &output, &dx, &grid_min] (glm::vec3 pos){
            glm::vec3 grid_pos = (pos - grid_min) / dx * 2.0f;
            std::array<int, 3> pos_i = {
                static_cast<int>(std::round(grid_pos[0])),
                static_cast<int>(std::round(grid_pos[1])),
                static_cast<int>(std::round(grid_pos[2]))
            };
            auto it = pos_to_idx.find(pos_i);
            if (it != pos_to_idx.end())
                return pos_to_idx[pos_i];
            else{
                pos_to_idx[pos_i] = cnt;
                //插值
                glm::vec3 p1, p2;
                if (pos_i[0] % 2 == 1){
                    p1 = pos - glm::vec3(dx * 0.5f, 0, 0);
                    p2 = pos + glm::vec3(dx * 0.5f, 0, 0);
                }
                else if (pos_i[1] % 2 == 1){
                    p1 = pos - glm::vec3(0, dx * 0.5f, 0);
                    p2 = pos + glm::vec3(0, dx * 0.5f, 0);
                }
                else{
                    p1 = pos - glm::vec3(0, 0, dx * 0.5f);
                    p2 = pos + glm::vec3(0, 0, dx * 0.5f);
                }
                float x1 = sdf(p1);
                float x2 = sdf(p2);
                if (glm::abs(x1-x2) > 1e-6 * x1){
                    // (1-lambda) x1 + lambda x2 = 0
                    auto lambda = x1 / (x1 - x2);
                    pos = (1-lambda) * p1 + lambda * p2;
                }
                output.Positions.push_back(pos);
                return cnt ++;
            }
        };
        auto do_marching = [&dx, &get_vertex_index, &output] (glm::vec3 start_point, uint32_t ref){
            // auto calculate_position = [&dx, &start_point](uint32_t id_e){
            //     glm::vec3 cal;
            //     switch(id_e){
            //         case 0:
            //             cal = glm::vec3(1,0,0);
            //             break;
            //         case 1:
            //             cal = glm::vec3(1,2,0);
            //             break;
            //         case 2:
            //             cal = glm::vec3(1,0,2);
            //             break;
            //         case 3:
            //             cal = glm::vec3(1,2,2);
            //             break;
            //         case 4:
            //             cal = glm::vec3(0,1,0);
            //             break;
            //         case 5:
            //             cal = glm::vec3(0,1,2);
            //             break;
            //         case 6:
            //             cal = glm::vec3(2,1,0);
            //             break;
            //         case 7:
            //             cal = glm::vec3(2,1,2);
            //             break;
            //         case 8:
            //             cal = glm::vec3(0,0,1);
            //             break;
            //         case 9:
            //             cal = glm::vec3(2,0,1);
            //             break;
            //         case 10:
            //             cal = glm::vec3(0,2,1);
            //             break;
            //         case 11:
            //             cal = glm::vec3(2,2,1);
            //             break;
            //         default:
            //             spdlog::warn("Error in MarchingCubes.position()");
            //             cal = glm::vec3(0,0,0);
            //             break;
            //     };
            //     return start_point + 0.5f * dx * cal;
            // };
            
            auto calculate_position = [&dx, &start_point](uint32_t edge_index){
                // 起始点: v0 + dx * (j & 1) * unit(((j >> 2) + 1) % 3) + dx * ((j >> 1) & 1) * unit(((j >> 2) + 2) % 3)
                // 方向: unit(j >> 2)
                
                glm::vec3 start(0.0f);
                glm::vec3 direction(0.0f);
                
                // 计算起始点
                int j = edge_index;
                glm::vec3 v0(0.0f); // 立方体的v0顶点
                
                auto unit = [](int i) -> glm::vec3 {
                    switch(i) {
                        case 0: return glm::vec3(1, 0, 0);
                        case 1: return glm::vec3(0, 1, 0);
                        case 2: return glm::vec3(0, 0, 1);
                        default: return glm::vec3(0, 0, 0);
                    }
                };
                
                start = v0 + 
                    dx * (j & 1) * unit(((j >> 2) + 1) % 3) + 
                    dx * ((j >> 1) & 1) * unit(((j >> 2) + 2) % 3);
                
                // 计算方向
                direction = unit(j >> 2);
                
                glm::vec3 midpoint = start + 0.5f * direction * dx;
                
                return start_point + midpoint;
            };
            auto triangle_table = c_EdgeOrdsTable[ref];
            for (int ti = 0; ti < 5; ti++) {
                int e0 = triangle_table[3 * ti + 0];
                int e1 = triangle_table[3 * ti + 1];
                int e2 = triangle_table[3 * ti + 2];
                
                if (e0 == -1) break;
                output.Indices.push_back(get_vertex_index(calculate_position(e0)));
                output.Indices.push_back(get_vertex_index(calculate_position(e1)));
                output.Indices.push_back(get_vertex_index(calculate_position(e2)));
            }
        };
        for (int i = 0; i < n-1; i ++){
            for (int j = 0; j < n-1; j ++){
                for (int k = 0; k < n-1; k ++){
                    glm::vec3 start = grid_min + glm::vec3(i * dx, j * dx, k * dx);
                    uint32_t outside = 0;
                    for (int u = 0; u < 8; u ++)
                        outside |= (sdf(start + dx * glm::vec3(u&1, (u>>1)&1, (u>>2)&1)) < 0) << u;
                    if (outside == 0 || outside == 255) continue;
                    do_marching(start, outside);
                }
            }
        }
    }
} // namespace VCX::Labs::GeometryProcessing
