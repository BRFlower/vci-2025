#pragma once
#include <iostream>
#include <vector>
#include <unordered_map>
#include <set>
#include "Engine/SurfaceMesh.h"

namespace VCX::Labs::GeometryProcessing {
    struct DCEL {
    public:
        using VertexIdx = std::uint32_t;
        using FaceIdx   = std::uint32_t;
        using EdgeIdx   = std::uint32_t;
        using Label     = std::int8_t;

        struct Triangle;

        struct HalfEdge {
            VertexIdx To() const & { return _to; }
            VertexIdx From() const & { return this[_prev]._to; }

            HalfEdge const * NextEdge() const & { return this + _next; }
            HalfEdge const * PrevEdge() const & { return this + _prev; }
            HalfEdge const * TwinEdgeOr(HalfEdge const * defaultValue) const & { return _twin ? (this + _twin) : defaultValue; }
            HalfEdge const * TwinEdge() const & { return this + _twin; }

            VertexIdx        OppositeVertex() const & { return this[_next]._to; }
            Triangle const * Face() const & { return reinterpret_cast<Triangle const *>(this - _idx); }
            Triangle const * OppositeFace() const & { return this[_twin].Face(); }
            VertexIdx        TwinOppositeVertex() const & { return this[_twin].OppositeVertex(); }

            bool  CountOnce() const & { return _twin < 0; }
            Label EdgeLabel() const & { return _idx; }

            HalfEdge()                 = default;
            HalfEdge(HalfEdge &&)      = default;
            HalfEdge(HalfEdge const &) = delete;
            HalfEdge & operator=(HalfEdge const &) = delete;

            friend struct DCEL;

        private:
            VertexIdx   _to;
            int         _twin;
            std::int8_t _prev;
            std::int8_t _next;
            std::int8_t _idx;
            std::int8_t _padding;
        };

        struct Triangle {
        public:
            HalfEdge const * Edge(Label i) const { return _e + i; }
            VertexIdx        VertexIndex(Label i) const { return _e[i].NextEdge()->_to; }

            Triangle const * OppositeFace(Label i) const { return _e[i].OppositeFace(); }
            VertexIdx        OppositeVertex(Label i) const { return _e[i].TwinOppositeVertex(); }
            bool             HasOppositeFace(Label i) const { return _e[i].TwinEdgeOr(nullptr); }

            Label LabelOfVertex(VertexIdx idx) const {
                if (_e[1]._to == idx) return 0;
                if (_e[2]._to == idx) return 1;
                if (_e[0]._to == idx) return 2;
                return -1;
            }

            bool HasVertex(VertexIdx idx) const {
                return _e[1]._to == idx || _e[2]._to == idx || _e[0]._to == idx;
            }

            Triangle()                 = default;
            Triangle(Triangle &&)      = default;
            Triangle(Triangle const &) = delete;
            Triangle & operator=(Triangle const &) = delete;

            friend struct DCEL;

        private:
            HalfEdge _e[3];
        };

        static_assert(sizeof(HalfEdge) == 12U);
        static_assert(sizeof(Triangle) == 36U);

        DCEL(Engine::SurfaceMesh const & mesh) :
            _vcnt(mesh.Positions.size()),
            _fcnt(mesh.Indices.size() / 3) {
            _verts.reserve(_vcnt);
            AddFaces(mesh.Indices);
            _vert_masks.resize(_verts.size());
            _face_masks.resize(_faces.size());
        }

        bool IsManifold() const { return _manifold; }

        bool IsWatertight() const { return _watertight; }

        struct VertexProxy {
        public:
            std::pair<VertexIdx, VertexIdx> BoundaryNeighbors() const & {
                HalfEdge const * _e = reinterpret_cast<HalfEdge const *>(this);
                HalfEdge const * e  = _e;
                while (true) {
                    e                      = e->PrevEdge();
                    HalfEdge const * trial = e->TwinEdgeOr(nullptr);
                    if (! trial) break;
                    e = trial;
                };
                return { _e->To(), e->From() };
            }

            std::vector<VertexIdx> Neighbors() const & {
                std::vector<VertexIdx> neighbors;
                HalfEdge const *       _e = reinterpret_cast<HalfEdge const *>(this);
                HalfEdge const *       e  = _e;
                do {
                    neighbors.emplace_back(e->To());
                    e                      = e->PrevEdge();
                    HalfEdge const * trial = e->TwinEdgeOr(nullptr);
                    if (! trial) {
                        neighbors.emplace_back(e->From());
                        break;
                    }
                    e = trial;
                } while (e != _e);
                return neighbors;
            }

            std::vector<HalfEdge const *> Ring() const & {
                std::vector<HalfEdge const *> ring;
                HalfEdge const *              _e = reinterpret_cast<HalfEdge const *>(this);
                HalfEdge const *              e  = _e;
                do {
                    ring.emplace_back(e->NextEdge());
                    e                      = e->PrevEdge();
                    HalfEdge const * trial = e->TwinEdgeOr(nullptr);
                    if (! trial) {
                        ring.emplace_back(e->NextEdge());
                        break;
                    }
                    e = trial;
                } while (e != _e);
                return ring;
            }

            std::vector<Triangle const *> Faces() const & {
                std::vector<Triangle const *> faces;
                HalfEdge const *              _e = reinterpret_cast<HalfEdge const *>(this);
                HalfEdge const *              e  = _e;
                do {
                    faces.emplace_back(e->Face());
                    e = e->PrevEdge()->TwinEdgeOr(nullptr);
                    if (! e) break;
                } while (e != _e);
                return faces;
            }

            bool OnBoundary() const & {
                return ! reinterpret_cast<HalfEdge const *>(this)->TwinEdgeOr(nullptr);
            }

            VertexProxy() = delete;
        };

        VertexProxy const * Vertex(VertexIdx idx) const {
            if (_vert_masks[idx]) return nullptr;
            return reinterpret_cast<VertexProxy const *>(
                reinterpret_cast<HalfEdge const *>(_faces.data()) + _verts[idx]
            );
        }

        Triangle const * Face(FaceIdx idx) const {
            if (_face_masks[idx]) return nullptr;
            return _faces.data() + idx;
        }

        HalfEdge const * Edge(EdgeIdx idx) const {
            if (_face_masks[idx / (sizeof(Triangle) / sizeof(HalfEdge))]) return nullptr;
            return reinterpret_cast<HalfEdge const *>(_faces.data()) + idx;
        }

        std::vector<Triangle const *> Faces() const {
            std::vector<Triangle const *> faces;
            for (size_t i = 0; i < _faces.size(); ++i) {
                if (! _face_masks[i]) faces.emplace_back(_faces.data() + i);
            }
            return faces;
        }

        std::vector<HalfEdge const *> Edges() const {
            std::vector<HalfEdge const *> results;
            for (std::size_t i = 0; i < _faces.size(); ++i) {
                if (_face_masks[i]) continue;
                for (std::size_t j = 0; j < 3; ++j) {
                    HalfEdge const * e = _faces[i].Edge(j);
                    if (! e->TwinEdgeOr(nullptr) || e->CountOnce()) results.emplace_back(e);
                }
            }
            return results;
        }

        FaceIdx IndexOf(Triangle const * face) const {
            return static_cast<FaceIdx>(face - _faces.data());
        }

        EdgeIdx IndexOf(HalfEdge const * edge) const {
            return static_cast<EdgeIdx>(edge - reinterpret_cast<HalfEdge const *>(_faces.data()));
        }

        bool IsVertexRemoved(VertexIdx idx) const {
            return _vert_masks[idx];
        }

        bool IsFaceRemoved(FaceIdx idx) const {
            return _face_masks[idx];
        }

        bool IsFaceRemoved(Triangle const * face) const {
            return IsFaceRemoved(IndexOf(face));
        }

        std::size_t NumOfVertices() const {
            return _vcnt;
        }
        
        std::size_t NumOfFaces() const {
            return _fcnt;
        }

        Engine::SurfaceMesh ExportMesh() const {
            Engine::SurfaceMesh mesh;
            mesh.Indices.reserve(3 * _fcnt);
            for (auto f : Faces()) {
                mesh.Indices.emplace_back(f->VertexIndex(0));
                mesh.Indices.emplace_back(f->VertexIndex(1));
                mesh.Indices.emplace_back(f->VertexIndex(2));
            }
            return mesh;
        }

        bool DebugWatertightManifold() const {
            auto current = ExportMesh();
            DCEL dummy { current };
            return dummy.IsManifold() && dummy.IsWatertight();
        }

        // Function: DebugEdge
        // ?   x   ?   ?  x  ?
        //  \ / \ /     \ | /
        //   a - b  ==>   a
        //  / \ / \     / | \ 
        // ?   y   ?   ?  y  ?
        //      
        // params:
        //     edge = a->b
        // print:
        //     wing faces: face_abx, face_bay
        //     wing vertices: x, y
        //     ring of vertices: Vertex(x)->Ring(), Vertex(y)->Ring()
        //
        void DebugEdge(HalfEdge const * edge, std::ostream & os = std::cout) const {
            os << "DEBUG edge " << edge->From() << "->" << edge->To() << std::endl;
            os << "  wing faces: " << std::endl;
            {
                auto f = edge->Face();
                os << "    face " << f->VertexIndex(0) << "," << f->VertexIndex(1) << "," << f->VertexIndex(2) << std::endl;
            }
            {
                auto f = edge->OppositeFace();
                os << "    face " << f->VertexIndex(0) << "," << f->VertexIndex(1) << "," << f->VertexIndex(2) << std::endl;
            }
            os << "  wing vertices: " << edge->OppositeVertex() << "," << edge->TwinOppositeVertex() << std::endl;
            os << "  ring of vertices:" << std::endl;
            {
                os << "    ring of " << edge->From() << ":" << std::endl;
                for (auto e : Vertex(edge->From())->Ring()) {
                    os << "      " << e->From() << "->" << e->To() << std::endl;
                }
            }
            {
                os << "    ring of " << edge->To() << ":" << std::endl;
                for (auto e : Vertex(edge->To())->Ring()) {
                    os << "      " << e->From() << "->" << e->To() << std::endl;
                }
            }
        }

        bool IsContractable(HalfEdge const * edge) const {
            if (! _watertight || ! _manifold)
                return false;
            std::array<VertexIdx, 2> wing_vertices = { edge->OppositeVertex(), edge->TwinOppositeVertex() };
            std::set<EdgeIdx>        conflict_edges;
            std::set<VertexIdx>      conflict_vertices;
            for (auto p_opposite_edge : Vertex(edge->From())->Ring()) {
                conflict_edges.insert(IndexOf(p_opposite_edge));
                conflict_vertices.insert(p_opposite_edge->To());
                // std::cout << "p " << p_opposite_edge->From() << "->" << p_opposite_edge->To() << std::endl;
            }
            for (auto q_opposite_edge : Vertex(edge->To())->Ring()) {
                // std::cout << "q " << q_opposite_edge->From() << "->" << q_opposite_edge->To() << std::endl;
                if (conflict_vertices.count(q_opposite_edge->To()) &&
                    q_opposite_edge->To() != wing_vertices[0] &&
                    q_opposite_edge->To() != wing_vertices[1])
                    return false;
                if (conflict_edges.count(IndexOf(q_opposite_edge)))
                    return false;
                if (conflict_edges.count(IndexOf(q_opposite_edge->TwinEdge())))
                    return false;
            }
            // std::cout << 'T' << std::endl;
            return true;
        }

        struct ContractionResult {
            std::array<std::pair<HalfEdge const *, HalfEdge const *>, 2> collapsed_edges; // [(ax, xb), (by, ya)]
            std::array<std::pair<HalfEdge const *, HalfEdge const *>, 2> removed_edges;   // [(xa, bx), (yb, ay)]
            std::array<std::pair<VertexIdx, Triangle const *>, 2>        removed_faces;   // [(x, abx), (y, bay)]
        };

        // Function: Contract
        // ?   x   ?   ?  x  ?
        //  \ / \ /     \ | /
        //   a - b  ==>   a
        //  / \ / \     / | \ 
        // ?   y   ?   ?  y  ?
        //      
        // params:
        //     edge = a->b
        // return:
        //     .collapsed_edges = [(a->x,  x->b), (b->y,  y->a)]
        //     .removed_edges   = [(x->a,  b->x), (y->b,  a->y)]
        //     .removed_faces   = [(x, face_abx), (y, face_bay)]
        //
        ContractionResult Contract(HalfEdge const * edge) {
            HalfEdge const *   ab     = edge;
            HalfEdge const *   ba     = ab->TwinEdge();
            HalfEdge *         ax     = const_cast<HalfEdge *>(ab->PrevEdge()->TwinEdge());
            HalfEdge *         by     = const_cast<HalfEdge *>(ba->PrevEdge()->TwinEdge());
            VertexIdx          a      = ab->From();
            VertexIdx          b      = ab->To();
            VertexIdx          x      = ax->To();
            VertexIdx          y      = by->To();
            HalfEdge *         ya     = const_cast<HalfEdge *>(ba->NextEdge()->TwinEdge());
            HalfEdge *         xb     = const_cast<HalfEdge *>(ab->NextEdge()->TwinEdge());
            ContractionResult result = {
                .collapsed_edges = {
                    std::pair<HalfEdge const *, HalfEdge const *> { ax, xb },
                    std::pair<HalfEdge const *, HalfEdge const *> { by, ya }
                },
                .removed_edges = {
                    std::pair<HalfEdge const *, HalfEdge const *> { ab->PrevEdge(), ab->NextEdge() },
                    std::pair<HalfEdge const *, HalfEdge const *> { ba->PrevEdge(), ba->NextEdge() }
                },
                .removed_faces = {
                    std::pair<VertexIdx, Triangle const *> { x, ab->Face() },
                    std::pair<VertexIdx, Triangle const *> { y, ba->Face() }
                }
            };

            for (auto e : Vertex(b)->Ring())
                if (e->To() != a && e->To() != y) {
                    const_cast<HalfEdge *>(e->NextEdge())->_to = a;
                }

            ax->_twin = static_cast<int>(xb - ax);
            xb->_twin = -ax->_twin;
            ya->_twin = static_cast<int>(by - ya);
            by->_twin = -ya->_twin;
            _verts[a] = IndexOf(ax);
            _verts[x] = IndexOf(xb);
            _verts[y] = IndexOf(ya);

            _face_masks[IndexOf(ab->Face())] = true;
            _face_masks[IndexOf(ba->Face())] = true;
            _vert_masks[b]                   = true;
            _vcnt                            = _vcnt - 1;
            _fcnt                            = _fcnt - 2;

            return result;
        }

    private:
        std::vector<EdgeIdx>                 _verts;
        std::vector<Triangle>                _faces;
        std::vector<bool>                    _vert_masks;
        std::vector<bool>                    _face_masks;
        bool                                 _manifold   = true;
        bool                                 _watertight = true;
        std::size_t                          _vcnt       = 0;
        std::size_t                          _fcnt       = 0;

        int GetTwin(std::unordered_map<std::size_t, int> & pairs, VertexIdx vFrom, VertexIdx vTo, int e) {
            std::size_t key  = static_cast<std::size_t>(std::min(vFrom, vTo)) << 32ULL | static_cast<std::size_t>(std::max(vFrom, vTo));
            auto        iter = pairs.find(key);
            if (iter == pairs.end()) {
                pairs[key] = e;
                return 0;
            }
            if (~iter->second) {
                HalfEdge & he = reinterpret_cast<HalfEdge *>(_faces.data())[iter->second];
                he._twin      = e - iter->second;
                _manifold     = _manifold && he._to == vFrom;
                iter->second  = -1;
                return -he._twin;
            }
            _manifold = false;
            return 0;
        }

        void AddFaces(std::vector<std::uint32_t> const & faces) {
            std::unordered_map<std::size_t, int> pairs;
            pairs.reserve(faces.size() / 2);
            _faces.reserve(_fcnt);
            for (std::size_t i = 0; i < faces.size(); i += 3U) {
                AddFaceImpl(pairs, faces[i + 0U], faces[i + 1U], faces[i + 2U]);
            }
            if (! _manifold) return;

            std::vector<int8_t> boundaryCount(_verts.size());
            for (std::size_t t = 0; t < _faces.size(); ++t) {
                for (std::size_t i = 0; i < 3; ++i) {
                    if (! _faces[t].HasOppositeFace(i)) {
                        _watertight = false;
                        VertexIdx j = _faces[t]._e[(i + 2) % 3]._to;
                        VertexIdx k = _faces[t]._e[i % 3]._to;
                        if (++boundaryCount[j] > 2 || ++boundaryCount[k] > 2) {
                            _manifold = false;
                            return;
                        }
                        _verts[j] = static_cast<EdgeIdx>(t * (sizeof(Triangle) / sizeof(HalfEdge)) + i);
                    }
                }
            }
            for (std::size_t vid = 0; vid < _verts.size(); ++vid) {
                if (boundaryCount[vid] == 1) {
                    _manifold = false;
                    return;
                }
            }
        }

        Triangle & AddFaceImpl(std::unordered_map<std::size_t, int> & pairs, VertexIdx v0, VertexIdx v1, VertexIdx v2) {
            EdgeIdx e0         = static_cast<EdgeIdx>(_faces.size() * (sizeof(Triangle) / sizeof(HalfEdge)));
            Triangle &  tri    = _faces.emplace_back();
            tri._e[0]._to      = v2;
            tri._e[0]._twin    = GetTwin(pairs, v1, v2, e0);
            tri._e[0]._prev    = 2;
            tri._e[0]._next    = 1;
            tri._e[0]._idx     = 0;
            tri._e[0]._padding = 0;
            tri._e[1]._to      = v0;
            tri._e[1]._twin    = GetTwin(pairs, v2, v0, e0 + 1);
            tri._e[1]._prev    = -1;
            tri._e[1]._next    = 1;
            tri._e[1]._idx     = 1;
            tri._e[1]._padding = 0;
            tri._e[2]._to      = v1;
            tri._e[2]._twin    = GetTwin(pairs, v0, v1, e0 + 2);
            tri._e[2]._prev    = -1;
            tri._e[2]._next    = -2;
            tri._e[2]._idx     = 2;
            tri._e[2]._padding = 0;

            _verts.resize(std::max<std::size_t>({ _verts.size(), v0 + 1, v1 + 1, v2 + 1 }), ~0);

            if (! ~_verts[v0]) _verts[v0] = e0 + 2;
            if (! ~_verts[v1]) _verts[v1] = e0;
            if (! ~_verts[v2]) _verts[v2] = e0 + 1;

            return tri;
        }


        // Function: FlipEdge
        // Function: FlipEdge
        // 翻转边 (v0, v1) -> (v2, v3)
        template<typename T>
        bool FlipEdge(EdgeIdx edgeIndex, std::vector<T> &data, T newE) {
            // 1. 获取指针
            HalfEdge* h_common = reinterpret_cast<HalfEdge*>(_faces.data()) + edgeIndex;
            if (!h_common->TwinEdgeOr(nullptr)) return false; // 边界不能翻
            HalfEdge* t_common = h_common + h_common->_twin;

            // T0 的边: h0(公共), h1, h2
            HalfEdge* h0 = h_common;
            HalfEdge* h1 = h0 + h0->_next;
            HalfEdge* h2 = h1 + h1->_next;

            // T1 的边: t0(公共), t1, t2
            HalfEdge* t0 = t_common;
            HalfEdge* t1 = t0 + t0->_next;
            HalfEdge* t2 = t1 + t1->_next;

            // 2. 获取顶点 ID
            VertexIdx v0 = h0->From();
            VertexIdx v1 = h0->To();
            VertexIdx v2 = h1->To();
            VertexIdx v3 = t1->To();

            T d01 = data[IndexOf(h1)];
            T d02 = data[IndexOf(h2)];
            T d11 = data[IndexOf(t1)];
            T d12 = data[IndexOf(t2)];

            // 3. 获取 4 个外部 Twin (如果存在)
            // 这些是我们需要通知更新的对象
            HalfEdge* h1_outer = h1->TwinEdgeOr(nullptr); // v1->v2 的外部
            HalfEdge* h2_outer = h2->TwinEdgeOr(nullptr); // v2->v0 的外部
            HalfEdge* t1_outer = t1->TwinEdgeOr(nullptr); // v0->v3 的外部
            HalfEdge* t2_outer = t2->TwinEdgeOr(nullptr); // v3->v1 的外部

            // ==================================================
            // 4. 重新分配槽位 (Rewiring)
            // ==================================================
            // 目标结构:
            // 新 T0: (v2, v3, v1) -> 边: (v2->v3), (v3->v1), (v1->v2)
            // 新 T1: (v3, v2, v0) -> 边: (v3->v2), (v2->v0), (v0->v3)

            // --- 配置新 T0 (使用原 h0, h1, h2 的内存) ---
            
            // Slot h0: 新公共边 v2->v3
            h0->_to = v3;
            h0->_twin = static_cast<int>(t0 - h0); // 连到 t0

            // Slot h1: 承接原 t2 (v3->v1)
            h1->_to = v1;
            if (t2_outer) {
                h1->_twin = static_cast<int>(t2_outer - h1);
                t2_outer->_twin = static_cast<int>(h1 - t2_outer); // 更新外部
            } else { h1->_twin = 0; }

            // Slot h2: 承接原 h1 (v1->v2)
            h2->_to = v2;
            if (h1_outer) {
                h2->_twin = static_cast<int>(h1_outer - h2);
                h1_outer->_twin = static_cast<int>(h2 - h1_outer); // 更新外部
            } else { h2->_twin = 0; }


            // --- 新 T1: (v3, v2, v0) ---
            // 边: (v3->v2), (v2->v0), (v0->v3)

            // Slot t0: 新公共边 v3->v2
            t0->_to = v2;
            t0->_twin = static_cast<int>(h0 - t0); // 连到 h0

            // Slot t1: 承接原 h2 (v2->v0)
            t1->_to = v0;
            if (h2_outer) {
                t1->_twin = static_cast<int>(h2_outer - t1);
                h2_outer->_twin = static_cast<int>(t1 - h2_outer); // 更新外部
            } else { t1->_twin = 0; }

            // Slot t2: 承接原 t1 (v0->v3)
            t2->_to = v3;
            if (t1_outer) {
                t2->_twin = static_cast<int>(t1_outer - t2);
                t1_outer->_twin = static_cast<int>(t2 - t1_outer); // 更新外部
            } else { t2->_twin = 0; }
            

            // 5. 更新顶点指针
            // v0 出发: t2 (v0->v3)
            // v1 出发: h2 (v1->v2)
            // v2 出发: h0 (v2->v3)
            // v3 出发: t0 (v3->v2)
            
            _verts[v0] = IndexOf(t2);
            _verts[v1] = IndexOf(h2);
            _verts[v2] = IndexOf(h0);
            _verts[v3] = IndexOf(t0);


            // 更新data

            data[IndexOf(h0)] = newE;
            data[IndexOf(h1)] = d12;
            data[IndexOf(h2)] = d01;
            data[IndexOf(t0)] = newE;
            data[IndexOf(t1)] = d02;
            data[IndexOf(t2)] = d11;
            return true;
        }
    };


}