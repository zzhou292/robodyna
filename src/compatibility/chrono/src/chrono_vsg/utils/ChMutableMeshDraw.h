// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
// Copyright (c) 2026 projectchrono.org
// Use of this source code is governed by the BSD-style license in LICENSE.
// =============================================================================

#ifndef CH_MUTABLE_MESH_DRAW_VSG_H
#define CH_MUTABLE_MESH_DRAW_VSG_H

#include <vsg/all.h>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace chrono::vsg3d {

/// The existing colored triangle-soup builder emits one indexed draw with
/// sequential indices. Its initial allocation bounds future visible faces.
/// Reusing that allocation permits removal, empty frames and backward replay
/// without rebinding or submitting the stale tail of the vertex buffers.
class MutableMeshDraw {
  public:
    bool Initialize(vsg::Node& node, std::size_t vertex_capacity) {
        if (m_draw || !vertex_capacity || vertex_capacity % 3 ||
            vertex_capacity > std::numeric_limits<std::uint32_t>::max())
            return false;
        FindDraw visitor;
        node.accept(visitor);
        if (visitor.count != 1 || !visitor.draw || visitor.draw->indexCount != vertex_capacity ||
            visitor.draw->firstIndex != 0 || visitor.draw->vertexOffset != 0 ||
            visitor.draw->instanceCount != 1 || visitor.draw->firstInstance != 0 ||
            !visitor.draw->indices || !visitor.draw->indices->data)
            return false;
        auto indices = visitor.draw->indices->data.cast<vsg::uintArray>();
        if (!indices || indices->size() != vertex_capacity)
            return false;
        for (std::size_t i = 0; i < vertex_capacity; ++i)
            if ((*indices)[i] != i)
                return false;
        m_draw = visitor.draw;
        m_capacity = vertex_capacity;
        return true;
    }

    bool Accepts(std::size_t faces) const noexcept { return m_draw && faces <= m_capacity / 3; }

    /// Caller updates the live prefix first, after checking Accepts(). VSG reads
    /// indexCount while recording the next frame; GPU allocations stay fixed.
    bool Publish(std::size_t faces) noexcept {
        if (!Accepts(faces))
            return false;
        m_draw->indexCount = static_cast<std::uint32_t>(3 * faces);
        return true;
    }

    bool initialized() const noexcept { return bool(m_draw); }
    std::size_t capacity() const noexcept { return m_capacity; }

  private:
    struct FindDraw : vsg::Visitor {
        void apply(vsg::Object& object) override { object.traverse(*this); }
        void apply(vsg::VertexIndexDraw& value) override { draw = &value; ++count; }
        vsg::ref_ptr<vsg::VertexIndexDraw> draw;
        std::size_t count = 0;
    };
    vsg::ref_ptr<vsg::VertexIndexDraw> m_draw;
    std::size_t m_capacity = 0;
};

}  // namespace chrono::vsg3d
#endif
