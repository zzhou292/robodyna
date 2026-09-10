#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// Immutable source/topology data, independent of nodal mechanics or rendering.
namespace crash::visual {
struct Identity {
    std::uint64_t owner = 0, run = 0, topology = 0;
};
struct SourceNode {
    std::uint64_t asset = 0, instance = 0, node = 0;
};
struct VertexBinding {
    std::uint32_t tl_node = 0;
    SourceNode source;
};
struct TriangleBinding {
    std::array<std::uint32_t, 3> vertices{};
    std::uint64_t asset = 0, instance = 0, element = 0, part = 0;
    std::uint32_t local_face = 0;   // Physical parent face, unchanged by display triangulation.
    std::uint32_t subtriangle = 0; // Display piece ordinal within that physical face.
};
struct Binding {
    Identity identity;
    std::size_t tl_node_count = 0;
    std::vector<VertexBinding> vertices;
    std::vector<TriangleBinding> triangles;
    // Explicit bounds for this preview slice; whole-vehicle capacity is a
    // separate admission decision. Callers may lower, but not raise, these.
    std::size_t max_vertices = 4096, max_triangles = 8192;
};
} // namespace crash::visual
