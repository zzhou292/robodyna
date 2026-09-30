#include "AcceptedSurfaceMesh.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <new>
#include <set>
#include <tuple>
#include <type_traits>
#include <utility>

namespace crash::visual {
namespace {
using NodeKey = std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>;
NodeKey Key(SourceNode s) { return {s.asset, s.instance, s.node}; }
bool Finite(const chrono::ChVector3d& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}
Report Validate(const Binding& b) {
    if (!b.identity.owner || !b.identity.run || !b.identity.topology || !b.tl_node_count ||
        b.tl_node_count > std::numeric_limits<std::size_t>::max() / 3)
        return {Status::InvalidBinding, "Missing identity or invalid TL node count"};
    if (!b.max_vertices || b.max_vertices > 4096 || !b.max_triangles || b.max_triangles > 8192 ||
        b.vertices.size() > b.max_vertices || b.triangles.size() > b.max_triangles)
        return {Status::ResourceLimit, "Surface preview capacity exceeded"};
    if (b.vertices.empty() || b.triangles.empty())
        return {Status::InvalidBinding, "Empty surface topology"};
    std::map<std::uint32_t, NodeKey> node_sources;
    std::map<NodeKey, std::uint32_t> source_nodes;
    for (const auto& v : b.vertices) {
        if (v.tl_node >= b.tl_node_count || !v.source.asset || !v.source.node)
            return {Status::InvalidBinding, "Invalid vertex node/source mapping"};
        auto node = node_sources.emplace(v.tl_node, Key(v.source));
        auto source = source_nodes.emplace(Key(v.source), v.tl_node);
        if (node.first->second != Key(v.source) || source.first->second != v.tl_node)
            return {Status::InvalidBinding, "Inconsistent physical-node/source identity"};
    }
    using FaceKey = std::tuple<std::uint64_t, std::uint64_t, std::uint64_t, std::uint32_t, std::uint32_t>;
    std::set<FaceKey> faces;
    std::map<NodeKey, std::uint64_t> element_parts;
    for (const auto& t : b.triangles) {
        if (!t.asset || !t.element || !t.part ||
            !faces.emplace(t.asset, t.instance, t.element, t.local_face, t.subtriangle).second)
            return {Status::InvalidBinding, "Missing or duplicate source-face identity"};
        auto part = element_parts.emplace(NodeKey{t.asset, t.instance, t.element}, t.part);
        if (part.first->second != t.part)
            return {Status::InvalidBinding, "Source element has inconsistent part identity"};
        for (int i = 0; i < 3; ++i) {
            if (t.vertices[i] >= b.vertices.size())
                return {Status::InvalidBinding, "Triangle index outside display vertex space"};
            for (int j = 0; j < i; ++j)
                if (b.vertices[t.vertices[i]].tl_node == b.vertices[t.vertices[j]].tl_node)
                    return {Status::InvalidBinding, "Triangle repeats one physical node"};
        }
    }
    return {Status::Ok, "Surface binding validated"};
}
}  // namespace

struct AcceptedSurfaceMesh::Impl {
    explicit Impl(const Binding& b)
        : binding(b), mesh(std::make_shared<chrono::ChTriangleMeshConnected>()),
          shape(std::make_shared<chrono::ChVisualShapeTriangleMesh>()), staged(b.vertices.size()) {
        mesh->GetCoordsVertices().resize(b.vertices.size());
        auto& indices = mesh->GetIndicesVertices();
        indices.reserve(b.triangles.size());
        for (const auto& t : b.triangles)
            indices.emplace_back(static_cast<int>(t.vertices[0]), static_cast<int>(t.vertices[1]),
                                 static_cast<int>(t.vertices[2]));
        shape->SetMesh(mesh, false);
        shape->SetMutable(true);
        shape->SetFixedConnectivity();
    }
    const Binding binding;
    std::shared_ptr<chrono::ChTriangleMeshConnected> mesh;
    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> shape;
    std::vector<chrono::ChVector3d> staged;
    FrameStamp stamp{};
    bool published = false;
};

AcceptedSurfaceMesh::AcceptedSurfaceMesh() = default;
AcceptedSurfaceMesh::~AcceptedSurfaceMesh() = default;
Report AcceptedSurfaceMesh::Initialize(const Binding& b) {
    if (impl_) return {Status::InvalidBinding, "Surface is already initialized"};
    try {
        const auto report = Validate(b);
        if (report.status != Status::Ok) return report;
        auto next = std::make_unique<Impl>(b);
        impl_ = std::move(next);
        return {Status::Ok, "Surface binding initialized"};
    } catch (const std::bad_alloc&) {
        return {Status::ResourceLimit, "Surface binding allocation failed"};
    }
}

Report AcceptedSurfaceMesh::Publish(tl::fea::HostNodalKinematicsView in, const FrameStamp& stamp) {
    if (!impl_) return {Status::NotInitialized, "Surface has no binding"};
    auto& s = *impl_;
    const auto& b = s.binding;
    if (stamp.identity.owner != b.identity.owner) return {Status::WrongOwner, "Wrong TL owner"};
    if (stamp.identity.run != b.identity.run) return {Status::WrongRun, "Wrong run identity"};
    if (stamp.identity.topology != b.identity.topology) return {Status::WrongTopology, "Wrong topology identity"};
    if (!std::isfinite(stamp.time) || stamp.time < 0 || !in.position_xyz || in.node_count != b.tl_node_count)
        return {Status::InvalidFrame, "Invalid accepted time or borrowed position view"};
    if (s.published && (stamp.epoch <= s.stamp.epoch || stamp.time <= s.stamp.time))
        return {Status::StaleFrame, "Accepted epoch and time must both advance"};
    for (std::size_t i = 0; i < b.vertices.size(); ++i) {
        const std::size_t node = b.vertices[i].tl_node;
        s.staged[i] = chrono::ChVector3d(in.position_xyz[3 * node], in.position_xyz[3 * node + 1],
                                       in.position_xyz[3 * node + 2]);
        if (!Finite(s.staged[i])) return {Status::InvalidFrame, "Nonfinite mapped position"};
    }
    for (const auto& t : b.triangles) {
        const auto u = s.staged[t.vertices[1]] - s.staged[t.vertices[0]];
        const auto v = s.staged[t.vertices[2]] - s.staged[t.vertices[0]];
        if (!Finite(u) || !Finite(v)) return {Status::InvalidFrame, "Unrepresentable triangle edge"};
        const double scale = std::max({std::abs(u.x()), std::abs(u.y()), std::abs(u.z()),
                                       std::abs(v.x()), std::abs(v.y()), std::abs(v.z())});
        if (!(scale > 0)) return {Status::InvalidFrame, "Degenerate display triangle"};
        const auto n = chrono::Vcross(u / scale, v / scale);
        if (!(std::hypot(n.x(), n.y(), n.z()) > 0))
            return {Status::InvalidFrame, "Degenerate display triangle"};
    }
    // No allocation, observer callback or throwing operation during publication.
    // Readers/renderers are externally serialized with this operation.
    static_assert(std::is_nothrow_copy_assignable<FrameStamp>::value, "Frame publication cannot throw");
    s.mesh->GetCoordsVertices().swap(s.staged);
    s.stamp = stamp;
    s.published = true;
    return {Status::Ok, "Accepted surface frame published"};
}

const Binding* AcceptedSurfaceMesh::binding() const noexcept { return impl_ ? &impl_->binding : nullptr; }
const FrameStamp* AcceptedSurfaceMesh::frame() const noexcept {
    return impl_ && impl_->published ? &impl_->stamp : nullptr;
}
std::shared_ptr<const chrono::ChTriangleMeshConnected> AcceptedSurfaceMesh::mesh() const noexcept {
    return impl_ && impl_->published ? impl_->mesh : nullptr;
}
std::shared_ptr<chrono::ChVisualShapeTriangleMesh> AcceptedSurfaceMesh::visual_shape() const noexcept {
    return impl_ && impl_->published ? impl_->shape : nullptr;
}
}  // namespace crash::visual
