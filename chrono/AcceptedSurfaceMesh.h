#pragma once

#include "lib_src/solvers/FENodalStateView.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace chrono {
class ChTriangleMeshConnected;
class ChVisualShapeTriangleMesh;
}

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
struct FrameStamp {
    Identity identity;
    std::uint64_t epoch = 0;
    double time = 0;
};
enum class Status {
    Ok, InvalidBinding, InvalidFrame, WrongOwner, WrongRun, WrongTopology,
    StaleFrame, ResourceLimit, NotInitialized
};
struct Report {
    Status status;
    const char* message;  // Static diagnostic; failure reporting cannot allocate.
};

// Application output adapter, not a dynamics owner. The coordinator calls
// Publish only with the TL owner's accepted() state after commit. Identity tags
// validate provenance consistency; they do not prove that a trial was accepted.
// World positions are already in the canonical assembled SI frame; this class
// applies no transform, contact offset, integration or collision response.
// The source compiler owns raw source-connectivity authenticity. Node and
// element identities may come from different includes; the adapter checks the
// supplied map's internal consistency, not an original deck or manifest hash.
// Calls and rendering must be externally serialized. Borrowed nodal memory is
// needed only until Publish returns; only mapped positions are read (V/VR are
// deliberately unused). Updates stage complete data before no-throw publication.
class AcceptedSurfaceMesh {
  public:
    AcceptedSurfaceMesh();
    ~AcceptedSurfaceMesh();
    AcceptedSurfaceMesh(const AcceptedSurfaceMesh&) = delete;
    AcceptedSurfaceMesh& operator=(const AcceptedSurfaceMesh&) = delete;

    Report Initialize(const Binding&);
    Report Publish(tl::fea::HostNodalKinematicsView, const FrameStamp&);
    // Borrowed metadata views are valid for the adapter lifetime. frame() is
    // live and changes on publication; copy it to retain a historical stamp.
    const Binding* binding() const noexcept;
    const FrameStamp* frame() const noexcept;
    // Live Chrono geometry, not an immutable historical snapshot. Do not retain
    // vertex data pointers/iterators across Publish; its vector storage swaps.
    std::shared_ptr<const chrono::ChTriangleMeshConnected> mesh() const noexcept;
    // Null until the first valid frame. Attach this handle to an identity-frame
    // fixed visual carrier. Borrowers may render it, but must not mutate mesh,
    // topology, scale or attachment transform behind the adapter. The current
    // headless gate and planned VSG path do not qualify every Chrono renderer.
    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> visual_shape() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace crash::visual
