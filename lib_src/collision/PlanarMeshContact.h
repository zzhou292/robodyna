#pragma once

#include "PlanarContactTypes.h"
#include "SurfaceContactGeometry.h"
#include "lib_src/solvers/FENodalState.h"
#include <cstddef>
#include <cstdint>
#include <memory>

namespace tlfea::contact {
constexpr std::uint32_t MaxPlanarSurfaceNodes = 25, MaxPlanarSurfaceTriangles = 32;
struct PlanarSurfaceNode {
  std::uint32_t global_node = 0;
  Vec3 reference_position;
  std::uint64_t source_node_id = 0;
};
struct PlanarSurfaceTriangle {
  std::uint32_t local_nodes[3]{};  // Indices into PlanarSurfaceView.nodes.
  std::uint64_t triangle_id = 0;
};
struct PlanarSurfaceView {
  const PlanarSurfaceNode* nodes = nullptr;
  std::uint32_t node_count = 0;
  const PlanarSurfaceTriangle* triangles = nullptr;
  std::uint32_t triangle_count = 0;
};
struct PlanarContactConfig {
  std::uint64_t owner_id = 0;  // FENodalState's immutable physical-node owner.
  std::uint32_t global_node_count = 0;
  double penalty_per_area = 0;  // N/m^3; point stiffness = this * reference area.
  double exposed_boundary_clearance_m = 1e-6;
  // Checked at force evaluation and on prepared vertices incident to covered
  // triangles. Wholly outside patches have no depth constraint. Not CCD.
  double max_penetration_m = 0;
  std::size_t max_device_bytes = MaxPlanarContactDeviceBytes;
};
struct PlanarContactSample {
  std::uint64_t surface_triangle_id = 0, wall_triangle_id = 0;
  std::uint64_t wall_source_quad_id = 0, wall_assembled_source_quad_id = 0;
  FeatureKey wall_feature;
  Vec3 surface_point, wall_point, force_on_surface;
  double reference_area = 0, stiffness = 0, gap = 0, normal_velocity = 0;
  double elastic_energy = 0, surface_power = 0;
  bool covered = false, active = false;
};
struct PlanarContactDiagnostics {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  std::uint32_t sample_count = 0, covered_count = 0, active_count = 0;
  PlanarContactSample samples[MaxPlanarSurfaceTriangles]{};
  Vec3 force_on_surface, wall_reaction, wall_moment;
  double elastic_energy = 0, surface_power = 0;
  bool valid = false;
};
struct PlanarContactAllocationInfo {
  std::size_t device_bytes = 0, device_allocations = 0;
};

// Bounded normal-only surface quadrature against a FINITE planar -X wall.
// Initial data is copied; all runtime allocations occur at Initialize. Complete
// projected surface triangles must be covered or wholly outside, away from
// exposed wall boundaries. Internal seams receive exactly one spring per
// surface centroid. The same barycentric J evaluates velocity and scatters force.
// No shell offsets/rotations, friction, damping, internal elasticity or CCD.
// Initial reference triangles are parallel to the wall. Different later nodal
// normal motions are representable, but the frozen rank-one bound alone does
// not qualify arbitrary asynchronous unilateral activation or nonlinear motion.
//
// The coordinator alone owns accepted/trial state and clears assembly once.
// Evaluate adds forces and ALL covered-point stiffness bounds, even inactive.
// Calls/writes are serialized on the NodalAssemblyView stream; Evaluate waits
// for completion. Never call twice for one owner/epoch/attempt. On ANY failure,
// discard that nodal trial; valid device assembly is also marked failed.
// No accepted time/history is owned here. Diagnostics describe force at the
// evaluation epoch; the app may log a COPY only after the matching nodal commit.
// A later failed evaluation invalidates diagnostics. CUDA errors poison this
// batch, not a promise of context recovery or continued nodal-owner usability.
class PlanarMeshContact {
 public:
  PlanarMeshContact();
  ~PlanarMeshContact();
  PlanarMeshContact(const PlanarMeshContact&) = delete;
  PlanarMeshContact& operator=(const PlanarMeshContact&) = delete;
  PlanarContactReport Initialize(const PlanarContactConfig&, PlanarWallView, PlanarSurfaceView);
  PlanarContactReport Evaluate(const tl::fea::NodalAssemblyView&);
  // Read-only validation of the prepared candidate after Advance and before
  // Commit; not a force evaluation or acceptance authority. Caller Discard on
  // failure. Successful validation retains the force-epoch diagnostics.
  PlanarContactReport ValidatePrepared(const tl::fea::NodalPreparedView&);
  const PlanarContactDiagnostics* diagnostics() const noexcept;
  PlanarContactAllocationInfo allocations() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace tlfea::contact
