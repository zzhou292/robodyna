#pragma once

#include "PlanarMeshContact.h"
#include "SurfaceContactLaw.h"
#include <array>

namespace tlfea::contact {
namespace planar_detail {
struct WallFace {
  TriangleGeometry geometry;
  std::uint64_t source_quad_id = 0, assembled_source_quad_id = 0;
};
struct SurfacePoint {
  SurfaceTriangle triangle;
  std::uint32_t local_nodes[3]{};
  double area = 0, stiffness = 0;
  bool covered = false;
};
struct Control {
  PlanarContactDiagnostics diagnostics;
  PlanarContactStatus status = PlanarContactStatus::Ok;
  std::uint32_t sample = UINT32_MAX;
  tl::fea::stability::RowContribution contributions[MaxPlanarSurfaceTriangles]{};
  Vec3 staged_force[MaxPlanarSurfaceNodes]{};
};
// Finite containment: project onto the common plane, then query every closed
// triangle. Tolerance handles roundoff at INTERNAL seams only; initialization
// excludes the exposed-boundary band. Smallest stable triangle ID wins ties.
TL_SURFACE_HD inline Status FindOwner(Vec3 projected, const WallFace* wall,
                                     std::uint32_t count, double tolerance,
                                     std::uint32_t* owner, TrianglePointGeometry* point) {
  *owner = UINT32_MAX;
  for (std::uint32_t i = 0; i < count; ++i) {
    TrianglePointGeometry next;
    const auto status = ClosestPointOnTriangle(projected, wall[i].geometry, &next);
    if (status != Status::kOk) return status;
    if (next.degenerate) return Status::kInvalidArgument;
    if (next.distance <= tolerance &&
        (*owner == UINT32_MAX || wall[i].geometry.face_id < wall[*owner].geometry.face_id)) {
      *owner = i; *point = next;
    }
  }
  return Status::kOk;
}
}  // namespace planar_detail

struct PlanarMeshContact::Impl {
  ~Impl();
  PlanarContactConfig config;
  PlanarContactAllocationInfo allocation;
  std::uint32_t wall_count = 0, node_count = 0, point_count = 0;
  double wall_x = 0, geometry_tolerance = 0;
  planar_detail::WallFace* wall = nullptr;
  PlanarSurfaceNode* nodes = nullptr;
  planar_detail::SurfacePoint* points = nullptr;
  planar_detail::Control* control = nullptr;
  planar_detail::Control host_control;
  std::uint64_t last_epoch = 0, last_attempt = 0;
  bool usable = true, attempted = false;
};
}  // namespace tlfea::contact
