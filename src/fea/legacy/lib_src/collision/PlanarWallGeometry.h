#pragma once

#include "PlanarContactTypes.h"
#include "SurfaceContactGeometry.h"
#include <vector>

namespace tlfea::contact {
namespace planar_detail {
struct WallFace {
  TriangleGeometry geometry;
  std::uint64_t source_quad_id = 0, assembled_source_quad_id = 0;
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
struct BoundarySegment { Vec3 a, b; };
}  // namespace planar_detail

// Immutable validated finite -X triangle mesh. Startup copies only its checked
// faces and exposed boundary. No positions, state, stream or physical clock.
// Input changes after Initialize do not change this object. Failure preserves
// the previous object and ClassifyTriangle preserves the caller result.
class PlanarWallGeometry {
 public:
  PlanarContactReport Initialize(PlanarWallView);
  PlanarContactReport ClassifyTriangle(const TriangleGeometry& projected,
                                      double exposed_clearance, bool* covered) const;
  bool initialized() const noexcept { return initialized_; }
  double wall_x() const noexcept { return wall_x_; }
  double tolerance() const noexcept { return tolerance_; }
  const std::vector<planar_detail::WallFace>& faces() const noexcept { return faces_; }
 private:
  std::vector<planar_detail::WallFace> faces_;
  std::vector<planar_detail::BoundarySegment> boundary_;
  double wall_x_ = 0, tolerance_ = 0;
  bool initialized_ = false;
};
}  // namespace tlfea::contact
