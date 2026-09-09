#pragma once

#include "PlanarWallGeometry.h"
#include <type_traits>

namespace tlfea::contact {

struct PreparedWallQueryDiagnostics {
  bool used_filter=false;
  std::uint32_t skipped_faces=0;
};

// A copied immutable acceleration packet, not a replacement wall validator.
// Initialize examines the COMPLETE supplied face array. Optimization-ineligible
// geometry is retained and queried through the unchanged full FindOwner scan.
// Fixed capacity matches the existing planar wall; no allocation or owner.
// A successful Initialize replaces the packet; argument failure preserves it.
class PreparedPlanarWallQuery {
 public:
  Status Initialize(const planar_detail::WallFace*,std::uint32_t count,double tolerance);
  TL_SURFACE_HD bool prepared() const { return prepared_; }
  TL_SURFACE_HD bool filter_eligible() const { return eligible_; }
  TL_SURFACE_HD std::uint32_t face_count() const { return count_; }

  // Same partial-output semantics as planar_detail::FindOwner: reset owner,
  // publish each selected candidate immediately, preserve point if none was
  // selected, and retain any earlier candidate if a later face fails. Extra
  // null/unprepared guards only close otherwise undefined caller operations.
  // The packet must be an unmodified copy produced by Initialize; it cannot
  // authenticate caller-fabricated bytes or a mismatched external face array.
  // All pointed-to objects belong to the current host/device execution space.
  TL_SURFACE_HD Status FindOwner(Vec3 query,std::uint32_t* owner,TrianglePointGeometry* point,
                                PreparedWallQueryDiagnostics* diagnostics=nullptr) const {
    if (diagnostics) *diagnostics={};
    if (!owner || !point) return Status::kInvalidArgument;
    *owner=UINT32_MAX;
    if (!prepared_) return Status::kInvalidArgument;
    // Exact common plane and a conservative finite-operation domain. A query
    // outside it takes the entire original scan, including invalid far faces.
    if (!eligible_ || !IsFinite(query) || query.x!=wall_x_ ||
        ::fabs(query.y)>CoordinateLimit || ::fabs(query.z)>CoordinateLimit)
      return planar_detail::FindOwner(query,faces_,count_,tolerance_,owner,point);
    if (diagnostics) diagnostics->used_filter=true;
    for (std::uint32_t i=0;i<count_;++i) {
      const auto& box=boxes_[i];
      if (query.y<box.y_min || query.y>box.y_max || query.z<box.z_min || query.z>box.z_max) {
        if (diagnostics) ++diagnostics->skipped_faces;
        continue;
      }
      TrianglePointGeometry next;
      const auto status=ClosestPointOnTriangle(query,faces_[i].geometry,&next);
      if (status!=Status::kOk) return status;
      if (next.degenerate) return Status::kInvalidArgument;
      if (next.distance<=tolerance_ &&
          (*owner==UINT32_MAX || faces_[i].geometry.face_id<faces_[*owner].geometry.face_id)) {
        *owner=i; *point=next;
      }
    }
    return Status::kOk;
  }

 private:
  static constexpr double CoordinateLimit=0x1p100;
  struct Box { double y_min=0,y_max=0,z_min=0,z_max=0; };
  planar_detail::WallFace faces_[MaxPlanarWallTriangles]{};
  Box boxes_[MaxPlanarWallTriangles]{};
  std::uint32_t count_=0;
  double wall_x_=0,tolerance_=0;
  bool prepared_=false,eligible_=false;
};
static_assert(sizeof(PreparedPlanarWallQuery)<80*1024,"Bounded copied wall-query packet");
static_assert(std::is_trivially_copyable<PreparedPlanarWallQuery>::value,
              "Host preparation is transported as one immutable byte copy");
} // namespace tlfea::contact
