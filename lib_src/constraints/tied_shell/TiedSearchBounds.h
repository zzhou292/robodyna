// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected I2BUC1/I2TRIVOX Ignore2/DSEARCH0, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "TiedSearchTypes.h"

namespace tl::constraints::tied_shell {
struct SearchBoundsInput {
  Vec3 master_position_m[4]{};
  MasterTopology topology=MasterTopology::Quad;
  double master_thickness_m=0;
  // I2BUC1 uses the maximum over the interface, not the per-node thickness
  // used by I2COR3's final projection gap. Zero for this original weld scope.
  double maximum_secondary_shell_thickness_m=0;
  double working_length_to_m=0;
};
struct NativeSearchBounds {
  Vec3 minimum{},maximum{};  // Original working units, not metres.
  double inflation=0,working_length_to_m=0;
};
struct WorkingSearchBoundsInput {
  Vec3 master_position[4]{};
  MasterTopology topology=MasterTopology::Quad;
  double master_thickness=0, maximum_secondary_shell_thickness=0;
  double working_length_to_m=0;
};

namespace search_detail {
// Shared arithmetic admits a positive SI thickness rounded to zero by unit
// conversion. The original-working public overload separately requires >0.
TL_TIED_PATCH_HD inline Status PrepareWorkingBounds(const WorkingSearchBoundsInput& in,
    NativeSearchBounds& output) noexcept {
  namespace v=tl::math::fixed3;
  using tl::math::Finite;
  const bool triangle=in.topology==MasterTopology::TriangleRepeatedThird;
  if ((!triangle && in.topology!=MasterTopology::Quad) ||
      !Finite(in.working_length_to_m) || in.working_length_to_m<=0 ||
      !Finite(in.master_thickness) || in.master_thickness<0 ||
      !Finite(in.maximum_secondary_shell_thickness) ||
      in.maximum_secondary_shell_thickness<0) return Status::InvalidInput;
  const auto& c=in.master_position[2];
  const auto& d=in.master_position[3];
  if (triangle && (c.x!=d.x || c.y!=d.y || c.z!=d.z)) return Status::InvalidInput;
  const auto& x=in.master_position;
  for (unsigned i=0;i<4;++i) {
    if (!v::Finite(x[i])) return Status::InvalidInput;
  }
  const double a=v::Norm(v::Subtract(x[0],x[2]));
  const double b=v::Norm(v::Subtract(x[1],x[3]));
  const double thickness=in.master_thickness;
  const double secondary=in.maximum_secondary_shell_thickness;
  if (!Finite(a) || !Finite(b) || !Finite(thickness) || !Finite(secondary) ||
      !Finite(thickness+secondary)) return Status::NonfiniteResult;
  NativeSearchBounds next;
  // Search enumeration uses MAX diagonal; projection separately uses MIN.
  next.inflation=::fmax((5./100.)*::fmax(a,b),(6./10.)*(thickness+secondary));
  if (!Finite(next.inflation) || next.inflation<=0) return Status::NonfiniteResult;
  next.minimum=next.maximum=x[0];
  for (unsigned i=1;i<4;++i) {
    next.minimum={::fmin(next.minimum.x,x[i].x),::fmin(next.minimum.y,x[i].y),::fmin(next.minimum.z,x[i].z)};
    next.maximum={::fmax(next.maximum.x,x[i].x),::fmax(next.maximum.y,x[i].y),::fmax(next.maximum.z,x[i].z)};
  }
  const Vec3 margin{next.inflation,next.inflation,next.inflation};
  next.minimum=v::Subtract(next.minimum,margin);
  next.maximum=v::Add(next.maximum,margin);
  next.working_length_to_m=in.working_length_to_m;
  if (!v::Finite(next.minimum) || !v::Finite(next.maximum)) return Status::NonfiniteResult;
  output=next;
  return Status::Success;
}
} // namespace search_detail
TL_TIED_PATCH_HD inline Status PrepareSearchBounds(const WorkingSearchBoundsInput& in,
    NativeSearchBounds& output) noexcept {
  if (!tl::math::Finite(in.master_thickness) || in.master_thickness<=0)
    return Status::InvalidInput;
  return search_detail::PrepareWorkingBounds(in,output);
}

TL_TIED_PATCH_HD inline Status PrepareSearchBounds(const SearchBoundsInput& in,
    NativeSearchBounds& output) noexcept {
  namespace v=tl::math::fixed3;
  using tl::math::Finite;
  const bool triangle=in.topology==MasterTopology::TriangleRepeatedThird;
  if ((!triangle && in.topology!=MasterTopology::Quad) ||
      !Finite(in.working_length_to_m) || in.working_length_to_m<=0 ||
      !Finite(in.master_thickness_m) || in.master_thickness_m<=0 ||
      !Finite(in.maximum_secondary_shell_thickness_m) ||
      in.maximum_secondary_shell_thickness_m<0) return Status::InvalidInput;
  const auto& c=in.master_position_m[2];
  const auto& d=in.master_position_m[3];
  if (triangle && (c.x!=d.x || c.y!=d.y || c.z!=d.z)) return Status::InvalidInput;
  WorkingSearchBoundsInput working;
  working.topology=in.topology;
  working.working_length_to_m=in.working_length_to_m;
  for (unsigned i=0;i<4;++i) {
    if (!v::Finite(in.master_position_m[i])) return Status::InvalidInput;
    working.master_position[i]=v::Divide(in.master_position_m[i],in.working_length_to_m);
    if (!v::Finite(working.master_position[i])) return Status::NonfiniteResult;
  }
  working.master_thickness=in.master_thickness_m/in.working_length_to_m;
  working.maximum_secondary_shell_thickness=in.maximum_secondary_shell_thickness_m/in.working_length_to_m;
  if (!Finite(working.master_thickness) || !Finite(working.maximum_secondary_shell_thickness))
    return Status::NonfiniteResult;
  return search_detail::PrepareWorkingBounds(working,output);
}

// Apply the native inclusive box after conservative GPU candidate generation.
// A caller must also exclude a secondary with a master's physical node ID.
// This preserves native startup geometry admission, including its area-floor
// branches; it is not a nonsingular force-patch or full-vehicle admission.
TL_TIED_PATCH_HD inline Status WithinWorkingSearchBounds(const NativeSearchBounds& bounds,
    Vec3 point,bool& output) noexcept {
  namespace v=tl::math::fixed3;
  if (!v::Finite(bounds.minimum) || !v::Finite(bounds.maximum) ||
      !tl::math::Finite(bounds.inflation) || bounds.inflation<=0 ||
      !tl::math::Finite(bounds.working_length_to_m) || bounds.working_length_to_m<=0 ||
      bounds.minimum.x>bounds.maximum.x || bounds.minimum.y>bounds.maximum.y ||
      bounds.minimum.z>bounds.maximum.z || !v::Finite(point)) return Status::InvalidInput;
  output=point.x>=bounds.minimum.x && point.x<=bounds.maximum.x &&
      point.y>=bounds.minimum.y && point.y<=bounds.maximum.y &&
      point.z>=bounds.minimum.z && point.z<=bounds.maximum.z;
  return Status::Success;
}
TL_TIED_PATCH_HD inline Status WithinSearchBounds(const NativeSearchBounds& bounds,
    Vec3 point_m,bool& output) noexcept {
  namespace v=tl::math::fixed3;
  if (!v::Finite(point_m) || !tl::math::Finite(bounds.working_length_to_m) ||
      bounds.working_length_to_m<=0) return Status::InvalidInput;
  const auto point=v::Divide(point_m,bounds.working_length_to_m);
  if (!v::Finite(point)) return Status::NonfiniteResult;
  return WithinWorkingSearchBounds(bounds,point,output);
}
} // namespace tl::constraints::tied_shell
