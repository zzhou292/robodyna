// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected I2COR3/I2DST3 Ignore2/DSEARCH0: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "TiedSearchProjection.h"

namespace tl::constraints::tied_shell {
namespace search_detail {
// Converted positive SI thickness can underflow to zero. Preserve that legacy
// diagonal-based domain; public original-working inputs are checked separately.
TL_TIED_PATCH_HD inline Status ProjectWorkingCandidate(const WorkingSearchInput& input,
    CandidateProjection& output) noexcept {
  namespace v=tl::math::fixed3;
  using tl::math::Finite;
  const bool triangle=input.topology==MasterTopology::TriangleRepeatedThird;
  if ((!triangle && input.topology!=MasterTopology::Quad) ||
      !Finite(input.working_length_to_m) || input.working_length_to_m<=0 ||
      !Finite(input.master_thickness) || input.master_thickness<0 ||
      !Finite(input.secondary_shell_thickness) || input.secondary_shell_thickness<0 ||
      !v::Finite(input.geometry.secondary_position)) return Status::InvalidInput;
  const auto& third=input.geometry.master_position[2];
  const auto& fourth=input.geometry.master_position[3];
  if (triangle && (third.x!=fourth.x || third.y!=fourth.y || third.z!=fourth.z))
    return Status::InvalidInput;
  const auto& x=input.geometry.master_position;
  for (unsigned i=0;i<4;++i) {
    if (!v::Finite(x[i])) return Status::InvalidInput;
  }
  const auto point=input.geometry.secondary_position;
  const double master=input.master_thickness;
  const double secondary=input.secondary_shell_thickness;
  const double diagonal13=v::Norm(v::Subtract(x[0],x[2]));
  const double diagonal24=v::Norm(v::Subtract(x[1],x[3]));
  if (!Finite(diagonal13) || !Finite(diagonal24)) return Status::NonfiniteResult;
  const double diagonal=::fmin(diagonal13,diagonal24);
  // Native ZEP05 is FIVE/EP02 = .05 (not HALF).
  const double gap=::fmax((5./100.)*diagonal,(1.-4./10.)*(secondary+master));
  if (!v::Finite(point) || !Finite(master) || !Finite(secondary) ||
      !Finite(diagonal) || !Finite(gap) || gap<=0) return Status::NonfiniteResult;
  const Vec3 center=triangle?x[2]:v::Scale(v::Add(v::Add(v::Add(x[0],x[1]),x[2]),x[3]),.25);
  if (!v::Finite(center)) return Status::NonfiniteResult;
  search_detail::FanProjection fans[4]{};
  CandidateProjection next;
  for (unsigned i=0;i<4;++i) {
    if (!search_detail::ProjectFan(point,center,x[i],x[(i+1)%4],gap,triangle,fans[i]))
      return Status::NonfiniteResult;
    if (fans[i].penetration>fans[next.fan].penetration) next.fan=i;
  }
  if (triangle) next.fan=0;
  const auto& f=fans[next.fan];
  if (triangle) {
    next.t=1.-2.*f.lb-2.*f.lc;
    if (next.t<1.-1e-10) next.s=(f.lc-f.lb)/(f.lc+f.lb);
    else if (f.lb < -1e-10) next.s=2.;
    else if (f.lc < -1e-10) next.s=-2.;
  } else {
    switch(next.fan) {
      case 0: next.s=-f.lb+f.lc; next.t=-f.lb-f.lc; break;
      case 1: next.s= f.lb+f.lc; next.t=-f.lb+f.lc; break;
      case 2: next.s= f.lb-f.lc; next.t= f.lb+f.lc; break;
      case 3: next.s=-f.lb-f.lc; next.t= f.lb-f.lc; break;
    }
  }
  next.gap_m=gap*input.working_length_to_m;
  next.penetration_m=f.penetration*input.working_length_to_m;
  next.distance_m=(gap-f.penetration)*input.working_length_to_m;
  next.selection_distance=gap-f.penetration;
  next.working_length_to_m=input.working_length_to_m;
  next.admissible=f.penetration>0 && next.s<1.5 && next.s>-1.5 && next.t<1.5 && next.t>-1.5;
  next.outside_warning=::fabs(next.s)>1.02 || ::fabs(next.t)>1.02;
  if (!Finite(next.s) || !Finite(next.t) || !Finite(next.gap_m) ||
      !Finite(next.penetration_m) || !Finite(next.distance_m)) return Status::NonfiniteResult;
  output=next;
  return Status::Success;
}
} // namespace search_detail
TL_TIED_PATCH_HD inline Status ProjectCandidate(const WorkingSearchInput& input,
    CandidateProjection& output) noexcept {
  if (!tl::math::Finite(input.master_thickness) || input.master_thickness<=0)
    return Status::InvalidInput;
  return search_detail::ProjectWorkingCandidate(input,output);
}
// Convenience adapter for existing SI callers. Validate topology before the
// conversion, which can merge distinct representable source coordinates.
TL_TIED_PATCH_HD inline Status ProjectCandidate(const SearchInput& input,
    CandidateProjection& output) noexcept {
  namespace v=tl::math::fixed3;
  using tl::math::Finite;
  const bool triangle=input.topology==MasterTopology::TriangleRepeatedThird;
  if ((!triangle && input.topology!=MasterTopology::Quad) ||
      !Finite(input.working_length_to_m) || input.working_length_to_m<=0 ||
      !Finite(input.master_thickness_m) || input.master_thickness_m<=0 ||
      !Finite(input.secondary_shell_thickness_m) || input.secondary_shell_thickness_m<0 ||
      !v::Finite(input.geometry_m.secondary_position)) return Status::InvalidInput;
  const auto& third=input.geometry_m.master_position[2];
  const auto& fourth=input.geometry_m.master_position[3];
  if (triangle && (third.x!=fourth.x || third.y!=fourth.y || third.z!=fourth.z))
    return Status::InvalidInput;
  WorkingSearchInput working;
  working.topology=input.topology;
  working.working_length_to_m=input.working_length_to_m;
  for (unsigned i=0;i<4;++i) {
    if (!v::Finite(input.geometry_m.master_position[i])) return Status::InvalidInput;
    working.geometry.master_position[i]=v::Divide(input.geometry_m.master_position[i],input.working_length_to_m);
    if (!v::Finite(working.geometry.master_position[i])) return Status::NonfiniteResult;
  }
  working.geometry.secondary_position=v::Divide(input.geometry_m.secondary_position,input.working_length_to_m);
  working.master_thickness=input.master_thickness_m/input.working_length_to_m;
  working.secondary_shell_thickness=input.secondary_shell_thickness_m/input.working_length_to_m;
  if (!v::Finite(working.geometry.secondary_position) || !Finite(working.master_thickness) ||
      !Finite(working.secondary_shell_thickness)) return Status::NonfiniteResult;
  return search_detail::ProjectWorkingCandidate(working,output);
}
// Consume caller-defined native candidate order. Exact score ties retain the
// existing match; this function never sorts EIDs or assigns a packing order.
// The two ProjectCandidate overloads admit SI or original working packets.
template<class Input>
TL_TIED_PATCH_HD inline Status ConsiderCandidate(const Input& input,
    std::uint64_t ordered_master,SearchChoice& choice) noexcept {
  const auto& prior=choice.projection;
  if (choice.matched && (!prior.admissible ||
      prior.working_length_to_m!=input.working_length_to_m ||
      !tl::math::Finite(prior.selection_distance) || prior.selection_distance<0 ||
      !tl::math::Finite(prior.s) || !tl::math::Finite(prior.t))) return Status::InvalidInput;
  CandidateProjection candidate;
  const auto status=ProjectCandidate(input,candidate);
  if (status!=Status::Success) return status;
  if (!candidate.admissible) return Status::Success;
  if (!choice.matched || candidate.selection_distance<prior.selection_distance ||
      (candidate.selection_distance==prior.selection_distance &&
       ::fmax(::fabs(candidate.s),::fabs(candidate.t))<::fmax(::fabs(prior.s),::fabs(prior.t))))
    choice={true,ordered_master,candidate};
  return Status::Success;
}
} // namespace tl::constraints::tied_shell
