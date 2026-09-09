// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected T3 engine composition, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3Rates.h"
namespace tl::fea::t3::detail {
// Producer record validation, not authentication of arbitrary writable memory.
// A resident owner must bind the immutable startup result before consuming it.
TL_T3_HD inline bool SaneReference(const ReferenceData& r) {
  if(!r.prepared||!Coordinates(r.input.position)||!Positive(r.input.density)||
     !Positive(r.input.thickness)||!Positive(r.input.young_modulus)||
     !tl::math::Finite(r.input.poisson_ratio)||r.input.poisson_ratio<0||r.input.poisson_ratio>=.5||
     !Positive(r.area)||!Proper(r.frame)||!Positive(r.characteristic_length)||
     !Positive(r.element_mass)||!Positive(r.element_isotropic_inertia)||
     !Positive(r.element_physical_inertia)||!Positive(r.element_added_inertia)) return false;
  for(unsigned i=0;i<3;++i) {
    for(unsigned j=0;j<i;++j) if(r.input.node_ids[i]==r.input.node_ids[j]) return false;
    if(!Finite(r.local_position[i])||!tl::math::Finite(r.angle_cosine[i])||
       ::fabs(r.angle_cosine[i])>1-AcosMargin-GuardBand||!Positive(r.angle_weight[i])||
       !Positive(r.nodal_mass[i])||!Positive(r.physical_inertia[i])||
       !Positive(r.added_inertia[i])||!Positive(r.isotropic_inertia[i])||r.startup_derivative[i]!=0) return false;
  }
  return true;
}
TL_T3_HD inline Status CheckPrescribed(const ReferenceData& reference,const PrescribedInterval& in,double& longest) {
  if(!SaneReference(reference)) return Status::kInvalidReference;
  if(!in.sample_index||!tl::math::Finite(in.base_time)||in.base_time<0||
     !Positive(in.dt)||.25*in.dt<=0||!Coordinates(in.position)) return Status::kInvalidInput;
  const double end=in.base_time+in.dt,mid=in.base_time+.5*in.dt;
  if(!tl::math::Finite(end)||!(in.base_time<mid&&mid<end)) return Status::kInvalidInput;
  for(unsigned n=0;n<3;++n) if(!Finite(in.velocity[n])||!Finite(in.angular_velocity[n])) return Status::kInvalidInput;
  if(!SupportedGeometry(in.position,longest)) return Status::kUnsupportedGeometry;
  return Status::kSuccess;
}
} // namespace tl::fea::t3::detail
namespace tl::fea::t3 {
// Prescribed endpoint positions/midpoint velocities only. Complete selected
// IRESP2 gather, IFRAM_OLD1 current frame, ISMSTR-1 derivatives and ISH3N2
// quarter-step rates. No native material/history, integration or force cache.
// Both reference and caller output remain unchanged on any failure.
TL_T3_HD inline Status EvaluatePrescribed(const ReferenceData& reference,const PrescribedInterval& in,Kinematics& output) {
  double longest=0; auto status=detail::CheckPrescribed(reference,in,longest);
  if(status!=Status::kSuccess) return status;
  detail::GeometryWork work;
  status=detail::CurrentGeometry(in.position,longest,work); if(status!=Status::kSuccess) return status;
  status=detail::EvaluateRates(in,work); if(status!=Status::kSuccess) return status;
  auto& k=work.kinematics;
  k.base_time=in.base_time; k.position_time=in.base_time+in.dt; k.velocity_time=in.base_time+.5*in.dt;
  k.dt=in.dt; k.sample_index=in.sample_index; k.valid=true;
  output=k; return Status::kSuccess;
}
} // namespace tl::fea::t3
