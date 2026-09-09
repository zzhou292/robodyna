// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected C3INMAS/C3DERII/SPMD_MSIN, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3Geometry.h"

namespace tl::fea::t3 {
// Three physical nodes; centered IGTYP1/ISMSTR-1. Preserves caller output on
// every failure, including input aliasing the old output. No mass scaling,
// angle normalization, clamp, stiffness, material history or clock.
TL_T3_HD inline Status InitializeReference(const ReferenceInput& input,ReferenceData& output) {
  if(!detail::Positive(input.density)||!detail::Positive(input.thickness)||
     !detail::Positive(input.young_modulus)||!tl::math::Finite(input.poisson_ratio)||
     input.poisson_ratio<0||input.poisson_ratio>=.5||!detail::Coordinates(input.position)) return Status::kInvalidInput;
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<i;++j)
    if(input.node_ids[i]==input.node_ids[j]) return Status::kInvalidInput;
  double longest=0;
  if(!detail::SupportedGeometry(input.position,longest)) return Status::kUnsupportedGeometry;
  detail::FrameWork w; const auto status=detail::NativeFrame(input.position,w);
  if(status!=Status::kSuccess) return status;
  ReferenceData next; next.input=input; next.frame=w.frame; next.area=w.area;
  const double x2=w.length21,x3=detail::Project(w.frame,0,w.edge31),y3=detail::Project(w.frame,1,w.edge31);
  if(!tl::math::Finite(x3)||!detail::Positive(y3)) return Status::kNonfiniteResult;
  next.local_position[1]={x2,0,0}; next.local_position[2]={x3,y3,0};
  const double a2=x2*x2,b2=(x2-x3)*(x2-x3)+y3*y3,bb=::sqrt(b2),c2=x3*x3+y3*y3,cc=::sqrt(c2);
  next.angle_cosine[0]=(a2+c2-b2)/(2*x2*cc);
  next.angle_cosine[1]=(a2+b2-c2)/(2*x2*bb);
  next.angle_cosine[2]=(b2+c2-a2)/(2*bb*cc);
  for(double c:next.angle_cosine)
    if(!tl::math::Finite(c)||::fabs(c)>1-detail::AcosMargin-detail::GuardBand) return Status::kUnsupportedGeometry;
  // PI is native ATAN2(ZERO,-ONE); no unsuffixed float literal enters a
  // noninteger coefficient. All selected native addition/product order stays.
  const double pi=::atan2(0.,-1.);
  for(unsigned i=0;i<3;++i) next.angle_weight[i]=::acos(next.angle_cosine[i])/pi;
  const double em=input.density*input.thickness*next.area;
  const double xi=em*(next.area/(9./2)+input.thickness*input.thickness*(1./12));
  const double physical=em*input.thickness*input.thickness*(1./12),added=em*(next.area/(9./2));
  next.element_mass=em; next.element_isotropic_inertia=xi;
  next.element_physical_inertia=physical; next.element_added_inertia=added;
  if(!detail::Positive(em)||!detail::Positive(xi)||!detail::Positive(physical)||!detail::Positive(added)) return Status::kNonfiniteResult;
  for(unsigned i=0;i<3;++i) {
    const double p=next.angle_weight[i];
    next.nodal_mass[i]=0+em*p; next.isotropic_inertia[i]=0+xi*p;
    next.physical_inertia[i]=physical*p; next.added_inertia[i]=added*p;
    if(!detail::Positive(p)||!detail::Positive(next.nodal_mass[i])||!detail::Positive(next.isotropic_inertia[i])||
       !detail::Positive(next.physical_inertia[i])||!detail::Positive(next.added_inertia[i])) return Status::kNonfiniteResult;
  }
  next.characteristic_length=detail::CharacteristicLength(x2,x3,y3,next.area);
  if(!detail::Positive(next.characteristic_length)) return Status::kNonfiniteResult;
  // ISMSTR=-1 startup derivative slots deliberately remain zero.
  next.prepared=true; output=next; return Status::kSuccess;
}
} // namespace tl::fea::t3
