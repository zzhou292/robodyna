// SPDX-License-Identifier: AGPL-3.0-or-later
// S8FOR_DISTOR, OpenRadioss a62b27e6 (C) 2026 Siemens.
#pragma once
#include "Damping.h"
#include "CenterForce.h"
#include "CornerForce.h"
namespace tl::fea::solid_common::distortion::native {
TL_BRICK_HD inline Status EvaluateForce(const Parameters& parameters,const ForceInput& input,
    bool native_batch_damping_enabled,ForceResult& output) noexcept {
  DampingActivity activity;
  const Status status=ClassifyDamping(parameters,input,activity);
  if(status!=Status::Success)return status;
  if(activity.triggers_native_batch&&!native_batch_damping_enabled)return Status::InvalidInput;
  // The family material/geometry caller owns Jacobian and volume admission.
  // S8FOR_DISTOR itself retains its native degenerate-face behavior.
  detail::Work work;work.stiffness=parameters.control_stiffness;
  work.energy=input.distortion_energy;
  const Vec3 mean=detail::Mean8(input.velocity),center=detail::Mean8(input.position);
  const bool damping=native_batch_damping_enabled&&activity.flag>0;
  if(damping)detail::Damping(parameters,input,mean,work);
  // SFOR_VISN8 always returns IFCTL=1; geometry executes regardless of damping.
  const double gapmin=(2.0/10.0)*(1.0/100.0),gapmax=5.0*gapmin;
  const double penmin=gapmin*parameters.length,penref=gapmax*parameters.length;
  const double margin=2.0*gapmax*parameters.length;
  const unsigned center_faces[6][4]{{1,0,3,2},{0,1,5,4},{1,2,6,5},{0,4,7,3},{3,7,6,2},{4,5,6,7}};
  for(const auto& face:center_faces)
    if(!detail::CenterForce(parameters,input,center,mean,face,activity.flag,penmin,penref,margin,work))return Status::NonfiniteResult;
  const unsigned corner_faces[6][4]{{0,3,2,1},{0,1,5,4},{1,2,6,5},{0,4,7,3},{3,7,6,2},{4,5,6,7}};
  const unsigned opposite[6][4]{{4,7,6,5},{3,2,6,7},{0,3,7,4},{1,5,6,2},{0,4,5,1},{0,1,2,3}};
  for(unsigned face=0;face<6;++face)
    if(!detail::CornerForces(parameters,input,corner_faces[face],opposite[face],activity.flag,penmin,penref,margin,work))return Status::NonfiniteResult;
  if(!detail::FiniteWork(work))return Status::NonfiniteResult;
  ForceResult next;
  const Vec3 center_force{(1.0/8.0)*work.center_force.x,(1.0/8.0)*work.center_force.y,(1.0/8.0)*work.center_force.z};
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)
    SetComponent(next.force[n],k,Component(input.incoming_force[n],k)+Component(work.force[n],k)+Component(center_force,k));
  next.raw_stiffness=input.raw_stiffness;
  if(work.stiffness>parameters.control_stiffness)next.raw_stiffness=::fmax(input.raw_stiffness,work.stiffness);
  next.distortion_energy=work.energy;next.distortion_work_increment=work.increment;
  next.damping_applied=damping?1:0;next.center_contacts=work.center_contacts;next.corner_contacts=work.corner_contacts;
  for(const auto& f:next.force)if(!Finite(f))return Status::NonfiniteResult;
  output=next;
  return Status::Success;
}
} // namespace tl::fea::solid_common::distortion::native
