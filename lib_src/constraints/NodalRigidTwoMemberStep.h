// SPDX-License-Identifier: AGPL-3.0-or-later
// RGBODV two-member arithmetic adapted from OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "NodalRigidFiniteVelocity.h"
#if defined(__CUDACC__)
#define TL_RIGID_TWO_HD __host__ __device__
#else
#define TL_RIGID_TWO_HD
#endif
namespace tl::fea::rigid {
TL_RIGID_TWO_HD inline StepStatus EvaluateTwoMemberPrimaryStep(const PrimaryStepInput& in,
    PrimaryStepTrial& output) { return EvaluatePrimaryStepPacket<true>(in,output); }
// Exactly two physical secondary members, in the model's native source units.
// The primary packet and the owner's old/new spin limits remain shared; those
// limits are our bounded qualification domain, not a donor two-member rejection.
// This is a value function: identity/phase and whole-trial rollback belong to owner.
TL_RIGID_TWO_HD inline StepStatus EvaluateTwoMemberStep(const PrimaryStepInput& body,
    const PrimaryStepTrial& primary,const MemberStepInput& in,double length_to_m,
    MemberStepTrial& output) {
  const double threshold=(1e-8*length_to_m)*length_to_m;
  if(!step_detail::Durations(body.durations)||!detail::Finite(body.center)||!detail::Finite(body.velocity)||
      !step_detail::Finite(primary)||!detail::Finite(in.position)||!detail::Finite(in.velocity)||
      !detail::Finite(in.omega)||!detail::Finite(in.force)||!detail::Finite(in.couple)||
      !tl::math::Finite(in.mass)||in.mass<=0||!tl::math::Finite(in.inertia)||in.inertia<=0||
      !tl::math::Finite(length_to_m)||length_to_m<=0||!tl::math::Finite(threshold)||threshold<=0)
    return StepStatus::InvalidInput;
  MemberStepTrial next;
  const double kick=body.durations.kick_dt,drift=body.durations.drift_dt,usdt=1/kick;
  const auto w=primary.omega,arm=detail::Subtract(in.position,body.center);
  const auto g=two_member_detail::FiniteVelocity(w,arm,kick,length_to_m);
  next.angular_acceleration={(w.x-in.omega.x)*usdt,(w.y-in.omega.y)*usdt,(w.z-in.omega.z)*usdt};
  next.acceleration={primary.acceleration.x+(body.velocity.x-in.velocity.x)*usdt,
    primary.acceleration.y+(body.velocity.y-in.velocity.y)*usdt,
    primary.acceleration.z+(body.velocity.z-in.velocity.z)*usdt};
  next.acceleration.x=next.acceleration.x+(g.x+.5*drift*(w.y*g.z-w.z*g.y))*usdt;
  next.acceleration.y=next.acceleration.y+(g.y+.5*drift*(w.z*g.x-w.x*g.z))*usdt;
  next.acceleration.z=next.acceleration.z+(g.z+.5*drift*(w.x*g.y-w.y*g.x))*usdt;
  step_detail::CompleteMember(in,kick,drift,next);
  if(!tl::math::Finite(usdt)||!detail::Finite(arm)||!detail::Finite(g)||!step_detail::Finite(next))
    return StepStatus::NonfiniteResult;
  output=next; return StepStatus::Success;
}
} // namespace tl::fea::rigid
#undef TL_RIGID_TWO_HD
