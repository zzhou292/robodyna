// SPDX-License-Identifier: AGPL-3.0-or-later
// RGBODFP/RGBODV arithmetic adapted from OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "NodalRigidFrameStep.h"

#if defined(__CUDACC__)
#define TL_RIGID_STEP_HD __host__ __device__
#else
#define TL_RIGID_STEP_HD
#endif

namespace tl::fea::rigid {
enum class StepStatus { Success,InvalidInput,RotationLimit,NonfiniteResult };
struct StepDurations {
  double previous_drift_dt=0,kick_dt=0,drift_dt=0;
};
struct PrimaryStepInput {
  PrincipalFrame previous_frame{};
  Vec3 center{},velocity{},omega{};
  double mass=0;
  Wrench applied{}; // Complete member force+couple wrench, assembled once.
  StepDurations durations{};
};
struct PrimaryStepTrial {
  PrincipalFrame force_frame{};
  Vec3 saved_body_omega{},acceleration{},angular_acceleration{};
  Vec3 center{},velocity{},omega{};
};
struct MemberStepInput {
  Vec3 position{},velocity{},omega{},force{},couple{};
  double mass=0,inertia=0; // Authoritative native scalar J, not a partition sum.
};
struct MemberStepTrial {
  Vec3 acceleration{},angular_acceleration{},position{},velocity{},omega{};
  Vec3 reaction_force{},reaction_couple{};
};
namespace step_detail {
TL_RIGID_STEP_HD inline bool Durations(StepDurations d) {
  return tl::math::Finite(d.previous_drift_dt)&&d.previous_drift_dt>=0&&
    tl::math::Finite(d.kick_dt)&&d.kick_dt>0&&tl::math::Finite(d.drift_dt)&&d.drift_dt>0;
}
TL_RIGID_STEP_HD inline bool Finite(const PrimaryStepTrial& t) {
  return detail::Orthonormal(t.force_frame.axes)&&detail::Positive(t.force_frame.inertia)&&
    detail::Finite(t.saved_body_omega)&&detail::Finite(t.acceleration)&&detail::Finite(t.angular_acceleration)&&
    detail::Finite(t.center)&&detail::Finite(t.velocity)&&detail::Finite(t.omega);
}
TL_RIGID_STEP_HD inline bool Finite(const MemberStepTrial& t) {
  return detail::Finite(t.acceleration)&&detail::Finite(t.angular_acceleration)&&
    detail::Finite(t.position)&&detail::Finite(t.velocity)&&detail::Finite(t.omega)&&
    detail::Finite(t.reaction_force)&&detail::Finite(t.reaction_couple);
}
} // namespace step_detail

// Pure packet for the >2-member, free explicit anisotropic branch. The caller
// owns the duration/phase contract: this function neither chooses a timestep nor
// asserts that a (0,h/2,h) packet is the donor engine's actual startup. The native
// old-spin guard is retained; case-specific new-spin limits belong to admission.
// Each output is published by value only after all arithmetic succeeds.
TL_RIGID_STEP_HD inline StepStatus EvaluatePrimaryStep(const PrimaryStepInput& in,
    PrimaryStepTrial& output) {
  if(!step_detail::Durations(in.durations)||!detail::Orthonormal(in.previous_frame.axes)||
      !detail::Positive(in.previous_frame.inertia)||!detail::Finite(in.center)||
      !detail::Finite(in.velocity)||!detail::Finite(in.omega)||!detail::Finite(in.applied.force)||
      !detail::Finite(in.applied.couple)||!tl::math::Finite(in.mass)||in.mass<=0) return StepStatus::InvalidInput;
  const double square=in.durations.drift_dt*in.durations.drift_dt*
    (in.omega.x*in.omega.x+in.omega.y*in.omega.y+in.omega.z*in.omega.z);
  if(!tl::math::Finite(square)) return StepStatus::NonfiniteResult;
  if(square>1) return StepStatus::RotationLimit;
  PrimaryStepTrial next;
  next.saved_body_omega=detail::ToLocal(in.previous_frame.axes,in.omega);
  next.force_frame.inertia=in.previous_frame.inertia;
  const auto rotated=RotatePrincipalFrame(in.previous_frame.axes,next.saved_body_omega,
      in.durations.previous_drift_dt,next.force_frame.axes);
  if(rotated!=MathStatus::Success) return StepStatus::NonfiniteResult;
  const auto torque=detail::ToLocal(next.force_frame.axes,in.applied.couple);
  const auto j=next.force_frame.inertia,w=next.saved_body_omega;
  const Vec3 local{(torque.x+(j.y-j.z)*w.y*w.z)/j.x,
                   (torque.y+(j.z-j.x)*w.z*w.x)/j.y,
                   (torque.z+(j.x-j.y)*w.x*w.y)/j.z};
  // Native primary proxy J multiply/divide cancels; it is not extra inertia.
  next.angular_acceleration=detail::ToWorld(next.force_frame.axes,local);
  next.acceleration={in.applied.force.x/in.mass,in.applied.force.y/in.mass,in.applied.force.z/in.mass};
  const double kick=in.durations.kick_dt,drift=in.durations.drift_dt;
  next.velocity={in.velocity.x+kick*next.acceleration.x,in.velocity.y+kick*next.acceleration.y,
                 in.velocity.z+kick*next.acceleration.z};
  next.omega={in.omega.x+next.angular_acceleration.x*kick,in.omega.y+next.angular_acceleration.y*kick,
              in.omega.z+next.angular_acceleration.z*kick};
  next.center={in.center.x+drift*next.velocity.x,in.center.y+drift*next.velocity.y,in.center.z+drift*next.velocity.z};
  if(!step_detail::Finite(next)) return StepStatus::NonfiniteResult;
  output=next; return StepStatus::Success;
}

// Compose with the corresponding input/trial returned by EvaluatePrimaryStep.
// No identity/transaction authority is conveyed by these plain math packets.
// The acceleration then common-kick order retains native cancellation/roundoff;
// replacing it by an exact geometric projection would change the recurrence.
TL_RIGID_STEP_HD inline StepStatus EvaluateMemberStep(const PrimaryStepInput& body,
    const PrimaryStepTrial& primary,const MemberStepInput& in,MemberStepTrial& output) {
  if(!step_detail::Durations(body.durations)||!detail::Finite(body.center)||!detail::Finite(body.velocity)||
      !step_detail::Finite(primary)||!detail::Finite(in.position)||!detail::Finite(in.velocity)||
      !detail::Finite(in.omega)||!detail::Finite(in.force)||!detail::Finite(in.couple)||
      !tl::math::Finite(in.mass)||in.mass<=0||!tl::math::Finite(in.inertia)||in.inertia<=0)
    return StepStatus::InvalidInput;
  MemberStepTrial next;
  const double kick=body.durations.kick_dt,drift=body.durations.drift_dt,usdt=1/kick;
  const auto w=primary.omega;
  next.angular_acceleration={(w.x-in.omega.x)*usdt,(w.y-in.omega.y)*usdt,(w.z-in.omega.z)*usdt};
  const auto arm=detail::Subtract(in.position,body.center),g=detail::Cross(w,arm);
  next.acceleration={primary.acceleration.x+(body.velocity.x+g.x+.5*drift*(w.y*g.z-w.z*g.y)-in.velocity.x)*usdt,
      primary.acceleration.y+(body.velocity.y+g.y+.5*drift*(w.z*g.x-w.x*g.z)-in.velocity.y)*usdt,
      primary.acceleration.z+(body.velocity.z+g.z+.5*drift*(w.x*g.y-w.y*g.x)-in.velocity.z)*usdt};
  next.reaction_force={in.mass*next.acceleration.x-in.force.x,in.mass*next.acceleration.y-in.force.y,
                        in.mass*next.acceleration.z-in.force.z};
  next.reaction_couple={in.inertia*next.angular_acceleration.x-in.couple.x,in.inertia*next.angular_acceleration.y-in.couple.y,
                         in.inertia*next.angular_acceleration.z-in.couple.z};
  next.velocity={in.velocity.x+kick*next.acceleration.x,in.velocity.y+kick*next.acceleration.y,in.velocity.z+kick*next.acceleration.z};
  next.omega={in.omega.x+kick*next.angular_acceleration.x,in.omega.y+kick*next.angular_acceleration.y,in.omega.z+kick*next.angular_acceleration.z};
  next.position={in.position.x+drift*next.velocity.x,in.position.y+drift*next.velocity.y,in.position.z+drift*next.velocity.z};
  if(!tl::math::Finite(usdt)||!detail::Finite(arm)||!detail::Finite(g)||!step_detail::Finite(next))
    return StepStatus::NonfiniteResult;
  output=next; return StepStatus::Success;
}
} // namespace tl::fea::rigid
#undef TL_RIGID_STEP_HD
