// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "RigidNormalResponseBounds.h"
#include "lib_src/constraints/NodalRigidGroupState.h"
#include "lib_src/constraints/NodalRigidFrameStep.h"

namespace tlfea::contact {
struct RigidContactBody {
  double mass=0;
  tl::fea::rigid::PrincipalFrame current_frame;
  tl::math::Vec3 center{};
};
struct RigidNormalResponse {
  double inverse_effective_mass=0,inverse_upper=0; // kg^-1
  bool valid=false;
};
// Same frame preparation as EvaluatePrimaryStepPacket, without force/kick/drift.
// For the existing fixed-step owner, previous_drift_dt is zero at epoch zero
// and fixed_dt thereafter. Caller authenticates the accepted snapshot/stamp and
// immutable source body coefficients; the plain value supplies no authority.
TL_SURFACE_HD inline Status PrepareRigidContactBodyFromAccepted(
    const tl::fea::NodalRigidGroupState& accepted,double mass,tl::math::Vec3 inertia,
    double previous_drift_dt,RigidContactBody& output) {
  namespace rigid=tl::fea::rigid;
  if(!rigid::ValidGroupState(accepted)||!IsFinite(mass)||mass<=0||
     !rigid::detail::Positive(inertia)||!IsFinite(previous_drift_dt)||previous_drift_dt<0)
    return Status::kInvalidArgument;
  RigidContactBody next{mass,{accepted.principal_axes,inertia},accepted.center};
  const auto saved=rigid::detail::ToLocal(accepted.principal_axes,accepted.omega);
  if(rigid::RotatePrincipalFrame(accepted.principal_axes,saved,previous_drift_dt,
      next.current_frame.axes)!=rigid::MathStatus::Success)return Status::kNonFiniteResult;
  output=next;return Status::kOk;
}
// Frozen CURRENT force-frame response to a unit normal force at a body member.
// No individual member mass/J, gyro tangent, finite-rotation update, owner token
// or temporal phase is inferred. The caller authenticates the actual body and
// frame. Positive aggregate M/principal-J also admits zero-M/J rigid members.
// Pure host/device value operation; failure preserves output.
TL_SURFACE_HD inline Status EvaluateRigidNormalResponse(const RigidContactBody& body,
    tl::math::Vec3 point,tl::math::Vec3 normal,RigidNormalResponse& output) {
  namespace rigid=tl::fea::rigid;
  if(!IsFinite(body.mass)||body.mass<=0||!rigid::detail::Positive(body.current_frame.inertia)||
     !rigid::detail::Orthonormal(body.current_frame.axes)||!rigid::detail::Finite(body.center)||
     !rigid::detail::Finite(point)||!rigid::detail::Finite(normal)||
     ::fabs(mass_detail::Norm({normal.x,normal.y,normal.z})-1)>1e-12)
    return Status::kInvalidArgument;
  const auto arm=rigid::detail::Subtract(point,body.center);
  const auto moment=rigid::detail::ToLocal(body.current_frame.axes,rigid::detail::Cross(arm,normal));
  if(!rigid::detail::Finite(arm)||!rigid::detail::Finite(moment))return Status::kNonFiniteResult;
  rigid_response_detail::Interval local[3];
  if(!rigid_response_detail::LocalMomentBounds(point,body.center,normal,body.current_frame.axes,local))
    return Status::kNonFiniteResult;
  const auto inertia=body.current_frame.inertia;
  const double coefficients[]{normal.x,normal.y,normal.z,moment.x,moment.y,moment.z};
  const double mass[]{body.mass,body.mass,body.mass,inertia.x,inertia.y,inertia.z};
  RigidNormalResponse next;
  for(unsigned i=0;i<6;++i) {
    const double value=coefficients[i]*coefficients[i]/mass[i];
    const auto interval=i<3?Q4IntegralInterval{coefficients[i],coefficients[i]}:local[i-3];
    double upper=0;
    if(!IsFinite(value)||(coefficients[i]!=0&&value==0)||
       !rigid_response_detail::SquareOver(interval,mass[i],upper)||
       !mass_detail::UpperSum(next.inverse_upper,upper,&next.inverse_upper))
      return Status::kNonFiniteResult;
    next.inverse_effective_mass+=value;
  }
  if(!IsFinite(next.inverse_effective_mass)||next.inverse_effective_mass<=0||
     next.inverse_upper<next.inverse_effective_mass)return Status::kNonFiniteResult;
  next.valid=true;output=next;return Status::kOk;
}
// Sum per-member upper k*J*M^-1*J^T for ONE body. This bounds the largest
// eigenvalue of its frozen symmetric contact operator by its trace. Use max
// across independent bodies/ordinary nodes. This is local step screening,
// not a theorem for nonlinear rigid/CIN/joint/structural recurrence.
TL_SURFACE_HD inline Status AccumulateRigidContactTrace(double stiffness_upper,
    const RigidNormalResponse& response,double& trace) {
  if(!IsFinite(stiffness_upper)||stiffness_upper<0||!IsFinite(trace)||trace<0||
     !response.valid||!IsFinite(response.inverse_effective_mass)||response.inverse_effective_mass<=0||
     !IsFinite(response.inverse_upper)||response.inverse_upper<response.inverse_effective_mass)
    return Status::kInvalidArgument;
  double term=0,next=0;
  if(!mass_detail::UpperProduct(stiffness_upper,response.inverse_upper,&term)||
     !mass_detail::UpperSum(trace,term,&next))return Status::kNonFiniteResult;
  trace=next;return Status::kOk;
}
} // namespace tlfea::contact
