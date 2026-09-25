// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "CppReplay.h"
#include "lib_src/elements/qeph/QephForceProjectionUnits.h"
namespace qeph_projection_test {
struct ProfileInput {RateInput rate;ForceInput force;double length_m=1;};
struct ProfileResult {q::Status rate_status=q::Status::kInvalidInput,force_status=q::Status::kInvalidInput;RateResult rate;ForceResult force;double metric_length=0;};
TL_QEPH_HD inline ProfileResult EvaluateProfile(const ProfileInput& in) {
  ProfileResult out;auto work=Work(in.rate.geometry);work.v13=Vector(in.rate.v13);work.v24=Vector(in.rate.v24);work.vhi=Vector(in.rate.vhi);
  q::PrescribedInterval interval;
  for(unsigned i=0;i<4;++i){interval.omega_midpoint[i]=Vector(in.rate.world_omega[i]);
    work.values.projected_omega[2*i]=in.rate.rlxyz[i][0];work.values.projected_omega[2*i+1]=in.rate.rlxyz[i][1];}
  out.rate_status=q::detail::ProjectWarpedRatesInWorkingLength(interval,in.length_m,work);
  if(out.rate_status!=q::Status::kSuccess)return out;
  out.rate=RateResultOf(work);out.metric_length=work.values.projection_metric.working_length_m;
  q::detail::LocalForceWork local;for(unsigned i=0;i<4;++i){local.force[i]=Vector(in.force.vf[i]);
    local.couple[i][0]=in.force.vm[i][0];local.couple[i][1]=in.force.vm[i][1];}
  q::Vec3 force[4],couple[4];out.force_status=q::detail::ProjectForcesInWorkingLength(work,local,in.length_m,force,couple);
  if(out.force_status!=q::Status::kSuccess)return out;
  for(unsigned i=0;i<4;++i){out.force.force[i]=Vector(force[i]);out.force.couple[i]=Vector(couple[i]);}return out;
}
inline ProfileInput Profile(const CapturedRow& row,double length) {
  ProfileInput out;out.length_m=length;out.rate=Scale(row.rate_entry,.001);
  out.force=Scale(row.force_entry,.001,{});return out;
}
} // namespace qeph_projection_test
