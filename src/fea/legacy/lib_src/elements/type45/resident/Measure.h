// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"
#include "ResultChecks.h"
#include "../../ShellBatchFields.h"

namespace tl::fea::type45::resident_detail {
TL_TYPE45_HD inline bool Measure(Storage& s,unsigned accepted,unsigned trial,
    const NodalPreparedView* view,BatchDiagnostics& d) {
  namespace fields=shell_batch_fields;
  d.joint_count=s.count;
  for(std::size_t j=0;j<s.count;++j) {
    if(s.status[j]!=Status::Success ||
        !ValidResult(s.joints[j],s.slab[trial][j],d.time,d.epoch,s.config.owner.fixed_dt)) {
      s.control.status=s.status[j]==Status::Success?BatchStatus::NonfiniteResult:BatchStatus::JointFailure;
      s.control.joint=j;s.control.joint_status=s.status[j];return false;
    }
    if(!view) continue;
    const auto& old=s.slab[accepted][j].cache;
    const auto& now=s.slab[trial][j].cache;
    d.native_internal_work_increment_j+=now.diagnostics.internal_work_increment_j;
    for(unsigned e=0;e<2;++e) {
      const auto node=s.joints[j].domain_nodes[e];
      const auto& rhs=old.endpoint[e];
      const auto v0=fields::ReadVector(view->base_kinematics.velocity_xyz,node);
      const auto v1=fields::ReadVector(view->kinematics.velocity_xyz,node);
      const auto w0=fields::ReadVector(view->base_kinematics.angular_velocity_xyz,node);
      const auto w1=fields::ReadVector(view->kinematics.angular_velocity_xyz,node);
      const auto dx=fields::Difference(fields::ReadVector(view->kinematics.position_xyz,node),
          fields::ReadVector(view->base_kinematics.position_xyz,node));
      const double h=s.config.owner.fixed_dt;
      const Vec3 rotation{h*w1.x,h*w1.y,h*w1.z};
      // Contributor RHS work, using the same owner kick/drift convention as
      // TYPE13. This is distinct from the native joint's signed internal work.
      d.internal_kick_work_j+=view->kick_dt*(fields::Dot(rhs.force_n,fields::Mean(v0,v1))+
          fields::Dot(rhs.couple_nm,fields::Mean(w0,w1)));
      d.internal_drift_work_j+=fields::Dot(rhs.force_n,dx)+fields::Dot(rhs.couple_nm,rotation);
    }
  }
  if(!detail::Finite(d.native_internal_work_increment_j) || !detail::Finite(d.internal_kick_work_j) ||
      !detail::Finite(d.internal_drift_work_j)) {
    s.control.status=BatchStatus::NonfiniteResult;return false;
  }
  return true;
}
} // namespace tl::fea::type45::resident_detail
