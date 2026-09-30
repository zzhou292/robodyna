// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type25BatchArena.h"
#include "../ShellBatchFields.h"

namespace tl::fea::type25::batch_detail {
// Source-order reductions of native scalar channels and the accepted RHS cache.
// The latter is a discrete nodal kick/drift observation, not constitutive energy.
TL_SURFACE_HD inline bool Measure(const DeviceModel& model,const Slab& accepted,const Slab& trial,
                                 const NodalPreparedView& view,BatchDiagnostics& d) {
  namespace f=shell_batch_fields;
  d.element_count=model.config.element_count;d.minimum_native_dt=trial.element[0].critical_dt_s;
  for(std::size_t e=0;e<model.config.element_count;++e) {
    const auto& old=accepted.element[e];const auto& now=trial.element[e];
    if(now.history.active)++d.active_count;
    if(old.history.active&&!now.history.active)++d.newly_failed_count;
    for(unsigned c=0;c<4;++c) {
      d.internal_work_J[c]+=now.history.internal_work_J[c];
      d.internal_work_increment_J[c]+=now.history.internal_work_J[c]-old.history.internal_work_J[c];
    }
    if(now.critical_dt_s<d.minimum_native_dt)d.minimum_native_dt=now.critical_dt_s;
    for(unsigned i=0;i<2;++i) {
      const auto n=model.elements[e].nodes[i];const auto& rhs=old.endpoints[i];
      const auto v0=f::ReadVector(view.base_kinematics.velocity_xyz,n),v1=f::ReadVector(view.kinematics.velocity_xyz,n);
      const auto w0=f::ReadVector(view.base_kinematics.angular_velocity_xyz,n),w1=f::ReadVector(view.kinematics.angular_velocity_xyz,n);
      const auto dx=f::Difference(f::ReadVector(view.kinematics.position_xyz,n),f::ReadVector(view.base_kinematics.position_xyz,n));
      const double h=model.config.owner.fixed_dt;const Vec3 rotation{h*w1.x,h*w1.y,h*w1.z};
      // Native endpoint cache is already the nodal RHS (+1); shell internal
      // caches have the opposite convention. Never negate this contributor.
      d.internal_kick_work+=view.kick_dt*(f::Dot(rhs.force_N,f::Mean(v0,v1))+f::Dot(rhs.couple_Nm,f::Mean(w0,w1)));
      d.internal_drift_work+=f::Dot(rhs.force_N,dx)+f::Dot(rhs.couple_Nm,rotation);
    }
  }
  for(unsigned c=0;c<4;++c)if(!tl::math::Finite(d.internal_work_J[c])||!tl::math::Finite(d.internal_work_increment_J[c]))return false;
  return tl::math::Finite(d.internal_kick_work)&&tl::math::Finite(d.internal_drift_work)&&
    tl::math::Finite(d.minimum_native_dt)&&d.minimum_native_dt>0;
}
} // namespace tl::fea::type25::batch_detail
