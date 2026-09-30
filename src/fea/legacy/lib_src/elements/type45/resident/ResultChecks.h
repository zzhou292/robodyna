// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "State.h"

namespace tl::fea::type45::resident_detail {
TL_TYPE45_HD inline bool MatchesJoint(const Joint& joint,const Reference& reference) {
  using detail::Same;
  if(!reference.ready() || !Same(joint.property,reference.property())) return false;
  const auto& source=reference.geometry();
  if(source.source_joint_id!=joint.geometry.source_joint_id) return false;
  for(unsigned i=0;i<3;++i)
    if(source.source_node_id[i]!=joint.geometry.source_node_id[i] ||
        !Same(source.position_m[i],joint.geometry.position_m[i])) return false;
  for(unsigned i=0;i<2;++i)
    if(!Same(reference.damping(i).mass_kg,joint.damping[i].mass_kg) ||
        !Same(reference.damping(i).mean_principal_inertia_kg_m2,
              joint.damping[i].mean_principal_inertia_kg_m2)) return false;
  return true;
}
TL_TYPE45_HD inline bool ValidResult(const Joint& joint,const State& state,
    double time,std::uint64_t epoch,double fixed_dt) {
  Result value;
  if(!Export(joint,state,value)) return false;
  if(epoch) {
    return value.automatic_stiffness_initialized && value.stamp.sample_index==epoch &&
        detail::Same(value.stamp.time_s,time) &&
        detail::Same(value.context.target_dt_s,fixed_dt) && MatchesJoint(joint,state.history.reference());
  }
  if(time!=0 || value.automatic_stiffness_initialized) return false;
  State virgin;
  if(InitializeState(joint,virgin)!=Status::Success) return false;
  for(unsigned i=0;i<9;++i)
    if(!detail::Same(state.initial_frame.v[i],virgin.initial_frame.v[i])) return false;
  const auto& d=value.diagnostics;
  if(!detail::Same(d.local_separation_m,virgin.cache.diagnostics.local_separation_m) ||
      !detail::Same(d.local_velocity_m_s,Vec3{}) || !detail::Same(d.relative_rate_rad_s,Vec3{}) ||
      d.harmonic_mass_kg!=0 || d.harmonic_inertia_kg_m2!=0 || d.maximum_stiffness_n_m!=0 ||
      d.maximum_rotational_stiffness_nm!=0 || d.damping_n_s_m!=0 ||
      d.rotational_damping_nm_s!=0 || d.internal_work_increment_j!=0) return false;
  for(const auto& e:value.endpoint)
    if(!detail::Same(e.force_n,Vec3{}) || !detail::Same(e.couple_nm,Vec3{}) ||
        e.translational_stiffness_n_m!=0 || e.rotational_stiffness_nm!=0) return false;
  return true;
}
} // namespace tl::fea::type45::resident_detail
