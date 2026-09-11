// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Batch.h"
#include "Virgin.h"
#include "../Type45Force.h"

namespace tl::fea::type45::resident_detail {
struct Cache {
  EndpointResult endpoint[2]{};
  Diagnostics diagnostics;
};
struct State {
  History history; // Unbound until the first authentic main context is supplied.
  Matrix3 initial_frame;
  Cache cache;
};
TL_TYPE45_HD inline Status InitializeState(const Joint& joint,State& output) {
  VirginCache virgin;
  const auto status=PrepareVirgin(joint.property,joint.geometry,virgin);
  if(status!=Status::Success) return status;
  State next;
  next.initial_frame=virgin.history.frame;
  next.cache.diagnostics.local_separation_m=detail::ToLocal(next.initial_frame,
      detail::Subtract(joint.geometry.position_m[1],joint.geometry.position_m[0]));
  output=next;
  return Status::Success;
}
TL_TYPE45_HD inline Status UpdateState(const Joint& joint,const State& accepted,
    const AutomaticStiffnessContext* initial_context,const Interval& interval,State& output) {
  History prior=accepted.history;
  if(!prior.ready()) {
    if(!initial_context || !detail::Same(initial_context->target_dt_s,interval.dt_s))
      return Status::InvalidContext;
    Reference reference;
    auto status=Reference::Prepare(joint.property,joint.geometry,joint.damping,*initial_context,reference);
    if(status!=Status::Success) return status;
    status=History::Initialize(reference,prior);
    if(status!=Status::Success) return status;
  } else if(initial_context) return Status::InvalidContext; // Accepted automatic K cannot be replaced.
  if(!detail::Same(prior.reference().property(),joint.property)) return Status::ReferenceMismatch;
  const auto& source=prior.reference().geometry();
  if(source.source_joint_id!=joint.geometry.source_joint_id) return Status::ReferenceMismatch;
  for(unsigned i=0;i<3;++i)
    if(source.source_node_id[i]!=joint.geometry.source_node_id[i] ||
        !detail::Same(source.position_m[i],joint.geometry.position_m[i])) return Status::ReferenceMismatch;
  for(unsigned i=0;i<2;++i)
    if(!detail::Same(prior.reference().damping(i).mass_kg,joint.damping[i].mass_kg) ||
        !detail::Same(prior.reference().damping(i).mean_principal_inertia_kg_m2,
                      joint.damping[i].mean_principal_inertia_kg_m2)) return Status::ReferenceMismatch;
  Evaluation evaluated;
  const auto status=Evaluate(prior.reference(),prior,interval,evaluated);
  if(status!=Status::Success) return status;
  State next=accepted;
  next.history=evaluated.history;
  for(unsigned i=0;i<2;++i) next.cache.endpoint[i]=evaluated.endpoint[i];
  next.cache.diagnostics=evaluated.diagnostics;
  output=next;
  return Status::Success;
}
TL_TYPE45_HD inline bool Export(const Joint& joint,const State& state,Result& output) {
  using namespace detail;
  Result next;
  next.source_joint_id=joint.geometry.source_joint_id;
  next.automatic_stiffness_initialized=state.history.ready();
  if(state.history.ready()) {
    next.history=state.history.values();
    next.stamp=state.history.stamp();
    next.automatic=state.history.reference().automatic_stiffness();
    next.context=state.history.reference().context();
  } else next.history.frame=state.initial_frame;
  if(!Valid(next.history)) return false;
  for(unsigned i=0;i<2;++i) {
    const auto& endpoint=state.cache.endpoint[i];
    if(!Finite(endpoint.force_n) || !Finite(endpoint.couple_nm) ||
        !Nonnegative(endpoint.translational_stiffness_n_m) || !Nonnegative(endpoint.rotational_stiffness_nm)) return false;
    next.endpoint[i]=endpoint;
  }
  const auto& d=state.cache.diagnostics;
  if(!Finite(d.local_separation_m) || !Finite(d.local_velocity_m_s) || !Finite(d.relative_rate_rad_s) ||
      !Nonnegative(d.harmonic_mass_kg) || !Nonnegative(d.harmonic_inertia_kg_m2) ||
      !Nonnegative(d.maximum_stiffness_n_m) || !Nonnegative(d.maximum_rotational_stiffness_nm) ||
      !Nonnegative(d.damping_n_s_m) || !Nonnegative(d.rotational_damping_nm_s) ||
      !Finite(d.internal_work_increment_j)) return false;
  next.diagnostics=d;
  output=next;
  return true;
}
} // namespace tl::fea::type45::resident_detail
