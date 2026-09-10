// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidObservationMath.h"
namespace tl::fea::rigid {
// Supplied phase and values only: no clock, source authentication, eigen solve,
// state reconstruction, allocation or publication. On failure output is intact.
// Stored-midpoint energy explicitly pairs midpoint velocities with the supplied
// lagged force-stage axes; it is not the native collocated RGBCOR diagnostic.
TL_RIGID_OBSERVATION_HD inline ObservationReport ObserveGroupKinetic(
    const GroupKineticInput& input,GroupKineticObservation& output) {
  using namespace observation_detail;
  if(!ValidPhase(input.phase)) return {ObservationStatus::UnsupportedPhase};
  if(!input.members||!ValidGroupState(input.group)) return {ObservationStatus::InvalidInput};
  if(!ValidMetric(input.metric)) return {ObservationStatus::InvalidMetric};
  const auto& metric=*input.metric.group; const auto& regularization=metric.regularization;
  GroupKineticObservation next; next.phase=input.phase;
  auto& nodal=next.members; auto& body=next.aggregate;
  const auto w=detail::ToLocal(input.group.principal_axes,input.group.omega);
  Sum translation,rotation,physical,added,orbital;
  for(std::size_t i=0;i<input.metric.member_count;++i) {
    const auto& m=input.metric.members[i]; const auto motion=input.members[i];
    if(!detail::Finite(motion.velocity)||!detail::Finite(motion.omega)) return {ObservationStatus::InvalidInput,i};
    const auto arm=detail::ToLocal(metric.principal.axes,detail::Subtract(m.position,metric.center));
    const auto tangent=detail::Cross(w,arm);
    if(!detail::Finite(arm)||!detail::Finite(tangent)||
        !translation.Add(Kinetic(m.mass_kg,motion.velocity))||!rotation.Add(Kinetic(m.total_inertia_kg_m2,motion.omega))||
        !physical.Add(Kinetic(m.physical_inertia_kg_m2,motion.omega))||!added.Add(Kinetic(m.added_inertia_kg_m2,motion.omega))||
        !orbital.Add(Kinetic(m.mass_kg,tangent))) return {ObservationStatus::NonfiniteResult,i};
  }
  nodal.translation=translation.Value(); nodal.native_rotation=rotation.Value();
  nodal.physical_rotation=physical.Value(); nodal.added_rotation=added.Value();
  nodal.total=nodal.translation+nodal.native_rotation;
  nodal.inertia_partition_residual=(nodal.native_rotation-nodal.physical_rotation)-nodal.added_rotation;
  body.translation=Kinetic(metric.total_mass_kg,input.group.velocity);
  // Principal representation is equivalent to omega^T (R J R^T) omega/2,
  // including all off-diagonal terms of the current WORLD tensor.
  body.rotation=PrincipalKinetic(metric.principal.inertia,w); body.total=body.translation+body.rotation;
  body.structural_translation=Kinetic(metric.structural_mass_kg,input.group.velocity);
  body.primary_translation=Kinetic(regularization.primary_mass_kg,input.group.velocity);
  body.member_orbital_rotation=orbital.Value();
  body.native_member_rotation=Kinetic(metric.native_total_inertia_sum,w);
  body.physical_member_rotation=Kinetic(metric.physical_inertia_sum,w);
  body.added_member_rotation=Kinetic(metric.added_inertia_sum,w);
  const auto primary_arm=detail::ToLocal(metric.principal.axes,
      detail::Subtract(metric.generated_primary_position,metric.center));
  body.primary_parallel_axis_rotation=Kinetic(regularization.primary_mass_kg,detail::Cross(w,primary_arm));
  body.primary_isotropic_rotation=Kinetic(regularization.primary_isotropic_inertia_kg_m2,w);
  body.principal_correction_rotation=PrincipalKinetic(regularization.principal_inertia_added,w);
  Sum decomposition;
  const double terms[]{body.member_orbital_rotation,body.native_member_rotation,body.primary_parallel_axis_rotation,
      body.primary_isotropic_rotation,body.principal_correction_rotation};
  for(double term:terms) if(!Nonnegative(term)||!decomposition.Add(term)) return {ObservationStatus::NonfiniteResult};
  body.decomposition_residual=body.rotation-decomposition.Value();
  // Covers existing principal-frame admission (1e-12), its transforms, startup
  // source-order sums and this bounded compensated positive decomposition.
  body.decomposition_roundoff_budget=(8e-12+(64+8*input.metric.member_count)*Epsilon)*
      ::fmax(::fabs(body.rotation),::fabs(decomposition.Value()));
  next.replacement=body.total-nodal.total;
  const double final_values[]{nodal.total,nodal.inertia_partition_residual,body.total,body.structural_translation,
      body.primary_translation,body.physical_member_rotation,body.added_member_rotation,
      body.decomposition_residual,body.decomposition_roundoff_budget,next.replacement};
  for(double value:final_values) if(!Finite(value)) return {ObservationStatus::NonfiniteResult};
  if(::fabs(body.decomposition_residual)>body.decomposition_roundoff_budget)
    return {ObservationStatus::InvalidMetric,SIZE_MAX,6,body.decomposition_residual,body.decomposition_roundoff_budget};
  output=next; return {};
}
} // namespace tl::fea::rigid
