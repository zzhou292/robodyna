// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidObservationTypes.h"
#include <limits>
namespace tl::fea::rigid::observation_detail {
constexpr double Epsilon=std::numeric_limits<double>::epsilon();
constexpr std::size_t MaxMembers=256;
TL_RIGID_OBSERVATION_HD inline double Get(Vec3 v,unsigned a) { return a==0?v.x:a==1?v.y:v.z; }
TL_RIGID_OBSERVATION_HD inline bool Finite(double x) { return tl::math::Finite(x); }
TL_RIGID_OBSERVATION_HD inline bool Nonnegative(double x) { return Finite(x)&&x>=0; }
// Neumaier accumulation bounds cancellation in signed work and partition sums.
// A nonfinite intermediate is rejected, never saturated into an observation.
struct Sum {
  double sum=0,correction=0;
  TL_RIGID_OBSERVATION_HD bool Add(double value) {
    if(!Finite(value)) return false;
    const double next=sum+value;
    const double error=::fabs(sum)>=::fabs(value)?(sum-next)+value:(value-next)+sum;
    correction+=error; sum=next; return Finite(sum)&&Finite(correction);
  }
  TL_RIGID_OBSERVATION_HD double Value() const { return sum+correction; }
};
TL_RIGID_OBSERVATION_HD inline bool Near(double a,double b,double factor) {
  return Finite(a)&&Finite(b)&&::fabs(a-b)<=factor*Epsilon*::fmax(::fabs(a),::fabs(b));
}
TL_RIGID_OBSERVATION_HD inline double Kinetic(double coefficient,Vec3 velocity) {
  return .5*coefficient*(velocity.x*velocity.x+velocity.y*velocity.y+velocity.z*velocity.z);
}
TL_RIGID_OBSERVATION_HD inline double PrincipalKinetic(Vec3 j,Vec3 w) {
  return .5*(j.x*w.x*w.x+j.y*w.y*w.y+j.z*w.z*w.z);
}
TL_RIGID_OBSERVATION_HD inline bool ValidPhase(ObservationPhase p) {
  if(!Nonnegative(p.position_time)||!Nonnegative(p.velocity_time)||!Nonnegative(p.frame_time)) return false;
  if(p.kind==ObservationPhaseKind::PhysicalInitialization)
    return p.position_time==p.velocity_time&&p.velocity_time==p.frame_time;
  if(p.kind!=ObservationPhaseKind::StoredMidpointWithLaggedFrame||
      !(p.position_time>p.velocity_time&&p.velocity_time>p.frame_time)) return false;
  return Near(p.velocity_time,p.frame_time+.5*(p.position_time-p.frame_time),8);
}
TL_RIGID_OBSERVATION_HD inline bool ValidMetric(GroupObservationMetric input) {
  if(!input.group||!input.members||input.member_count<3||input.member_count>MaxMembers||
      input.member_count!=input.group->member_count) return false;
  const auto& g=*input.group; const auto& r=g.regularization;
  if(!g.source_group_id||!g.source_node_set_id||!detail::Orthonormal(g.principal.axes)||
      !detail::Positive(g.principal.inertia)||!detail::Finite(g.raw_principal_inertia)||
      !detail::Finite(g.center)||!detail::Finite(g.generated_primary_position)||
      !Nonnegative(g.structural_mass_kg)||!Nonnegative(g.total_mass_kg)||g.total_mass_kg==0||
      !Nonnegative(r.primary_mass_kg)||!Nonnegative(r.primary_isotropic_inertia_kg_m2)) return false;
  PrincipalCorrection correction;
  if(CorrectPrincipalInertia(g.raw_principal_inertia,correction)!=MathStatus::Success) return false;
  for(unsigned a=0;a<3;++a)
    if(Get(correction.effective,a)!=Get(g.principal.inertia,a)||
        Get(correction.added,a)!=Get(r.principal_inertia_added,a)) return false;
  Sum mass,j,physical,added;
  for(std::size_t i=0;i<input.member_count;++i) {
    const auto& m=input.members[i];
    if(!m.source_node_id||!detail::Finite(m.position)||!Nonnegative(m.mass_kg)||m.mass_kg==0||
        !Nonnegative(m.total_inertia_kg_m2)||m.total_inertia_kg_m2==0||
        !Nonnegative(m.physical_inertia_kg_m2)||!Nonnegative(m.added_inertia_kg_m2)||
        !Near(m.total_inertia_kg_m2,m.physical_inertia_kg_m2+m.added_inertia_kg_m2,64)||
        !mass.Add(m.mass_kg)||!j.Add(m.total_inertia_kg_m2)||
        !physical.Add(m.physical_inertia_kg_m2)||!added.Add(m.added_inertia_kg_m2)) return false;
  }
  const double factor=64+8*input.member_count;
  if(!Near(mass.Value(),g.structural_mass_kg,factor)||!Near(j.Value(),g.native_total_inertia_sum,factor)||
      !Near(physical.Value(),g.physical_inertia_sum,factor)||!Near(added.Value(),g.added_inertia_sum,factor)||
      !Near(g.total_mass_kg,g.structural_mass_kg+r.primary_mass_kg,factor)) return false;
  // No new eigen solve. Verify the supplied reference tensor/ledger against its
  // already prepared principal representation, allowing existing frame error.
  double scale=0;
  for(unsigned i=0;i<9;++i) {
    if(!Finite(g.raw_tensor.v[i])||!Finite(g.effective_tensor.v[i])||!Finite(r.tensor_added.v[i])) return false;
    scale=::fmax(scale,::fabs(g.effective_tensor.v[i]));
  }
  for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b) {
    double reconstructed=0;
    for(unsigned k=0;k<3;++k) reconstructed+=g.principal.axes.v[3*a+k]*Get(g.principal.inertia,k)*g.principal.axes.v[3*b+k];
    const auto index=3*a+b;
    if(!Finite(reconstructed)||::fabs(reconstructed-g.effective_tensor.v[index])>2e-12*scale||
        ::fabs(g.raw_tensor.v[index]+r.tensor_added.v[index]-g.effective_tensor.v[index])>factor*Epsilon*scale) return false;
  }
  return true;
}
} // namespace tl::fea::rigid::observation_detail
