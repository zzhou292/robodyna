#pragma once
#include "NodalWallContactStorage.h"

namespace tlfea::contact::nodal_wall_device_detail {
// Floating-work and reference-coefficient uncertainty are separate. Both
// kick/impulse use the actual kick duration; geometry drift always spans h.
TL_SURFACE_HD inline bool MeasureInterval(Storage& s,const tl::fea::NodalPreparedView& v) {
  auto& d=s.result.diagnostics; const auto& b=s.base.diagnostics;
  d.base_potential=b.potential.value; d.base_potential_error=b.potential.error;
  d.potential_increment=d.potential.value-b.potential.value;
  Q4IntegralInterval kick,drift,potential_delta,defect,moment_y,moment_z;
  double addition_kick=0,addition_drift=0,force_uncertainty=0,quadratic=0;
  for (unsigned i=0;i<s.model.node_count;++i) {
    const auto& node=s.base.nodes[i]; const auto n=node.node;
    const double force=node.force_world.x;
    const double a=v.base_kinematics.position_xyz[3*n],x=v.kinematics.position_xyz[3*n];
    const double va=v.base_kinematics.velocity_xyz[3*n],vx=v.kinematics.velocity_xyz[3*n];
    const double dx=x-a,mean=.5*(va+vx);
    Q4IntegralInterval displacement,velocity,term;
    if (!q4_bounds::Difference(x,a,&displacement) ||
        !q4_bounds::Add({va,va},{vx,vx},&velocity) || !q4_bounds::Scale(velocity,.5,&velocity) ||
        !SignedScale(velocity,force,&term) || !q4_bounds::Scale(term,v.kick_dt,&term) ||
        !q4_bounds::Add(kick,term,&kick) || !SignedScale(displacement,force,&term) ||
        !q4_bounds::Add(drift,term,&drift) ||
        !SignedScale({node.force.lower,node.force.upper},node.wall_point.z,&term) ||
        !q4_bounds::Add(moment_y,term,&moment_y) ||
        !SignedScale({node.force.lower,node.force.upper},-node.wall_point.y,&term) ||
        !q4_bounds::Add(moment_z,term,&moment_z)) return Fail(s.control,Code::NonFiniteArithmetic,n);
    d.kick_work+=v.kick_dt*force*mean; d.drift_work+=force*dx;
    const double abs_dx=::fmax(::fabs(displacement.lower),::fabs(displacement.upper));
    const double abs_v=::fmax(::fabs(velocity.lower),::fabs(velocity.upper));
    double error=0,square=0;
    if (!mass_detail::UpperProduct(s.addition_error[i],abs_v,&error) ||
        !mass_detail::UpperProduct(error,v.kick_dt,&error) || !AddUpper(addition_kick,error,&addition_kick) ||
        !mass_detail::UpperProduct(s.addition_error[i],abs_dx,&error) ||
        !AddUpper(addition_drift,error,&addition_drift) ||
        !mass_detail::UpperProduct(node.force.error,abs_dx,&error) ||
        !AddUpper(force_uncertainty,error,&force_uncertainty) ||
        !mass_detail::UpperProduct(abs_dx,abs_dx,&square) ||
        !mass_detail::UpperProduct(node.stiffness.upper,square,&error) ||
        !mass_detail::UpperProduct(.5,error,&error) || !AddUpper(quadratic,error,&quadratic))
      return Fail(s.control,Code::NonFiniteArithmetic,n);
  }
  if (!Radius(d.kick_work,kick,&d.kick_work_roundoff) ||
      !AddUpper(d.kick_work_roundoff,addition_kick,&d.kick_work_roundoff) ||
      !Radius(d.drift_work,drift,&d.drift_work_roundoff) ||
      !AddUpper(d.drift_work_roundoff,addition_drift,&d.drift_work_roundoff) ||
      !AddUpper(b.potential.error,d.potential.error,&d.work_uncertainty) ||
      !AddUpper(d.work_uncertainty,force_uncertainty,&d.work_uncertainty) ||
      !AddUpper(d.work_uncertainty,d.drift_work_roundoff,&d.work_uncertainty) ||
      !q4_bounds::Difference(d.potential.value,b.potential.value,&potential_delta) ||
      !q4_bounds::Add(potential_delta,{d.drift_work,d.drift_work},&defect))
    return Fail(s.control,Code::NonFiniteArithmetic);
  d.conservative_defect=d.potential_increment+d.drift_work; d.quadratic_work_upper=quadratic;
  double arithmetic=0;
  Q4IntegralInterval impulse;
  if (!Radius(d.conservative_defect,defect,&arithmetic) ||
      !AddUpper(d.work_uncertainty,arithmetic,&d.work_uncertainty) ||
      !q4_bounds::Scale({b.resultant.lower,b.resultant.upper},v.kick_dt,&impulse))
    return Fail(s.control,Code::NonFiniteArithmetic);
  d.wall_kick_impulse=v.kick_dt*b.wall_reaction.x;
  d.wall_kick_moment={0,v.kick_dt*b.wall_moment.y,v.kick_dt*b.wall_moment.z};
  if (!Radius(d.wall_kick_impulse,impulse,&d.wall_kick_impulse_error) ||
      !q4_bounds::Scale(moment_y,v.kick_dt,&moment_y) || !q4_bounds::Scale(moment_z,v.kick_dt,&moment_z) ||
      !Radius(d.wall_kick_moment.y,moment_y,&d.wall_kick_moment_error.y) ||
      !Radius(d.wall_kick_moment.z,moment_z,&d.wall_kick_moment_error.z) ||
      !IsFinite(d.kick_work) || !IsFinite(d.drift_work) || !IsFinite(d.potential_increment) ||
      !IsFinite(d.conservative_defect)) return Fail(s.control,Code::NonFiniteArithmetic);
  return true;
}
} // namespace tlfea::contact::nodal_wall_device_detail
