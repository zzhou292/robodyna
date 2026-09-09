#pragma once

#include "Q4PlanarContactStorage.h"
#include "Q4ContactIntegration.h"

namespace tlfea::contact::q4_planar_detail {
using Interval=Q4IntegralInterval;
using Code=Q4PlanarContactStatus;

TL_SURFACE_HD inline bool SignedScale(Interval a,double value,Interval* output) {
  Interval next;
  if (!IsFinite(value) || !q4_bounds::Scale(a,::fabs(value),&next)) return false;
  *output=value < 0 ? Interval{-next.upper,-next.lower} : next;
  return true;
}
TL_SURFACE_HD inline bool Radius(double value,Interval truth,double* output) {
  double a=0,b=0;
  if (!q4_bounds::Finite(truth) || !q4_bounds::AbsoluteDifferenceUpper(value,truth.lower,&a) ||
      !q4_bounds::AbsoluteDifferenceUpper(value,truth.upper,&b)) return false;
  *output=a > b ? a : b; return true;
}
TL_SURFACE_HD inline bool AddUpper(double a,double b,double* output) {
  return a >= 0 && b >= 0 && q4_bounds::AddScalar(a,b,true,output);
}
TL_SURFACE_HD inline Q4PlanarReferenceView Reference(const Model& model) {
  return {model.reference,model.parent_count,static_cast<std::uint32_t>(model.config.owner.node_count),
          model.wall_x,model.wall_tolerance};
}
TL_SURFACE_HD inline Q4SurfaceView Surface(const Model& model,const tl::fea::DeviceNodalKinematicsView& state) {
  const auto n=static_cast<std::uint32_t>(model.config.owner.node_count);
  return {{state.position_xyz,n,3,1},{state.velocity_xyz,n,3,1},model.parents,model.parent_count};
}
TL_SURFACE_HD inline Q4FixedYZMassView Mass(const Model& model,std::uint64_t epoch) {
  return {model.inverse_mass,model.fixed,static_cast<std::uint32_t>(model.config.owner.node_count),epoch};
}
TL_SURFACE_HD inline unsigned Incident(const Model& model,unsigned global) {
  for (unsigned i=0;i<model.stiffness.count;++i) if (model.stiffness.nodes[i] == global) return i;
  return static_cast<unsigned>(MaxIncidentNodes);
}
TL_SURFACE_HD inline bool ExpandCertificate(Interval ratio,Q4CertifiedIntegral* value) {
  Interval truth;
  return q4_bounds::MultiplyPositive({value->lower,value->upper},ratio,&truth) &&
         q4_bounds::Certify(value->value,truth,value);
}
TL_SURFACE_HD inline bool ExpandArea(const PreparedQ4PlanarParent& reference,Q4IntegrationResult* result) {
  Interval ratio{1,1};
  if ((reference.area_enclosure.lower != reference.projected_area || reference.area_enclosure.upper != reference.projected_area) &&
      !q4_bounds::DividePositive(reference.area_enclosure,reference.projected_area,&ratio)) return false;
  for (auto& force:result->force) if (!ExpandCertificate(ratio,&force)) return false;
  return ExpandCertificate(ratio,&result->resultant) && ExpandCertificate(ratio,&result->potential) &&
         q4_bounds::MultiplyPositive(result->active_area,ratio,&result->active_area);
}
TL_SURFACE_HD inline bool Fail(Control& control,Code status,std::uint32_t parent=UINT32_MAX,std::uint32_t node=UINT32_MAX) {
  control.status=status; control.parent=parent; control.node=node; control.diagnostics.valid=false;
  return false;
}

TL_SURFACE_HD inline bool Evaluate(Storage& storage,const tl::fea::DeviceNodalKinematicsView& state,
                                   std::uint64_t attempt,Q4PlanarContactPhase phase) {
  auto& control=storage.control; control={};
  const auto& model=storage.model; auto& d=control.diagnostics;
  d.owner_id=model.config.owner.owner_id; d.base_epoch=state.base_epoch; d.attempt=attempt;
  d.configuration_id=model.config.configuration_id; d.wall_binding_id=model.config.wall_binding_id;
  d.phase=phase; d.parent_count=model.parent_count; d.stiffness_rate_bound=model.stiffness.rate_bound;
  for (unsigned i=0;i<MaxIncidentNodes;++i) {
    storage.force[i]=0; storage.force_error[i]=0; storage.force_truth[i]={};
  }
  for (auto& result:storage.result) result={};
  const auto surface=Surface(model,state); const auto mass=Mass(model,state.base_epoch);
  Interval potential;
  for (unsigned p=0;p<model.parent_count;++p) {
    Q4PreparedIntegration prepared;
    control.geometry=PrepareQ4PlanarIntegration(Reference(model),surface,mass,p,model.config.stiffness_per_area,
                                               model.config.maximum_penetration,attempt,&prepared);
    if (control.geometry != PlanarContactStatus::Ok) return Fail(control,Code::GeometryFailure,p);
    auto& parent=storage.result[p]; parent.covered=prepared.covered;
    if (!prepared.covered) continue;
    ++d.covered_count;
    control.integration=IntegrateQ4NormalContact(prepared.input,model.config.integration,
        {storage.leaves,storage.heap,MaxQ4IntegrationLeaves,MaxQ4IntegrationLeaves},&parent.integration);
    if (control.integration.status != Q4IntegrationStatus::Ok) return Fail(control,Code::IntegrationFailure,p);
    auto& integral=parent.integration;
    if (!ExpandArea(model.reference[p],&integral)) return Fail(control,Code::NonFiniteArithmetic,p);
    bool certified=integral.resultant.error <= model.config.integration.force_error &&
                   integral.potential.error <= model.config.integration.energy_error;
    for (const auto& force:integral.force) certified=certified && force.error <= model.config.integration.force_error;
    if (!certified) {
      control.integration.status=Q4IntegrationStatus::UnattainableAccuracy;
      return Fail(control,Code::IntegrationFailure,p);
    }
    d.leaves+=integral.leaf_count; d.visited+=integral.visited;
    if (integral.deepest_leaf > d.deepest_leaf) d.deepest_leaf=integral.deepest_leaf;
    if (!q4_bounds::Add(d.active_area,integral.active_area,&d.active_area) ||
        !q4_bounds::Add(potential,{integral.potential.lower,integral.potential.upper},&potential))
      return Fail(control,Code::NonFiniteArithmetic,p);
    d.potential.value+=integral.potential.value;
    for (unsigned local=0;local<4;++local) {
      const auto node=integral.nodal.nodes[local]; const auto index=Incident(model,node);
      if (index >= model.stiffness.count) return Fail(control,Code::InvalidInput,p,node);
      storage.force[index]+=integral.nodal.forces[local].x;
      if (!IsFinite(storage.force[index]) || !q4_bounds::Add(storage.force_truth[index],
          {integral.force[local].lower,integral.force[local].upper},&storage.force_truth[index]))
        return Fail(control,Code::NonFiniteArithmetic,p,node);
      const double depth=state.position_xyz[3*node]-model.wall_x;
      if (!IsFinite(depth)) return Fail(control,Code::NonFiniteArithmetic,p,node);
      if (depth > d.maximum_penetration) d.maximum_penetration=depth;
    }
  }
  if (!q4_bounds::Certify(d.potential.value,potential,&d.potential)) return Fail(control,Code::NonFiniteArithmetic);
  Interval force,moment_y,moment_z,power;
  for (unsigned i=0;i<model.stiffness.count;++i) {
    const auto node=model.stiffness.nodes[i]; const double normal_force=storage.force[i];
    const Interval positive=storage.force_truth[i],negative{-positive.upper,-positive.lower};
    if (!Radius(normal_force,negative,&storage.force_error[i]) || !q4_bounds::Add(force,negative,&force))
      return Fail(control,Code::NonFiniteArithmetic,UINT32_MAX,node);
    const double y=state.position_xyz[3*node+1],z=state.position_xyz[3*node+2],v=state.velocity_xyz[3*node];
    d.force_on_surface.x+=normal_force;
    d.wall_moment.y-=z*normal_force; d.wall_moment.z+=y*normal_force;
    d.surface_power+=normal_force*v;
    Interval term;
    if (!SignedScale(positive,z,&term) || !q4_bounds::Add(moment_y,term,&moment_y) ||
        !SignedScale(positive,-y,&term) || !q4_bounds::Add(moment_z,term,&moment_z) ||
        !SignedScale(negative,v,&term) || !q4_bounds::Add(power,term,&power))
      return Fail(control,Code::NonFiniteArithmetic,UINT32_MAX,node);
  }
  d.wall_reaction.x=-d.force_on_surface.x;
  if (!Radius(d.force_on_surface.x,force,&d.force_error.x) ||
      !Radius(d.wall_moment.y,moment_y,&d.wall_moment_error.y) ||
      !Radius(d.wall_moment.z,moment_z,&d.wall_moment_error.z) ||
      !Radius(d.surface_power,power,&d.surface_power_error)) return Fail(control,Code::NonFiniteArithmetic);
  return true;
}

TL_SURFACE_HD inline bool MeasureInterval(Storage& storage,const tl::fea::NodalPreparedView& prepared) {
  auto& d=storage.control.diagnostics; const auto& base=storage.base; const auto& model=storage.model;
  const double h=model.config.owner.fixed_dt;
  d.base_potential=base.diagnostics.potential.value; d.base_potential_error=base.diagnostics.potential.error;
  d.potential_increment=d.potential.value-d.base_potential;
  Interval midpoint_work,coordinate_work,delta[MaxIncidentNodes]{};
  double continuum_force_work_error=0,addition_midpoint_error=0,addition_coordinate_error=0;
  for (unsigned i=0;i<model.stiffness.count;++i) {
    const auto node=model.stiffness.nodes[i]; const auto offset=3*node;
    const double old_x=prepared.base_kinematics.position_xyz[offset],new_x=prepared.kinematics.position_xyz[offset];
    const double old_v=prepared.base_kinematics.velocity_xyz[offset],new_v=prepared.kinematics.velocity_xyz[offset];
    const double displacement=new_x-old_x,velocity=.5*(old_v+new_v),force=base.force[i];
    Interval mean,term;
    if (!q4_bounds::Difference(new_x,old_x,&delta[i]) ||
        !q4_bounds::Add({old_v,old_v},{new_v,new_v},&mean) || !q4_bounds::Scale(mean,.5,&mean) ||
        !SignedScale(mean,force,&term) || !q4_bounds::Scale(term,h,&term) ||
        !q4_bounds::Add(midpoint_work,term,&midpoint_work) ||
        !SignedScale(delta[i],force,&term) || !q4_bounds::Add(coordinate_work,term,&coordinate_work))
      return Fail(storage.control,Code::NonFiniteArithmetic,UINT32_MAX,node);
    d.kinetic_midpoint_work+=h*force*velocity; d.force_coordinate_work+=force*displacement;
    const double dx=::fabs(delta[i].lower) > ::fabs(delta[i].upper) ? ::fabs(delta[i].lower) : ::fabs(delta[i].upper);
    const double vm=::fabs(mean.lower) > ::fabs(mean.upper) ? ::fabs(mean.lower) : ::fabs(mean.upper);
    double error=0;
    if (!mass_detail::UpperProduct(dx,base.force_error[i],&error) ||
        !AddUpper(continuum_force_work_error,error,&continuum_force_work_error) ||
        !mass_detail::UpperProduct(vm,base.addition_error[i],&error) ||
        !mass_detail::UpperProduct(h,error,&error) || !AddUpper(addition_midpoint_error,error,&addition_midpoint_error) ||
        !mass_detail::UpperProduct(dx,base.addition_error[i],&error) ||
        !AddUpper(addition_coordinate_error,error,&addition_coordinate_error))
      return Fail(storage.control,Code::NonFiniteArithmetic,UINT32_MAX,node);
  }
  if (!Radius(d.kinetic_midpoint_work,midpoint_work,&d.kinetic_midpoint_roundoff) ||
      !AddUpper(d.kinetic_midpoint_roundoff,addition_midpoint_error,&d.kinetic_midpoint_roundoff) ||
      !Radius(d.force_coordinate_work,coordinate_work,&d.force_coordinate_roundoff) ||
      !AddUpper(d.force_coordinate_roundoff,addition_coordinate_error,&d.force_coordinate_roundoff) ||
      !AddUpper(d.base_potential_error,d.potential.error,&d.continuum_work_uncertainty) ||
      !AddUpper(d.continuum_work_uncertainty,continuum_force_work_error,&d.continuum_work_uncertainty) ||
      !AddUpper(d.continuum_work_uncertainty,d.force_coordinate_roundoff,&d.continuum_work_uncertainty))
    return Fail(storage.control,Code::NonFiniteArithmetic);
  Interval potential_difference,defect;
  if (!q4_bounds::Difference(d.potential.value,d.base_potential,&potential_difference) ||
      !q4_bounds::Add(potential_difference,{d.force_coordinate_work,d.force_coordinate_work},&defect))
    return Fail(storage.control,Code::NonFiniteArithmetic);
  d.conservative_force_coordinate_defect=d.potential_increment+d.force_coordinate_work;
  double arithmetic=0;
  if (!Radius(d.conservative_force_coordinate_defect,defect,&arithmetic) ||
      !AddUpper(d.continuum_work_uncertainty,arithmetic,&d.continuum_work_uncertainty) ||
      BoundQ4PlanarQuadratic(model.stiffness,delta,&d.quadratic_work_upper) != PlanarContactStatus::Ok)
    return Fail(storage.control,Code::NonFiniteArithmetic);
  return IsFinite(d.potential_increment) && IsFinite(d.kinetic_midpoint_work) && IsFinite(d.force_coordinate_work) &&
         IsFinite(d.conservative_force_coordinate_defect) ? true : Fail(storage.control,Code::NonFiniteArithmetic);
}
}  // namespace tlfea::contact::q4_planar_detail
