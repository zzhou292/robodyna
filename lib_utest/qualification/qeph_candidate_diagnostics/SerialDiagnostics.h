// Frozen b939d76 diagnostics; namespace and callable qualifiers only changed.
#pragma once
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/qeph/QephForce.h"
#include "lib_src/elements/ShellBatchFields.h"
#include "lib_src/elements/qeph/mapped/Result.h"
namespace qeph_diagnostics_test::frozen {
using namespace tl::fea;
using namespace tl::fea::qeph;
using namespace tl::fea::qeph::batch_detail;
using namespace tl::fea::shell_batch_fields;
TL_QEPH_HD inline bool Measure(const Model& model,const Slab& base,const Slab& next,
                              const NodalPreparedView& view,Control& out,const ShellSectionLaw* roles=nullptr) {
  auto& d=out.diagnostics;
  if(d.kinetic_available==model.joined) return false;
  for(unsigned n=0;n<model.config.owner.node_count;++n) {
    const auto v=ReadVector(view.kinematics.velocity_xyz,n),w=ReadVector(view.kinematics.angular_velocity_xyz,n);
    if(d.kinetic_available) {
      const double vv=Dot(v,v),ww=Dot(w,w);
      d.kinetic_translation+=.5*model.mass[n]*vv; d.kinetic_rotation+=.5*model.inertia[n]*ww;
      d.kinetic_physical_isotropic+=.5*model.physical[n]*ww; d.kinetic_added_isotropic+=.5*model.added[n]*ww;
    }
    const auto dx=Difference(ReadVector(view.kinematics.position_xyz,n),model.initial_position[n]);
    const double length=::hypot(::hypot(dx.x,dx.y),dx.z);
    if(!tl::math::Finite(length)) return false;
    d.maximum_displacement=::fmax(d.maximum_displacement,length);
    const auto* q=view.kinematics.orientation_wxyz+4*n;
    if(!tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]})) return false;
  }
  unsigned material_parent=0;
  for(unsigned e=0;e<model.config.element_count;++e) {
    if(roles&&roles[e]==ShellSectionLaw::RigidSkin) continue;
    const auto& element=model.element[e]; const auto& r=next.element[e]; const auto& h=r.proposed_history.data();
    const auto& old=base.element[e];
    const double area=r.kinematics.area/element.reference.area;
    const double thickness=h.thickness/element.reference.input.thickness;
    if(!detail::Positive(area)||!detail::Positive(thickness)) return false;
    d.minimum_area_ratio=material_parent?::fmin(d.minimum_area_ratio,area):area;
    d.minimum_thickness_ratio=material_parent?::fmin(d.minimum_thickness_ratio,thickness):thickness;
    d.minimum_native_dt=material_parent?::fmin(d.minimum_native_dt,r.diagnostics.unscaled_element_dt):r.diagnostics.unscaled_element_dt;
    ++material_parent;
    for(unsigned c=0;c<2;++c) { d.internal_work[c]+=h.internal_work[c]; d.internal_work_increment[c]+=r.diagnostics.internal_work_increment[c]; }
    d.hourglass_viscous_work+=h.hourglass_viscous_work;
    d.hourglass_viscous_work_increment+=r.diagnostics.hourglass_viscous_work_increment;
    for(unsigned c=0;c<5;++c) d.maximum_absolute_strain=::fmax(d.maximum_absolute_strain,::fabs(h.strain_curvature[c]));
    for(unsigned c=5;c<8;++c) d.maximum_thickness_curvature=::fmax(d.maximum_thickness_curvature,
      ::fabs(element.reference.input.thickness*h.strain_curvature[c]));
    if(d.accepted_force_assembled)
      AccumulateInternalWork(element.nodes,old.internal_force,old.internal_couple,view,model.config.owner.fixed_dt,
                             d.internal_kick_work,d.internal_drift_work);
  }
  const double finite[]={d.kinetic_translation,d.kinetic_rotation,d.kinetic_physical_isotropic,d.kinetic_added_isotropic,
    d.internal_work[0],d.internal_work[1],d.internal_work_increment[0],d.internal_work_increment[1],
    d.hourglass_viscous_work,d.hourglass_viscous_work_increment,d.minimum_area_ratio,d.minimum_thickness_ratio,
    d.maximum_displacement,d.maximum_absolute_strain,d.maximum_thickness_curvature,d.minimum_native_dt,
    d.internal_kick_work,d.internal_drift_work};
  for(double value:finite) if(!tl::math::Finite(value)) return false;
  return true;
}
TL_QEPH_HD inline void FinalizeCandidate(Storage* storage,const Slab* accepted,const Slab* trial,
                                   NodalPreparedView v,BatchDiagnostics identity,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  auto& s=*storage; s.control={}; s.control.diagnostics=identity;
  const auto* element_status=s.candidate_status;
  // The same-stream kernel boundary makes every parent result visible before
  // preserving the original first-failure scan and serial reduction order.
  // A failed candidate never exposes partially evaluated higher-index cells.
  for(unsigned i=0;i<s.model.config.element_count;++i) {
    const auto status=element_status[i];
    if(status!=Status::kSuccess) {
      s.control.status=BatchStatus::ElementFailure; s.control.element=i; s.control.element_status=status; return;
    }
  }
  if(s.model.mapped) {
    if(!mixed) { s.control.status=BatchStatus::InvalidInput; return; }
    for(unsigned e=0;e<s.model.config.element_count;++e) {
      const bool skin=mixed->law[e]==ShellSectionLaw::RigidSkin;
      if(!mapped::ValidResult(s.model.element[e].reference,trial->element[e],v.proposed_time,
          v.kinematics.base_epoch+1,skin)) {
        s.control.status=BatchStatus::NonfiniteResult; s.control.element=e; return;
      }
    }
  }
  if(!qeph_diagnostics_test::frozen::Measure(s.model,*accepted,*trial,v,s.control,s.model.mapped?mixed->law:nullptr)) { s.control.status=BatchStatus::NonfiniteResult; return; }
  s.control.diagnostics.valid=true;
}
} // namespace qeph_diagnostics_test::frozen
