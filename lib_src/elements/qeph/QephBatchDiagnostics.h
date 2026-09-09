#pragma once
#include "QephBatchStorage.h"
#include "QephForce.h"
#include "../../math/Quaternion.h"

namespace tl::fea::qeph::batch_detail {
TL_QEPH_HD inline Vec3 ReadVector(const double* x,std::size_t n) { return {x[3*n],x[3*n+1],x[3*n+2]}; }
TL_QEPH_HD inline double Dot(Vec3 a,Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
TL_QEPH_HD inline Vec3 Difference(Vec3 a,Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
TL_QEPH_HD inline Vec3 Mean(Vec3 a,Vec3 b) { return {(a.x+b.x)*.5,(a.y+b.y)*.5,(a.z+b.z)*.5}; }
TL_QEPH_HD inline bool FiniteVector(Vec3 x) { return tl::math::Finite(x.x)&&tl::math::Finite(x.y)&&tl::math::Finite(x.z); }

// Actual global shared masses/J and one owner velocity per node. Physical and
// added scalar partitions are explicitly isotropic (including drilling).
TL_QEPH_HD inline bool Measure(const Model& model,const Slab& base,const Slab& next,
                              const NodalPreparedView& view,Control& out) {
  auto& d=out.diagnostics;
  for(unsigned n=0;n<model.config.owner.node_count;++n) {
    const auto v=ReadVector(view.kinematics.velocity_xyz,n),w=ReadVector(view.kinematics.angular_velocity_xyz,n);
    const double vv=Dot(v,v),ww=Dot(w,w);
    d.kinetic_translation+=.5*model.mass[n]*vv; d.kinetic_rotation+=.5*model.inertia[n]*ww;
    d.kinetic_physical_isotropic+=.5*model.physical[n]*ww; d.kinetic_added_isotropic+=.5*model.added[n]*ww;
    const auto dx=Difference(ReadVector(view.kinematics.position_xyz,n),model.initial_position[n]);
    const double length=::hypot(::hypot(dx.x,dx.y),dx.z);
    if(!tl::math::Finite(length)) return false;
    d.maximum_displacement=::fmax(d.maximum_displacement,length);
    const auto* q=view.kinematics.orientation_wxyz+4*n;
    if(!tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]})) return false;
  }
  for(unsigned e=0;e<model.config.element_count;++e) {
    const auto& element=model.element[e]; const auto& r=next.element[e]; const auto& h=r.proposed_history.data();
    const auto& old=base.element[e];
    const double area=r.kinematics.area/element.reference.area;
    const double thickness=h.thickness/element.reference.input.thickness;
    if(!detail::Positive(area)||!detail::Positive(thickness)) return false;
    d.minimum_area_ratio=e?::fmin(d.minimum_area_ratio,area):area;
    d.minimum_thickness_ratio=e?::fmin(d.minimum_thickness_ratio,thickness):thickness;
    d.minimum_native_dt=e?::fmin(d.minimum_native_dt,r.diagnostics.unscaled_element_dt):r.diagnostics.unscaled_element_dt;
    for(unsigned c=0;c<2;++c) { d.internal_work[c]+=h.internal_work[c]; d.internal_work_increment[c]+=r.diagnostics.internal_work_increment[c]; }
    d.hourglass_viscous_work+=h.hourglass_viscous_work;
    d.hourglass_viscous_work_increment+=r.diagnostics.hourglass_viscous_work_increment;
    for(unsigned c=0;c<5;++c) d.maximum_absolute_strain=::fmax(d.maximum_absolute_strain,::fabs(h.strain_curvature[c]));
    for(unsigned c=5;c<8;++c) d.maximum_thickness_curvature=::fmax(d.maximum_thickness_curvature,
      ::fabs(element.reference.input.thickness*h.strain_curvature[c]));
    for(unsigned i=0;d.accepted_force_assembled&&i<4;++i) {
      const auto n=element.nodes[i];
      const auto v0=ReadVector(view.base_kinematics.velocity_xyz,n),v1=ReadVector(view.kinematics.velocity_xyz,n);
      const auto w0=ReadVector(view.base_kinematics.angular_velocity_xyz,n),w1=ReadVector(view.kinematics.angular_velocity_xyz,n);
      const auto dx=Difference(ReadVector(view.kinematics.position_xyz,n),ReadVector(view.base_kinematics.position_xyz,n));
      // The owning update uses this exact world rotation-vector increment.
      const Vec3 rotation{model.config.owner.fixed_dt*w1.x,model.config.owner.fixed_dt*w1.y,model.config.owner.fixed_dt*w1.z};
      d.internal_kick_work-=view.kick_dt*(Dot(old.internal_force[i],Mean(v0,v1))+Dot(old.internal_couple[i],Mean(w0,w1)));
      d.internal_drift_work-=Dot(old.internal_force[i],dx)+Dot(old.internal_couple[i],rotation);
    }
  }
  const double finite[]={d.kinetic_translation,d.kinetic_rotation,d.kinetic_physical_isotropic,d.kinetic_added_isotropic,
    d.internal_work[0],d.internal_work[1],d.internal_work_increment[0],d.internal_work_increment[1],
    d.hourglass_viscous_work,d.hourglass_viscous_work_increment,d.minimum_area_ratio,d.minimum_thickness_ratio,
    d.maximum_displacement,d.maximum_absolute_strain,d.maximum_thickness_curvature,d.minimum_native_dt,
    d.internal_kick_work,d.internal_drift_work};
  for(double value:finite) if(!tl::math::Finite(value)) return false;
  return true;
}
} // namespace tl::fea::qeph::batch_detail
