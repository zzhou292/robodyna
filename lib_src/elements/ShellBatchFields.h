#pragma once
#include "../solvers/FENodalState.h"
#include "../math/Fixed3.h"
#include "../math/Quaternion.h"
namespace tl::fea::shell_batch_fields {
TL_SURFACE_HD inline tl::math::Vec3 ReadVector(const double* x,std::size_t n) { return {x[3*n],x[3*n+1],x[3*n+2]}; }
TL_SURFACE_HD inline double Dot(tl::math::Vec3 a,tl::math::Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
TL_SURFACE_HD inline tl::math::Vec3 Difference(tl::math::Vec3 a,tl::math::Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
TL_SURFACE_HD inline tl::math::Vec3 Mean(tl::math::Vec3 a,tl::math::Vec3 b) { return {(a.x+b.x)*.5,(a.y+b.y)*.5,(a.z+b.z)*.5}; }
TL_SURFACE_HD inline bool FiniteVector(tl::math::Vec3 x) { return tl::math::Finite(x.x)&&tl::math::Finite(x.y)&&tl::math::Finite(x.z); }
// Caller already validates complete bounded views/connectivity. Gathering is
// pure scratch work; owning formulation supplies its interval field names.
template<unsigned Count> TL_SURFACE_HD inline void Gather(const std::size_t (&nodes)[Count],
    DeviceNodalKinematicsView view,tl::math::Vec3 (&x)[Count],tl::math::Vec3 (&v)[Count],tl::math::Vec3 (&w)[Count]) {
  static_assert(Count==3||Count==4,"Native shell topology");
  for(unsigned i=0;i<Count;++i) {
    const auto n=nodes[i]; x[i]=ReadVector(view.position_xyz,n);
    v[i]=ReadVector(view.velocity_xyz,n); w[i]=ReadVector(view.angular_velocity_xyz,n);
  }
}
// Exact signed cache-work order from QephBatchDiagnostics; no constitutive
// energy interpretation. Caller measures finite outputs and owns publication.
template<unsigned Count> TL_SURFACE_HD inline void AccumulateInternalWork(
    const std::size_t (&nodes)[Count],const tl::math::Vec3 (&force)[Count],
    const tl::math::Vec3 (&couple)[Count],const NodalPreparedView& view,double h,
    double& kick_work,double& drift_work) {
  static_assert(Count==3||Count==4,"Native shell topology");
  for(unsigned i=0;i<Count;++i) {
    const auto n=nodes[i];
    const auto v0=ReadVector(view.base_kinematics.velocity_xyz,n),v1=ReadVector(view.kinematics.velocity_xyz,n);
    const auto w0=ReadVector(view.base_kinematics.angular_velocity_xyz,n),w1=ReadVector(view.kinematics.angular_velocity_xyz,n);
    const auto dx=Difference(ReadVector(view.kinematics.position_xyz,n),ReadVector(view.base_kinematics.position_xyz,n));
    const tl::math::Vec3 rotation{h*w1.x,h*w1.y,h*w1.z};
    kick_work-=view.kick_dt*(Dot(force[i],Mean(v0,v1))+Dot(couple[i],Mean(w0,w1)));
    drift_work-=Dot(force[i],dx)+Dot(couple[i],rotation);
  }
}
} // namespace tl::fea::shell_batch_fields
