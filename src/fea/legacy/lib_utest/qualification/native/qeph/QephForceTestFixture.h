#pragma once

#include "QephForceReference.h"
#include "lib_src/math/Quaternion.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <type_traits>

namespace qeph_force_test {
namespace native=tl::qualification::qeph;
using namespace native;
// Frozen before Q2 force execution. Small <=2m, E=2MPa reference fixtures;
// these are numerical reference budgets, not a solver admission policy.
constexpr double kRelative=2e-10,kAbsolute=2e-11,kCovariance=2e-10,kEnergy=2e-22;
inline void Near(double a,double b,double absolute=kAbsolute) {
  EXPECT_NEAR(a,b,absolute+kRelative*std::max(std::abs(a),std::abs(b)));
}
template<std::size_t N> void Near(const std::array<double,N>& a,const std::array<double,N>& b,
                                 double absolute=kAbsolute) {
  for (unsigned i=0;i<N;++i) { SCOPED_TRACE(i); Near(a[i],b[i],absolute); }
}
inline Vec3 Add(Vec3 a,Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 Scale(Vec3 a,double s) { return {s*a.x,s*a.y,s*a.z}; }
inline double Dot(Vec3 a,Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 Cross(Vec3 a,Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline double Norm(Vec3 a) { return std::hypot(a.x,a.y,a.z); }
inline void Near(Vec3 a,Vec3 b,double absolute=kAbsolute) {
  Near(a.x,b.x,absolute); Near(a.y,b.y,absolute); Near(a.z,b.z,absolute);
}
inline tl::math::Quaternion Rotation(Vec3 vector) {
  const double delta[]{vector.x,vector.y,vector.z}; tl::math::Quaternion result;
  EXPECT_TRUE(tl::math::IncrementWorldRotation({},delta,result)); return result;
}
inline Vec3 Rotate(tl::math::Quaternion q,Vec3 v) {
  const auto p=tl::math::Product(tl::math::Product(q,{0,v.x,v.y,v.z}),{q.w,-q.x,-q.y,-q.z});
  return {p.x,p.y,p.z};
}
template<class T> auto Bytes(const T& object) {
  static_assert(std::is_trivially_copyable_v<T>);
  std::array<unsigned char,sizeof(T)> bytes; std::memcpy(bytes.data(),&object,sizeof(T)); return bytes;
}
inline ReferenceInput Rectangle(double warp=0) {
  ReferenceInput input;
  input.position={{{-1,-.5,warp},{1,-.5,-warp},{1,.5,warp},{-1,.5,-warp}}};
  input.node_ids={{17,3,1001,51}};
  input.density=10; input.young_modulus=2e6; input.poisson_ratio=.25; input.thickness=.1;
  return input;
}
inline PrescribedInterval Interval(const ReferenceInput& input,const History& history,double dt=1e-6) {
  PrescribedInterval result; result.position_endpoint=input.position; result.dt=dt;
  result.base_time=history.stamp().time; result.sample_index=history.stamp().sample_index+1;
  return result;
}
// Independent continuous affine velocity/director fields; material order is
// XX YY XY YZ ZX KXX KYY KXY (unlike Q1's XZ/YZ rate slots).
inline void Mode(PrescribedInterval& interval,unsigned mode,double rate) {
  for (unsigned n=0;n<4;++n) {
    const auto p=interval.position_endpoint[n];
    if (mode==0) interval.velocity_midpoint[n].x=rate*p.x;
    if (mode==1) interval.velocity_midpoint[n].y=rate*p.y;
    if (mode==2) interval.velocity_midpoint[n].x=rate*p.y;
    if (mode==3) interval.omega_midpoint[n].x=-rate;
    if (mode==4) interval.omega_midpoint[n].y=rate;
    if (mode==5) interval.omega_midpoint[n].y=rate*p.x;
    if (mode==6) interval.omega_midpoint[n].x=-rate*p.y;
    if (mode==7) interval.omega_midpoint[n]={-.5*rate*p.x,.5*rate*p.y,0};
  }
}
inline void SameHistory(const HistoryValues& a,const HistoryValues& b,double absolute=kAbsolute) {
  Near(a.stress,b.stress,absolute); Near(a.material_stress,b.material_stress,absolute);
  Near(a.bending_stress,b.bending_stress,absolute); Near(a.stabilization,b.stabilization,absolute);
  Near(a.strain_curvature,b.strain_curvature,absolute); Near(a.thickness,b.thickness,absolute);
  Near(a.internal_work,b.internal_work,kEnergy); Near(a.hourglass_viscous_work,b.hourglass_viscous_work,kEnergy);
  EXPECT_EQ(a.active,b.active);
}
inline void SameGeometry(const Kinematics& a,const Kinematics& b) {
  for (unsigned i=0;i<9;++i) EXPECT_DOUBLE_EQ(a.frame.v[i],b.frame.v[i]);
  EXPECT_DOUBLE_EQ(a.area,b.area); EXPECT_DOUBLE_EQ(a.reciprocal_area,b.reciprocal_area);
  EXPECT_DOUBLE_EQ(a.characteristic_length,b.characteristic_length);
  EXPECT_EQ(a.nodal_factors,b.nodal_factors); EXPECT_DOUBLE_EQ(a.raw_warpage_abs,b.raw_warpage_abs);
  EXPECT_DOUBLE_EQ(a.effective_warpage,b.effective_warpage); EXPECT_EQ(a.planar,b.planar);
  EXPECT_EQ(a.projection_inverse,b.projection_inverse); EXPECT_EQ(a.projected_omega,b.projected_omega);
  EXPECT_EQ(a.regular_rate,b.regular_rate); EXPECT_EQ(a.hourglass_rate,b.hourglass_rate);
  for (unsigned i=0;i<4;++i) {
    Near(a.local_position[i],b.local_position[i],0); Near(a.local_normals[i],b.local_normals[i],0);
    Near(a.projection_columns[i],b.projection_columns[i],0);
  }
  EXPECT_DOUBLE_EQ(a.base_time,b.base_time); EXPECT_DOUBLE_EQ(a.dt,b.dt); EXPECT_EQ(a.sample_index,b.sample_index);
}
inline void SameTrial(const ForceTrial& a,const ForceTrial& b) {
  SameHistory(a.proposed_history.data(),b.proposed_history.data()); SameGeometry(a.kinematics,b.kinematics);
  EXPECT_EQ(a.proposed_history.stamp().sample_index,b.proposed_history.stamp().sample_index);
  EXPECT_DOUBLE_EQ(a.proposed_history.stamp().time,b.proposed_history.stamp().time);
  for (unsigned i=0;i<4;++i) { Near(a.internal_force[i],b.internal_force[i]); Near(a.internal_couple[i],b.internal_couple[i]); }
  const auto& x=a.diagnostics; const auto& y=b.diagnostics;
  Near(x.effective_thickness,y.effective_thickness); Near(x.native_sound_speed,y.native_sound_speed);
  Near(x.membrane_viscosity,y.membrane_viscosity); Near(x.stabilization_viscosity,y.stabilization_viscosity);
  Near(x.translational_stiffness,y.translational_stiffness); Near(x.rotational_stiffness,y.rotational_stiffness);
  Near(x.unscaled_element_dt,y.unscaled_element_dt); Near(x.internal_work_increment,y.internal_work_increment,kEnergy);
  Near(x.hourglass_viscous_work_increment,y.hourglass_viscous_work_increment,kEnergy);
}
inline double Power(const ForceTrial& force,const PrescribedInterval& virtual_field) {
  double result=0;
  for (unsigned i=0;i<4;++i) result+=Dot(force.internal_force[i],virtual_field.velocity_midpoint[i])+
                                      Dot(force.internal_couple[i],virtual_field.omega_midpoint[i]);
  return result;
}
inline std::string Text(double x) { std::ostringstream s; s<<std::setprecision(17)<<x; return s.str(); }
}  // namespace qeph_force_test
