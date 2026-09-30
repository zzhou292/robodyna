#pragma once
// Test-only adapters; production never depends on native reference types.
#include "lib_src/elements/t3/T3Startup.h"
#include "lib_src/elements/t3/T3Kinematics.h"
#include "../native/t3/T3KinematicsTestOracle.h"
#include <limits>

namespace t3_port_test {
namespace port=tl::fea::t3;
namespace native=tl::qualification::t3;
namespace kt=native::kinematic_test;
using kt::Bytes;
inline port::ReferenceInput Input(const native::ReferenceInput& in) {
  port::ReferenceInput out;
  for(unsigned n=0;n<3;++n) { out.position[n]=in.position[n]; out.node_ids[n]=in.node_ids[n]; }
  out.density=in.density; out.thickness=in.thickness; out.young_modulus=in.young_modulus; out.poisson_ratio=in.poisson_ratio;
  return out;
}
inline native::ReferenceInput Native(const port::ReferenceInput& in) {
  native::ReferenceInput out;
  for(unsigned n=0;n<3;++n) { out.position[n]=in.position[n]; out.node_ids[n]=in.node_ids[n]; }
  out.density=in.density; out.thickness=in.thickness; out.young_modulus=in.young_modulus; out.poisson_ratio=in.poisson_ratio;
  return out;
}
inline port::PrescribedInterval Input(const native::PrescribedInterval& in) {
  port::PrescribedInterval out; out.base_time=in.base_time; out.dt=in.dt; out.sample_index=in.sample_index;
  for(unsigned n=0;n<3;++n) { out.position[n]=in.position[n]; out.velocity[n]=in.velocity[n]; out.angular_velocity[n]=in.angular_velocity[n]; }
  return out;
}
inline native::PrescribedInterval Native(const port::PrescribedInterval& in) {
  native::PrescribedInterval out; out.base_time=in.base_time; out.dt=in.dt; out.sample_index=in.sample_index;
  for(unsigned n=0;n<3;++n) { out.position[n]=in.position[n]; out.velocity[n]=in.velocity[n]; out.angular_velocity[n]=in.angular_velocity[n]; }
  return out;
}
inline native::Kinematics Native(const port::Kinematics& in) {
  native::Kinematics out; out.frame=in.frame; out.area=in.area; out.characteristic_length=in.characteristic_length;
  out.area_scale=in.area_scale; out.base_time=in.base_time; out.position_time=in.position_time;
  out.velocity_time=in.velocity_time; out.dt=in.dt; out.sample_index=in.sample_index; out.valid=in.valid;
  for(unsigned n=0;n<3;++n) { out.local_position[n]=in.local_position[n]; out.derivative[n]=in.derivative[n];
    out.corrected_velocity_difference[n]=in.corrected_velocity_difference[n]; }
  for(unsigned i=0;i<8;++i) { out.raw_rate[i]=in.raw_rate[i]; out.normalized_rate[i]=in.normalized_rate[i]; }
  return out;
}
inline port::ReferenceInput Triangle(double scale=1,unsigned shape=0) {
  auto in=kt::Triangle(scale);
  if(shape==1) in.position={{{0,0,0},{scale,0,0},{0,scale,0}}};
  if(shape==2) in.position={{{0,0,0},{scale,0,0},{.5*scale,std::sqrt(3.)*.5*scale,0}}};
  return Input(in);
}
inline port::ReferenceData Reference(const port::ReferenceInput& in) {
  port::ReferenceData r;
  if(port::InitializeReference(in,r)!=port::Status::kSuccess) throw std::runtime_error("Port startup rejected fixture");
  return r;
}
inline port::PrescribedInterval Interval(const port::ReferenceInput& in,double h=.01) {
  port::PrescribedInterval out; out.dt=h;
  for(unsigned n=0;n<3;++n) out.position[n]=in.position[n];
  return out;
}
inline void Set(port::Vec3& x,unsigned c,double value) { if(c==0)x.x=value; else if(c==1)x.y=value; else x.z=value; }
inline double Component(port::Vec3 x,unsigned c) { return c==0?x.x:c==1?x.y:x.z; }
inline void Near(double a,double expected,double scale) { native::test::Near(a,expected,scale); }
inline void Vector(port::Vec3 a,port::Vec3 b,double scale) {
  for(unsigned c=0;c<3;++c) Near(Component(a,c),Component(b,c),scale);
}
inline void StartupAgreement(const port::ReferenceData& a,const native::Reference& oracle) {
  ASSERT_TRUE(a.prepared); ASSERT_TRUE(oracle.prepared()); const auto& b=oracle.data();
  const auto independent=native::test::Independent(b.input); const double L=2*b.area/b.characteristic_length;
  for(unsigned i=0;i<9;++i) Near(a.frame.v[i],b.frame.v[i],1);
  Near(a.area,b.area,L*L); Near(a.characteristic_length,b.characteristic_length,L);
  Near(a.element_mass,b.element_mass,b.element_mass);
  Near(a.element_isotropic_inertia,b.element_isotropic_inertia,b.element_isotropic_inertia);
  Near(a.element_physical_inertia,b.element_physical_inertia,b.element_physical_inertia);
  Near(a.element_added_inertia,b.element_added_inertia,b.element_added_inertia);
  native::test::Near(a.element_mass,independent.mass,independent.mass);
  native::test::Near(a.element_isotropic_inertia,independent.total,independent.total);
  for(unsigned n=0;n<3;++n) {
    Vector(a.local_position[n],b.local_position[n],L);
    Near(a.angle_cosine[n],b.angle_cosine[n],1); Near(a.angle_weight[n],b.angle_weight[n],1);
    Near(a.nodal_mass[n],b.nodal_mass[n],b.nodal_mass[n]);
    Near(a.physical_inertia[n],b.physical_inertia[n],b.physical_inertia[n]);
    Near(a.added_inertia[n],b.added_inertia[n],b.added_inertia[n]);
    Near(a.isotropic_inertia[n],b.isotropic_inertia[n],b.isotropic_inertia[n]);
    EXPECT_EQ(a.startup_derivative[n],0.);
    native::test::Near(a.angle_weight[n],independent.weight[n],1);
    native::test::Near(a.nodal_mass[n],independent.mass*independent.weight[n],independent.mass*independent.weight[n]);
    EXPECT_EQ(a.input.node_ids[n],b.input.node_ids[n]); Vector(a.input.position[n],b.input.position[n],0);
  }
  EXPECT_EQ(a.input.density,b.input.density); EXPECT_EQ(a.input.thickness,b.input.thickness);
  EXPECT_EQ(a.input.young_modulus,b.input.young_modulus); EXPECT_EQ(a.input.poisson_ratio,b.input.poisson_ratio);
}
inline void RatesAgreement(const port::Kinematics& a,const native::Kinematics& b,const port::PrescribedInterval& in) {
  const auto wide=kt::Independent(Native(in)); const double L=wide.scale_length,V=wide.velocity+L*wide.omega;
  ASSERT_TRUE(a.valid); ASSERT_TRUE(b.valid);
  for(unsigned i=0;i<9;++i) Near(a.frame.v[i],b.frame.v[i],1);
  Near(a.area,b.area,L*L); Near(a.characteristic_length,b.characteristic_length,L); EXPECT_EQ(a.area_scale,b.area_scale);
  for(unsigned n=0;n<3;++n) {
    Vector(a.local_position[n],b.local_position[n],L); Near(a.derivative[n],b.derivative[n],L);
    Near(a.corrected_velocity_difference[n],b.corrected_velocity_difference[n],V);
  }
  for(unsigned c=0;c<8;++c) { const double scale=c<5?L*V:L*wide.omega;
    Near(a.raw_rate[c],b.raw_rate[c],scale); Near(a.normalized_rate[c],b.normalized_rate[c],scale/wide.area); }
  EXPECT_EQ(a.base_time,b.base_time); EXPECT_EQ(a.position_time,b.position_time); EXPECT_EQ(a.velocity_time,b.velocity_time);
  EXPECT_EQ(a.dt,b.dt); EXPECT_EQ(a.sample_index,b.sample_index);
  kt::Check(Native(a),Native(in)); // Independent affine/angular contraction of the PORT result.
}
inline void CheckStartup(const port::ReferenceInput& in) {
  native::Reference oracle; ASSERT_EQ(native::Initialize(Native(in),oracle),native::Status::kSuccess);
  port::ReferenceData r; ASSERT_EQ(port::InitializeReference(in,r),port::Status::kSuccess);
  StartupAgreement(r,oracle);
}
inline void CheckRates(const port::ReferenceData& reference,const port::PrescribedInterval& in) {
  native::Reference nr; ASSERT_EQ(native::Initialize(Native(reference.input),nr),native::Status::kSuccess);
  native::Kinematics truth; ASSERT_EQ(native::EvaluatePrescribed(nr,Native(in),truth),native::Status::kSuccess);
  port::Kinematics result; ASSERT_EQ(port::EvaluatePrescribed(reference,in,result),port::Status::kSuccess);
  RatesAgreement(result,truth,in);
}
inline void ExactRates(const port::Kinematics& a,const port::Kinematics& b) {
  for(unsigned c=0;c<9;++c) EXPECT_EQ(a.frame.v[c],b.frame.v[c]);
  for(unsigned n=0;n<3;++n) {
    for(unsigned c=0;c<3;++c) EXPECT_EQ(Component(a.local_position[n],c),Component(b.local_position[n],c));
    EXPECT_EQ(a.derivative[n],b.derivative[n]);
    EXPECT_EQ(a.corrected_velocity_difference[n],b.corrected_velocity_difference[n]);
  }
  for(unsigned c=0;c<8;++c) { EXPECT_EQ(a.raw_rate[c],b.raw_rate[c]); EXPECT_EQ(a.normalized_rate[c],b.normalized_rate[c]); }
  EXPECT_EQ(a.area,b.area); EXPECT_EQ(a.characteristic_length,b.characteristic_length); EXPECT_EQ(a.area_scale,b.area_scale);
  EXPECT_EQ(a.base_time,b.base_time); EXPECT_EQ(a.position_time,b.position_time); EXPECT_EQ(a.velocity_time,b.velocity_time);
  EXPECT_EQ(a.dt,b.dt); EXPECT_EQ(a.sample_index,b.sample_index); EXPECT_EQ(a.valid,b.valid);
}
} // namespace t3_port_test
