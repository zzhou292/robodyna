// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/elements/type25/Type25Math.h"
#include <gtest/gtest.h>
#include <cfloat>
#include <cstring>
#include <limits>

namespace type25_test {
namespace {
using L=long double;
struct V { L x,y,z; };
V Lift(spring::Vec3 v) { return {v.x,v.y,v.z}; }
V Add(V a,V b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
V Sub(V a,V b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
V Cross(V a,V b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
L Dot(V a,V b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
}
TEST(Type25Math,IndependentAngularMomentumAndVirtualWorkIncludeBothShearArms) {
  const spring::Vec3 points[2]{{.5,-.2,.3},{.6,-.2,.3}};spring::Reference ref;
  ASSERT_EQ(spring::InitializeReference({1,1,1},points,{},ref),spring::Status::Success);
  auto p=Property();spring::History old;old.transverse_axis=ref.transverse_axis;
  spring::EndpointKinematics nodes[2]{{points[0],{1,2,3},{.4,-.8,1.2}},
      {points[1],{2,3,4},{-.3,.5,.9}}};
  spring::Evaluation e;ASSERT_EQ(spring::Evaluate({1,1,1},p,ref,old,nodes,1e-7,e),spring::Status::Success);
  V torque{};L power=0,magnitude=0;
  for(unsigned i=0;i<2;++i) {
    const auto f=Lift(e.endpoints[i].force_N),m=Lift(e.endpoints[i].couple_Nm);
    torque=Add(torque,Add(Cross(Lift(nodes[i].position),f),m));
    power+=Dot(f,Lift(nodes[i].velocity))+Dot(m,Lift(nodes[i].angular_velocity));
    magnitude+=std::abs(f.x)+std::abs(f.y)+std::abs(f.z)+std::abs(m.x)+std::abs(m.y)+std::abs(m.z);
  }
  EXPECT_NEAR(static_cast<double>(torque.x),0,128*DBL_EPSILON*static_cast<double>(magnitude));
  EXPECT_NEAR(static_cast<double>(torque.y),0,128*DBL_EPSILON*static_cast<double>(magnitude));
  EXPECT_NEAR(static_cast<double>(torque.z),0,128*DBL_EPSILON*static_cast<double>(magnitude));
  // Independent wrench virtual work at the CURRENT endpoint frame. This does
  // not substitute a differential work identity for native radial history.
  const auto f=Lift(e.endpoints[0].force_N);
  const auto localm=e.history.local_couple_Nm;
  const V m{e.frame.axes.v[0]*L(localm.x)+e.frame.axes.v[1]*L(localm.y)+e.frame.axes.v[2]*L(localm.z),
      e.frame.axes.v[3]*L(localm.x)+e.frame.axes.v[4]*L(localm.y)+e.frame.axes.v[5]*L(localm.z),
      e.frame.axes.v[6]*L(localm.x)+e.frame.axes.v[7]*L(localm.y)+e.frame.axes.v[8]*L(localm.z)};
  const auto chord=Sub(Lift(nodes[1].position),Lift(nodes[0].position));
  const auto sumw=Add(Lift(nodes[0].angular_velocity),Lift(nodes[1].angular_velocity));
  const L expected=Dot(f,Sub(Lift(nodes[0].velocity),Lift(nodes[1].velocity)))+
      Dot(m,Sub(Lift(nodes[0].angular_velocity),Lift(nodes[1].angular_velocity)))+.5L*Dot(Cross(chord,f),sumw);
  EXPECT_NEAR(static_cast<double>(power-expected),0,128*DBL_EPSILON*static_cast<double>(magnitude));
}
TEST(Type25Math,TranslationAndCommonAxialSpinHaveZeroDeformation) {
  const spring::Vec3 points[2]{{},{.125,0,0}};spring::Reference ref;
  ASSERT_EQ(spring::InitializeReference({1,1,1},points,{},ref),spring::Status::Success);
  spring::History old;old.transverse_axis=ref.transverse_axis;
  spring::EndpointKinematics nodes[2]{{points[0],{8,2,-3},{4,0,0}},{points[1],{8,2,-3},{4,0,0}}};
  spring::Evaluation e;ASSERT_EQ(spring::Evaluate({1,1,1},Property(),ref,old,nodes,.01,e),spring::Status::Success);
  EXPECT_DOUBLE_EQ(tl::math::Norm(e.history.displacement_m),0);EXPECT_DOUBLE_EQ(tl::math::Norm(e.history.rotation_rad),0);
  EXPECT_DOUBLE_EQ(tl::math::Norm(e.endpoints[0].force_N),0);EXPECT_DOUBLE_EQ(tl::math::Norm(e.endpoints[0].couple_Nm),0);
  EXPECT_NEAR(e.history.transverse_axis.y,std::cos(.04),2*DBL_EPSILON);
  EXPECT_NEAR(e.history.transverse_axis.z,std::sin(.04),2*DBL_EPSILON);
}
TEST(Type25Math,ShearArmLowersRotationalStepAndMatchesIndependentLongDoubleFormula) {
  auto p=Property();spring::Stability short_step,long_step;
  ASSERT_EQ(spring::CriticalStep({1000,.001,1},p,.003,short_step),spring::Status::Success);
  ASSERT_EQ(spring::CriticalStep({1000,.001,1},p,.02,long_step),spring::Status::Success);
  EXPECT_LT(long_step.critical_dt_s,short_step.critical_dt_s);
  const L kr=L(p.stiffness[1])*.02L*.02L+std::max(L(p.stiffness[2]),L(p.stiffness[3]));
  const L expected=std::min(std::sqrt(L(p.mass_kg)/p.stiffness[0]),std::sqrt(L(p.isotropic_inertia_kg_m2)/kr));
  EXPECT_NEAR(long_step.critical_dt_s,static_cast<double>(expected),8*DBL_EPSILON*static_cast<double>(expected));
}
TEST(Type25Math,EveryLateFailurePreservesFullOutputAndCleanRetry) {
  Fixture f;const auto& c=f.connections[2];spring::Reference ref;
  ASSERT_EQ(spring::InitializeReference(f.Input().source_units,c.position,c.seed,ref),spring::Status::Success);
  spring::History old;old.transverse_axis=ref.transverse_axis;
  for(unsigned mutation=0;mutation<9;++mutation) {
    auto p=Property();auto reference=ref;auto history=old;double dt=1e-7;
    spring::EndpointKinematics nodes[2]{{c.position[0],{},{}},{c.position[1],{1,2,3},{.3,.4,.5}}};
    if(mutation==0)p.stiffness[3]=DBL_MAX;
    if(mutation==1)history.internal_work_J[3]=std::numeric_limits<double>::quiet_NaN();
    if(mutation==2)nodes[1].position=nodes[0].position;
    if(mutation==3)nodes[1].angular_velocity.z=std::numeric_limits<double>::infinity();
    if(mutation==4)dt=0;
    if(mutation==5)history.active=false;
    if(mutation==6)p.failure_positive[3]=std::numeric_limits<double>::denorm_min();
    if(mutation==7)reference.length_m=std::nextafter(reference.length_m,1.);
    if(mutation==8)reference.transverse_axis=tl::math::Divide(tl::math::Subtract(c.position[1],c.position[0]),reference.length_m);
    spring::Evaluation out;out.critical_dt_s=42;const auto saved=out;
    EXPECT_NE(spring::Evaluate(f.Input().source_units,p,reference,history,nodes,dt,out),spring::Status::Success)<<mutation;
    EXPECT_EQ(std::memcmp(&saved,&out,sizeof(out)),0)<<mutation;
    spring::EndpointKinematics clean[2]{{c.position[0],{},{}},{c.position[1],{1,2,3},{.3,.4,.5}}};
    EXPECT_EQ(spring::Evaluate(f.Input().source_units,Property(),ref,old,clean,1e-7,out),spring::Status::Success);
  }
}
} // namespace type25_test
