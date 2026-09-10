// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cfloat>
#include <cmath>

namespace type25_test {
namespace {
void Near(double actual,double expected,double conversion_allowance=0) {
  EXPECT_NEAR(actual,expected,conversion_allowance+5e-10*std::max({std::abs(actual),std::abs(expected),1e-20}));
}
void Near(spring::Vec3 a,spring::Vec3 b) { Near(a.x,b.x);Near(a.y,b.y);Near(a.z,b.z); }
void WrenchNear(spring::Vec3 a,spring::Vec3 b,double allowance) {
  Near(a.x,b.x,allowance);Near(a.y,b.y,allowance);Near(a.z,b.z,allowance);
}
void Compare(const spring::Evaluation& a,const spring::Evaluation& b,double force_allowance=0) {
  for(unsigned i=0;i<9;++i){Near(a.frame.axes.v[i],b.frame.axes.v[i]);Near(a.frame.midpoint_axes.v[i],b.frame.midpoint_axes.v[i]);}
  Near(a.frame.length_m,b.frame.length_m);Near(a.frame.midpoint_length_m,b.frame.midpoint_length_m);
  Near(a.history.transverse_axis,b.history.transverse_axis);Near(a.history.displacement_m,b.history.displacement_m);
  Near(a.history.rotation_rad,b.history.rotation_rad);
  {SCOPED_TRACE("local_force_N");WrenchNear(a.history.local_force_N,b.history.local_force_N,force_allowance);}
  Near(a.history.local_couple_Nm,b.history.local_couple_Nm);
  for(unsigned i=0;i<4;++i)Near(a.history.internal_work_J[i],b.history.internal_work_J[i]);
  EXPECT_EQ(a.history.active,b.history.active);Near(a.history.failure_criterion,b.history.failure_criterion);
  for(unsigned i=0;i<2;++i){WrenchNear(a.endpoints[i].force_N,b.endpoints[i].force_N,force_allowance);
    WrenchNear(a.endpoints[i].couple_Nm,b.endpoints[i].couple_Nm,force_allowance*a.frame.length_m);}
  Near(a.critical_dt_s,b.critical_dt_s);Near(a.translation_stiffness_N_per_m,b.translation_stiffness_N_per_m);
  Near(a.rotation_stiffness_Nm_per_rad,b.rotation_stiffness_Nm_per_rad);
}
}
TEST(Type25Native,ReferenceFrameUsesExactFiniteOffsetAndSeedFallbackOperations) {
  const double seed[6]={1,0,0,0,1,0};
  for(const spring::Vec3 chord:{spring::Vec3{.007,.002,.001},spring::Vec3{0,.01,0},
      spring::Vec3{9e-8,.01,0},spring::Vec3{1.1e-7,.01,0}}) {
    const spring::Vec3 p[2]{{.5,.3,.1},{.5+chord.x,.3+chord.y,.1+chord.z}};
    const double packed[6]={p[0].x,p[0].y,p[0].z,p[1].x,p[1].y,p[1].z};double expected[4];
    type25_native_reference(packed,seed,expected);
    spring::Reference ref;ASSERT_EQ(spring::InitializeReference({1,1,1},p,{},ref),spring::Status::Success);
    EXPECT_DOUBLE_EQ(ref.transverse_axis.x,expected[0]);EXPECT_DOUBLE_EQ(ref.transverse_axis.y,expected[1]);
    EXPECT_DOUBLE_EQ(ref.transverse_axis.z,expected[2]);EXPECT_DOUBLE_EQ(ref.length_m,expected[3]);
  }
}
TEST(Type25Native,FullFramesDeformationLinearChannelsScatterAndHistoryAcrossSourceUnits) {
  for(const spring::SourceUnits units: {spring::SourceUnits{1,1,1},spring::SourceUnits{1000,.001,1},spring::SourceUnits{1,.001,.001}}) {
    Fixture f;const auto& c=f.connections[2];auto p=Property();
    for(unsigned i=0;i<4;++i)p.damping[i]=i<2?2:.02;
    spring::Reference ref;ASSERT_EQ(spring::InitializeReference(units,c.position,c.seed,ref),spring::Status::Success);
    spring::History old;old.transverse_axis=ref.transverse_axis;auto native_old=old;
    spring::EndpointKinematics nodes[2]{{c.position[0],{},{}},{c.position[1],{},{}}};
    const double dt=1e-7;
    for(unsigned step=0;step<64;++step) {
      SCOPED_TRACE(step);
      SCOPED_TRACE(units.length_to_m);
      for(unsigned k=0;k<2;++k) {
        const double a=.04*step+.2*k;
        nodes[k].velocity={1+std::sin(a),.5+std::cos(a),.2+std::sin(2*a)};
        nodes[k].angular_velocity={.4+std::sin(a),-.3+std::cos(a),.8+std::sin(2*a)};
        nodes[k].position.x+=dt*nodes[k].velocity.x;nodes[k].position.y+=dt*nodes[k].velocity.y;nodes[k].position.z+=dt*nodes[k].velocity.z;
      }
      spring::Evaluation actual;ASSERT_EQ(spring::Evaluate(units,p,ref,old,nodes,dt,actual),spring::Status::Success);
      const auto expected=NativeEvaluate(units,p,ref,native_old,nodes,dt);
      if(units.mass_to_kg==1&&units.length_to_m==1&&units.time_to_s==1) {
        EXPECT_DOUBLE_EQ(actual.history.local_force_N.x,expected.history.local_force_N.x);
        EXPECT_DOUBLE_EQ(actual.history.local_force_N.y,expected.history.local_force_N.y);
        EXPECT_DOUBLE_EQ(actual.history.local_force_N.z,expected.history.local_force_N.z);
        for(unsigned a=0;a<4;++a)EXPECT_DOUBLE_EQ(actual.history.internal_work_J[a],expected.history.internal_work_J[a]);
      }
      // SI and source-mm coordinate subtraction follow different binary64
      // rounding paths. A cancelling local/world force component needs an absolute
      // geometric conditioning allowance, K*deltaX + C*deltaX/dt. This is only
      // an oracle conversion enclosure; it changes no production tolerance.
      double coordinate_scale=ref.length_m;
      for(const auto& node:nodes)coordinate_scale=std::max({coordinate_scale,std::abs(node.position.x),std::abs(node.position.y),std::abs(node.position.z)});
      const double conversion=units.length_to_m==1?0:64*DBL_EPSILON*coordinate_scale*
          (std::max(p.stiffness[0],p.stiffness[1])+std::max(p.damping[0],p.damping[1])/dt);
      Compare(actual,expected,conversion);
      old=actual.history;native_old=expected.history;
    }
  }
}
TEST(Type25Native,HalfPropertyMassAndInertiaAndRegularizedStep) {
  auto p=Property();double native[2];type25_native_mass(&p.mass_kg,&p.isotropic_inertia_kg_m2,native);
  spring::MassCoefficients m;ASSERT_EQ(spring::EndpointCoefficients(p,m),spring::Status::Success);
  EXPECT_DOUBLE_EQ(m.mass_kg,native[0]);EXPECT_DOUBLE_EQ(m.isotropic_inertia_kg_m2,native[1]);
  for(double stiffness:{1e-20,1e-15,1e-12,1e8}) {
    for(unsigned i=0;i<4;++i){p.stiffness[i]=stiffness;p.damping[i]=stiffness/10;}
    const double length=.25,kt=stiffness,kr=stiffness+stiffness*length*length;
    const double ct=stiffness/10,cr=ct+ct*length*length;double expected;
    type25_native_dt(&p.mass_kg,&p.isotropic_inertia_kg_m2,&kt,&kr,&ct,&cr,&expected);
    spring::Stability stable;ASSERT_EQ(spring::CriticalStep({1,1,1},p,length,stable),spring::Status::Success);
    EXPECT_DOUBLE_EQ(stable.critical_dt_s,expected);
  }
}
TEST(Type25Native,CoupledStrictFailureRetainsCurrentForceThenZerosNextEvaluation) {
  auto p=Property();for(unsigned i=0;i<4;++i)p.stiffness[i]=8;
  const spring::Vec3 original[2]{{},{1,0,0}};spring::Reference ref;
  ASSERT_EQ(spring::InitializeReference({1,1,1},original,{},ref),spring::Status::Success);
  spring::History old;old.transverse_axis=ref.transverse_axis;
  spring::EndpointKinematics nodes[2]{{original[0],{},{}},{{1.125,0,0},{},{}}};
  p.failure_positive[0]=1;spring::Evaluation at;
  ASSERT_EQ(spring::Evaluate({1,1,1},p,ref,old,nodes,.01,at),spring::Status::Success);
  EXPECT_TRUE(at.history.active);EXPECT_DOUBLE_EQ(at.history.failure_criterion,1);
  p.failure_positive[0]=std::nextafter(1.,0.);
  spring::Evaluation failed;ASSERT_EQ(spring::Evaluate({1,1,1},p,ref,old,nodes,.01,failed),spring::Status::Success);
  Compare(failed,NativeEvaluate({1,1,1},p,ref,old,nodes,.01));
  EXPECT_FALSE(failed.history.active);EXPECT_DOUBLE_EQ(failed.history.local_force_N.x,1);
  nodes[1].position.x=1.25;spring::Evaluation after;
  ASSERT_EQ(spring::Evaluate({1,1,1},p,ref,failed.history,nodes,.01,after),spring::Status::Success);
  Compare(after,NativeEvaluate({1,1,1},p,ref,failed.history,nodes,.01));
  EXPECT_FALSE(after.history.active);EXPECT_DOUBLE_EQ(after.history.local_force_N.x,0);
  EXPECT_DOUBLE_EQ(after.history.internal_work_J[0],failed.history.internal_work_J[0]+.0625);
}
} // namespace type25_test
