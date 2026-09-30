// SPDX-License-Identifier: MIT
#include "NativeValues.h"
#include "lib_src/elements/type25/mapped/Stiffness.h"
#include <gtest/gtest.h>
namespace type25_mapped_test {
namespace spring=tl::fea::type25;
namespace mapped=spring::mapped;
TEST(Type25MappedNative, VirginPreFloorCoefficientsUseSourceUnitsAndIgnoreCurrentMassAtZeroDamping) {
  for (const auto units:{spring::SourceUnits{1,1,1},spring::SourceUnits{1000,.001,1}}) {
    for (double k:{1e-20,1000.}) {
      auto property=type25_test::Property();
      for (auto& value:property.stiffness) value=k;
      spring::Stability coefficient;
      ASSERT_EQ(spring::CriticalStep(units,property,.025,coefficient),spring::Status::Success);
      spring::Evaluation evaluation;
      evaluation.translation_stiffness_N_per_m=coefficient.translation_stiffness_N_per_m;
      evaluation.rotation_stiffness_Nm_per_rad=coefficient.rotation_stiffness_Nm_per_rad;
      for (bool active:{true,false}) {
        evaluation.history.active=active;
        mapped::NodalStiffness actual;
        ASSERT_TRUE(mapped::AcceptedStiffness(property,evaluation,actual));
        for (unsigned mode=0;mode<3;++mode) {
          const auto expected=NativeStiffness(units,property,.025,active,mode);
          EXPECT_DOUBLE_EQ(actual.translation,expected[0]);
          EXPECT_DOUBLE_EQ(actual.translation,expected[1]);
          EXPECT_DOUBLE_EQ(actual.rotation,expected[2]);
          EXPECT_DOUBLE_EQ(actual.rotation,expected[3]);
        }
      }
    }
  }
}
TEST(Type25MappedNative, PositiveDampingDistinguishesNativeNodalFromElementCoefficient) {
  auto property=type25_test::Property();
  for (auto& damping:property.damping) damping=2;
  const auto native=NativeStiffness({1,1,1},property,.025,true,1);
  spring::Stability coefficient;
  ASSERT_EQ(spring::CriticalStep({1,1,1},property,.025,coefficient),spring::Status::Success);
  EXPECT_GT(native[0],coefficient.translation_stiffness_N_per_m);
  EXPECT_NE(native[0],native[1]);
  spring::Evaluation evaluation;
  evaluation.translation_stiffness_N_per_m=coefficient.translation_stiffness_N_per_m;
  evaluation.rotation_stiffness_Nm_per_rad=coefficient.rotation_stiffness_Nm_per_rad;
  mapped::NodalStiffness unchanged{17,19};
  EXPECT_FALSE(mapped::AcceptedStiffness(property,evaluation,unchanged));
  EXPECT_EQ(unchanged.translation,17);
}
TEST(Type25MappedNative, NativeFailureIntervalAndTwoInactiveIntervalsKeepSeparateForceAndStiffness) {
  const spring::SourceUnits units{1,1,1};
  auto p=type25_test::Property();
  for (auto& k:p.stiffness) k=1000;
  for (auto& value:p.failure_positive) value=1e-7;
  const spring::Vec3 x[2]{{0,0,0},{.01,0,0}};
  spring::Reference ref;
  ASSERT_EQ(spring::InitializeReference(units,x,{},ref),spring::Status::Success);
  spring::History history;
  history.transverse_axis=ref.transverse_axis;
  auto native_history=history;
  spring::EndpointKinematics nodes[2]{{x[0],{},{}},{{.011,0,0},{1,.2,0},{.1,.2,.3}}};
  for (unsigned interval=0;interval<3;++interval) {
    spring::Evaluation actual;
    ASSERT_EQ(spring::Evaluate(units,p,ref,history,nodes,.001,actual),spring::Status::Success);
    const auto expected=type25_test::NativeEvaluate(units,p,ref,native_history,nodes,.001);
    CompareNative(actual,expected,units,p,nodes,.001);
    ASSERT_FALSE(actual.history.active);
    if (!interval) EXPECT_GT(tl::math::fixed3::Norm(actual.endpoints[0].force_N),0);
    else EXPECT_EQ(tl::math::fixed3::Norm(actual.endpoints[0].force_N),0);
    mapped::NodalStiffness stiffness;
    ASSERT_TRUE(mapped::AcceptedStiffness(p,actual,stiffness));
    const auto nodal=NativeStiffness(units,p,expected.frame.length_m,expected.history.active,2);
    EXPECT_EQ(stiffness.translation,nodal[0]);
    EXPECT_EQ(stiffness.rotation,nodal[2]);
    history=actual.history;
    native_history=expected.history;
    nodes[1].position.x+=.001;
    nodes[1].position.y+=.0002;
  }
}
TEST(Type25MappedNative, OrderedSharedNodeScatterMatchesOriginalIndependentArrays) {
  const int nodes[6]{3,1,1,2,3,2};
  const int count=3,elements=3;
  const double values[12]{1,1,2,2,3,3,4,4,5,5,6,6};
  double native_t[3]{1e16,1,0},native_r[3]{};
  mapped_type25_native_scatter(&count,&elements,nodes,values,native_t,native_r);
  double actual_t[3]{1e16,1,0},actual_r[3]{};
  for (unsigned e=0;e<3;++e) {
    const std::size_t mapped_nodes[2]{std::size_t(nodes[2*e]-1),std::size_t(nodes[2*e+1]-1)};
    ASSERT_TRUE(mapped::AddStiffness(mapped_nodes,{values[4*e],values[4*e+2]},actual_t,actual_r,3));
  }
  for (unsigned n=0;n<3;++n) {
    EXPECT_DOUBLE_EQ(actual_t[n],native_t[n]);
    EXPECT_DOUBLE_EQ(actual_r[n],native_r[n]);
  }
}
}
