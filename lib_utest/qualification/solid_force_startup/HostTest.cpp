// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../solid18_force/TestSupport.h"
#include "../solid24_force/TestSupport.h"
#include "../solid6z_force/TestSupport.h"
#include "lib_src/elements/solids/ForceStiffness.h"
#include <gtest/gtest.h>
#include <limits>

namespace {
namespace a=tl::fea::solid18;
namespace b=tl::fea::solid24;
namespace c=tl::fea::solid6z;
namespace solids=tl::fea::solids;
TEST(SolidForceStartup, ConstructorsHaveSampleZeroAndPositiveNativeStiffness) {
  const auto ar=solid18_force_test::Reference();
  const auto am=solid18_force_test::Material();
  const auto br=heph_test::Reference(); const auto bm=heph_test::Material(br.input().density_kg_m3);
  const auto cr=solid6z_force_test::Reference(); const auto cm=solid6z_force_test::Material();
  for (const a::Vec3 velocity : {a::Vec3{},a::Vec3{11.123,-.37,.129}}) {
    a::ForceTrial av; b::ForceTrial bv; c::ForceTrial cv;
    ASSERT_EQ(a::InitializeForce(ar,am,velocity,av),a::Status::Success);
    ASSERT_EQ(b::InitializeForce(br,bm,velocity,bv),b::ForceStatus::Success);
    ASSERT_EQ(c::InitializeForce(cr,cm,{},velocity,cv),c::Status::Success);
    EXPECT_EQ(av.proposed_history.stamp().time_s,0);
    EXPECT_EQ(bv.proposed_history.stamp().time_s,0);
    EXPECT_EQ(cv.proposed_history.stamp().time_s,0);
    EXPECT_EQ(av.proposed_history.stamp().sample_index,0u);
    EXPECT_EQ(bv.proposed_history.stamp().sample_index,0u);
    EXPECT_EQ(cv.proposed_history.stamp().sample_index,0u);
    solids::NodalStiffness value;
    ASSERT_TRUE(solids::PrepareNodalStiffness(av,value));
    EXPECT_EQ(value.translation_n_m,.25*av.diagnostics.raw_stiffness_n_m);
    EXPECT_EQ(value.rotation_nm,0);
    ASSERT_TRUE(solids::PrepareNodalStiffness(bv,value));
    EXPECT_EQ(value.translation_n_m,.25*bv.diagnostics.material.raw_stiffness_n_m);
    ASSERT_TRUE(solids::PrepareNodalStiffness(cv,value));
    EXPECT_EQ(value.translation_n_m,(1.0/3.0)*cv.material.raw_stiffness_n_m);
  }
}
TEST(SolidForceStartup, InitializationRejectsLateNonfiniteAndPreservesOutput) {
  const auto reference=heph_test::Reference(); const auto material=heph_test::Material(reference.input().density_kg_m3);
  b::ForceTrial value;
  ASSERT_EQ(b::InitializeForce(reference,material,{},value),b::ForceStatus::Success);
  const auto saved=value;
  EXPECT_EQ(b::InitializeForce(reference,material,{0,0,std::numeric_limits<double>::quiet_NaN()},value),
            b::ForceStatus::InvalidInput);
  EXPECT_TRUE(heph_test::Same(value,saved));
  auto wrong=material; wrong.density_kg_m3*=2;
  EXPECT_NE(b::InitializeForce(reference,wrong,{},value),b::ForceStatus::Success);
  EXPECT_TRUE(heph_test::Same(value,saved));
  ASSERT_EQ(b::InitializeForce(reference,material,{},value),b::ForceStatus::Success);
  EXPECT_TRUE(heph_test::Same(value,saved));
  auto interval=heph_test::Interval(reference,value.proposed_history,0);
  EXPECT_EQ(b::EvaluateForce(reference,value.proposed_history,interval,material,value),b::ForceStatus::InvalidInput);
  EXPECT_TRUE(heph_test::Same(value,saved));
  solids::NodalStiffness stiffness{71,72};
  auto invalid=value; invalid.diagnostics.material.raw_stiffness_n_m=std::numeric_limits<double>::infinity();
  EXPECT_FALSE(solids::PrepareNodalStiffness(invalid,stiffness));
  EXPECT_EQ(stiffness.translation_n_m,71); EXPECT_EQ(stiffness.rotation_nm,72);
}
TEST(SolidForceStartup, ConstructorMaterialEntryDoesNotRelaxLegacyZeroStep) {
  namespace m=tl::material::law42;
  const auto material=heph_test::Material();
  m::CallerInput input; input.current_volume_m3=1;input.storage_volume_m3=1;input.characteristic_length_m=1;
  m::CallerResult result;
  ASSERT_EQ(m::InitializeCaller(material,input,result),m::Status::Ok);
  const auto saved=law42_caller_test::Values(result);
  EXPECT_EQ(m::UpdateCaller(material,result.history,input,result),m::Status::InvalidInput);
  EXPECT_EQ(law42_caller_test::Values(result),saved);
  input.dt_s=1e-6;
  EXPECT_EQ(m::InitializeCaller(material,input,result),m::Status::InvalidInput);
  EXPECT_EQ(law42_caller_test::Values(result),saved);
}
TEST(SolidForceStartup, Law36InitializationRetainsNativeDensityPressureAndRateAtSampleZero) {
  namespace m=tl::material::law36;
  const auto material=solid18_force_test::Material();
  m::Input input;
  input.relative_density=.125;
  input.kinematics.engineering_rate_per_s[0]=2.5;
  input.kinematics.engineering_rate_per_s[3]=-.5;
  m::Result result;
  ASSERT_EQ(m::Initialize(material,input,result),m::Status::Ok);
  for(unsigned k=0;k<3;++k)
    EXPECT_EQ(result.history.stress_pa[k],-material.bulk_pa*input.relative_density);
  for(double value:result.history.engineering_strain) EXPECT_EQ(value,0);
  EXPECT_GT(result.history.deviatoric_rate_per_s,0);
  EXPECT_EQ(result.history.plastic_strain,0);
  const auto saved=result;
  EXPECT_EQ(m::Update(material,m::History{},input,result),m::Status::InvalidInput);
  EXPECT_EQ(result.history.deviatoric_rate_per_s,saved.history.deviatoric_rate_per_s);
  EXPECT_EQ(result.history.stress_pa[2],saved.history.stress_pa[2]);
  input.kinematics.dt_s=1e-6;
  EXPECT_EQ(m::Initialize(material,input,result),m::Status::InvalidInput);
  EXPECT_EQ(result.history.stress_pa[2],saved.history.stress_pa[2]);
}
} // namespace
