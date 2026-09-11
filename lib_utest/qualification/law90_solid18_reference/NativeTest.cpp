// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "Agreement.h"
using namespace law90_reference_test;
namespace {
void Check(const s::ReferenceInput& input) {
  const auto expected=ReferenceOracle(input);
  ASSERT_EQ(expected.status,0);
  const int tags[4]={10,0,72,6};
  for(unsigned k=0;k<4;++k) ASSERT_EQ(expected.allocation[k],tags[k]);
  t::Reference reference;
  ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
  for(unsigned n=0;n<8;++n) ASSERT_EQ(reference.source_slot(n),expected.source_slot[n]);
  ASSERT_TRUE(ReferenceAgreement(ReferenceValues(reference),expected.values));
}
}
TEST(Law90Solid18Native, CompleteAllocationAllPijAndSourceOrientation) {
  for(auto input:{Cube(),Distorted()}) {
    Check(input);
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    for(unsigned n=0;n<4;++n) {
      std::swap(input.position_m[n],input.position_m[n+4]);
      std::swap(input.source_node_id[n],input.source_node_id[n+4]);
    }
    Check(input);
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
  }
}
TEST(Law90Solid18Native, RotatedDistortedCurrentPlanesRatesAndUnchangedStorage) {
  auto input=Distorted();
  for(auto& x:input.position_m) {
    const auto old=x;
    x={.8*old.x-.6*old.y+13, .6*old.x+.8*old.y-21, old.z+7};
  }
  Check(input);
  ASSERT_FALSE(::testing::Test::HasFatalFailure());
  t::Reference reference;
  ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
  const auto reference_before=Bytes(reference);
  for(unsigned step=0;step<=64;++step) {
    SCOPED_TRACE(step);
    const auto current=step ? Path(input,.4*std::sin(step*.043)) : Current(input);
    const auto expected=CurrentOracle(input,current);
    ASSERT_EQ(expected.status,0);
    t::Kinematics actual;
    ASSERT_EQ(t::EvaluateKinematics90(reference,current,actual),s::Status::Success);
    ASSERT_TRUE(CurrentAgreement(CurrentValues(actual),expected.values));
    ASSERT_EQ(Bytes(reference),reference_before);
  }
}
TEST(Law90Solid18Native, ReversedSourceSnapshotStillMatchesCurrentCaller) {
  auto input=Distorted();
  for(unsigned n=0;n<4;++n) {
    std::swap(input.position_m[n],input.position_m[n+4]);
    std::swap(input.source_node_id[n],input.source_node_id[n+4]);
  }
  t::Reference reference;
  ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
  const auto current=Current(input);
  const auto expected=CurrentOracle(input,current);
  ASSERT_EQ(expected.status,0);
  t::Kinematics actual;
  ASSERT_EQ(t::EvaluateKinematics90(reference,current,actual),s::Status::Success);
  ASSERT_TRUE(CurrentAgreement(CurrentValues(actual),expected.values));
}
TEST(Law90Solid18Native, DimensionalGroupsRejectPerturbedPijTensorAndRate) {
  const auto input=Distorted();
  const auto expected=ReferenceOracle(input);
  ASSERT_EQ(expected.status,0);
  for(unsigned index:{148u,157u,158u,190u,733u,754u}) {
    auto bad=expected.values;
    bad[index]+=(1+std::abs(bad[index]))*1e-6;
    EXPECT_FALSE(ReferenceAgreement(bad,expected.values));
  }
  const auto current=CurrentOracle(input,Path(input,.4));
  ASSERT_EQ(current.status,0);
  for(unsigned index:{0u,30u,58u,70u,100u,726u,758u,991u-1}) {
    auto bad=current.values;
    bad[index]+=(1+std::abs(bad[index]))*1e-6;
    EXPECT_FALSE(CurrentAgreement(bad,current.values));
  }
}
