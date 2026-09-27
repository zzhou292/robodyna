// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
namespace connector_operand_test {
template<class T> class ConnectorOperands : public ::testing::Test {};
using Families=::testing::Types<Type13,Type25>;
TYPED_TEST_SUITE(ConnectorOperands,Families);
TYPED_TEST(ConnectorOperands, ExactOrderedChannelsAndEachAcceptedEndpointWithNonzeroSeed) {
  Fixture<TypeParam> f; Compare(f);
  EXPECT_TRUE(f.state.control.diagnostics.valid);
  for(double minimum : {-0.0,0.0,-1.0,DBL_MIN,DBL_MAX,double(NAN)}) {
    TypeParam::Dt(f.trial[0])=minimum; Compare(f);
  }
}
TYPED_TEST(ConnectorOperands, GlobalElementFailurePriorityPrecedesEarlierDerivedOverflow) {
  Fixture<TypeParam> f(3);
  TypeParam::Work(f.trial[0],0)=DBL_MAX; TypeParam::Work(f.trial[1],0)=DBL_MAX;
  f.status[2]=TypeParam::Status::NonfiniteResult;
  Compare(f);
  EXPECT_EQ(f.state.control.element,2u);
  EXPECT_EQ(f.state.control.diagnostics.element_count,f.seed.element_count);
  EXPECT_EQ(Bits(f.state.control.diagnostics.internal_work_J[0]),Bits(f.seed.internal_work_J[0]));
  f.status[2]=TypeParam::Status::Success;
  Compare(f); EXPECT_FALSE(f.state.control.diagnostics.valid);
}
TYPED_TEST(ConnectorOperands, FailedRowsNeedNoModelTrialAcceptedOrNodalInputAndAreOverwritten) {
  typename TypeParam::State state;
  typename TypeParam::Status status=TypeParam::Status::NonfiniteResult;
  state.candidate_status=&status;
  const auto value=TypeParam::Prepare(state,{},0);
  for(double x:value.work) EXPECT_EQ(Bits(x),Bits(0));
  for(double x:value.increment) EXPECT_EQ(Bits(x),Bits(0));
  for(double x:value.kick) EXPECT_EQ(Bits(x),Bits(0));
  for(double x:value.drift) EXPECT_EQ(Bits(x),Bits(0));
  EXPECT_EQ(value.active,0); EXPECT_EQ(value.newly_failed,0); EXPECT_EQ(Bits(value.native_dt),Bits(0));
}
TYPED_TEST(ConnectorOperands, DerivedOverflowAndRetryKeepTheOriginalTerminalCheck) {
  Fixture<TypeParam> f(3);
  f.accepted[0].endpoints[0].force_N={DBL_MAX,DBL_MAX,DBL_MAX};
  std::fill(f.v.begin(),f.v.end(),DBL_MAX);
  Compare(f); EXPECT_FALSE(f.state.control.diagnostics.valid);
  f.status[1]=TypeParam::Status::DegenerateGeometry;
  Compare(f); EXPECT_EQ(f.state.control.element,1u);
  f.status[1]=TypeParam::Status::Success;
  std::fill(f.v.begin(),f.v.end(),.125);
  f.accepted[0].endpoints[0].force_N={3,5,7};
  Compare(f); EXPECT_TRUE(f.state.control.diagnostics.valid);
}
} // namespace connector_operand_test
