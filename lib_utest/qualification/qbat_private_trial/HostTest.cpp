// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "../qbat_force/NativeOracle.h"
namespace qbat_private_test {
TEST(QbatPrivateTrial, DefinedFieldsMatchPublicAndOriginalNativeRecurrences) {
  for(bool warped:{false,true}) {
    SCOPED_TRACE(warped);
    Fixture fixture(warped,.1);const auto element=Element(fixture);
    qb::BatchResult accepted;
    ASSERT_EQ(batch::InitializeResult(element,accepted),qb::Status::kSuccess);
    qbat_force_test::NativeState native(fixture.Virgin().data());
    for(unsigned step=0;step<160;++step) {
      SCOPED_TRACE(step);
      const auto interval=Path(fixture,step);qb::BatchResult expected,actual;
      Dirty(actual);
      ASSERT_EQ(batch::Advance(element,accepted,interval,expected),qb::Status::kSuccess);
      ASSERT_EQ(batch::AdvanceIntoTrial(element,accepted,interval,actual),qb::Status::kSuccess);
      EXPECT_EQ(ResultValues(actual),ResultValues(expected));
      native.Step(fixture,interval);
      ASSERT_FALSE(HasFailure());
      qbat_force_test::CompareNative(qbat_resident_test::Restore(element,actual),native);
      ASSERT_FALSE(HasFailure());
      accepted=actual;
    }
  }
}
TEST(QbatPrivateTrial, AllRemovalPointOrdersAndInactiveUpdatesRetainNativeFields) {
  Fixture fixture;const auto element=Element(fixture);
  for(unsigned last=0;last<4;++last) {
    SCOPED_TRACE(last);
    const auto start=qbat_force_test::NearRemoval(fixture,last);qb::BatchResult accepted;
    accepted.history=start.data();accepted.stamp=start.stamp();
    qbat_force_test::NativeState native(start.data());
    for(unsigned step=0;step<3;++step) {
      SCOPED_TRACE(step);
      const auto interval=Path(fixture,step);qb::BatchResult expected,actual;Dirty(actual);
      ASSERT_EQ(batch::Advance(element,accepted,interval,expected),qb::Status::kSuccess);
      ASSERT_EQ(batch::AdvanceIntoTrial(element,accepted,interval,actual),qb::Status::kSuccess);
      EXPECT_EQ(ResultValues(actual),ResultValues(expected));
      native.Step(fixture,interval);
      ASSERT_FALSE(HasFailure());
      qbat_force_test::CompareNative(qbat_resident_test::Restore(element,actual),native);
      ASSERT_FALSE(HasFailure());
      EXPECT_FALSE(actual.history.element_active);EXPECT_EQ(actual.diagnostics.removed_now,step==0);
      accepted=actual;
    }
  }
}
TEST(QbatPrivateTrial, InitialStatusOrderAcceptedAliasAndDirtyLateFailureRetry) {
  Fixture fixture;auto element=Element(fixture);qb::BatchResult accepted;
  ASSERT_EQ(batch::InitializeResult(element,accepted),qb::Status::kSuccess);
  const auto interval=Path(fixture,0);qb::BatchResult expected,actual;
  ASSERT_EQ(batch::Advance(element,accepted,interval,expected),qb::Status::kSuccess);
  for(unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault);
    auto e=element;auto old=accepted;auto in=interval;
    if(fault==0){e.failure.failure_strain=0;in.dt=0;}
    if(fault==1){old.history.thickness_m=-1;in.dt=0;}
    if(fault==2){old.stamp.time=-1;in.sample_index=17;}
    if(fault==3)old.history.point[3].material.stress[0]=1e308;
    const auto before=ResultValues(old);qb::BatchResult value;Dirty(actual);
    const auto status=batch::Advance(e,old,in,value);
    ASSERT_NE(status,qb::Status::kSuccess);
    EXPECT_EQ(batch::AdvanceIntoTrial(e,old,in,actual),status);
    EXPECT_EQ(ResultValues(old),before);
    ASSERT_EQ(batch::AdvanceIntoTrial(element,accepted,interval,actual),qb::Status::kSuccess);
    EXPECT_EQ(ResultValues(actual),ResultValues(expected));
  }
  const auto before=ResultValues(accepted);
  EXPECT_EQ(batch::AdvanceIntoTrial(element,accepted,interval,accepted),qb::Status::kInvalidInput);
  EXPECT_EQ(ResultValues(accepted),before);
}
}
