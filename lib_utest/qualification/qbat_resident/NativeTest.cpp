// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "ResultValues.h"
#include "../qbat_force/NativeOracle.h"

namespace qbat_resident_test {

TEST(QbatResidentNative, PointerFreeCacheRetainsIndependentFourPointNativeUnloadAndRollback) {
  qbat_force_test::Fixture fixture(true,.1);
  auto element=Element(fixture);
  qb::BatchResult accepted;
  ASSERT_EQ(batch::InitializeResult(element,accepted),qb::Status::kSuccess);
  qbat_force_test::NativeState native(fixture.Virgin().data());
  bool yielded=false;
  for(unsigned step=0;step<160;++step) {
    const auto interval=qbat_force_test::Path(fixture,step);
    qb::BatchResult proposed;
    ASSERT_EQ(batch::Advance(element,accepted,interval,proposed),qb::Status::kSuccess);
    auto native_trial=native;
    native_trial.Step(fixture,interval);
    ASSERT_FALSE(HasFailure());
    qbat_force_test::CompareNative(Restore(element,proposed),native_trial);
    ASSERT_FALSE(HasFailure())<<step;
    if(step==32) {
      auto invalid=interval;
      invalid.position_endpoint[3].z=std::numeric_limits<double>::quiet_NaN();
      const auto before=Bytes(proposed);
      EXPECT_NE(batch::Advance(element,accepted,invalid,proposed),qb::Status::kSuccess);
      EXPECT_EQ(Bytes(proposed),before);
      qb::BatchResult retry;
      ASSERT_EQ(batch::Advance(element,accepted,interval,retry),qb::Status::kSuccess);
      EXPECT_EQ(ResultValues(retry),ResultValues(proposed));
    }
    for(const auto& point:proposed.history.point) yielded|=point.material.plastic_strain>1e-5;
    accepted=proposed;
    native=native_trial;
  }
  EXPECT_TRUE(yielded);
}
TEST(QbatResidentNative, FourthPointRemovalKeepsSavedCurrentWorkAndFinalZeroProjection) {
  qbat_force_test::Fixture fixture;
  const auto element=Element(fixture);
  qb::BatchResult accepted;
  ASSERT_EQ(batch::InitializeResult(element,accepted),qb::Status::kSuccess);
  const auto initial=qbat_force_test::NearRemoval(fixture,3);
  accepted.history=initial.data();
  accepted.stamp=initial.stamp();
  qbat_force_test::NativeState native(initial.data());
  for(unsigned step=0;step<3;++step) {
    const auto interval=qbat_force_test::Path(fixture,step);
    qb::BatchResult proposed;
    ASSERT_EQ(batch::Advance(element,accepted,interval,proposed),qb::Status::kSuccess);
    native.Step(fixture,interval);
    ASSERT_FALSE(HasFailure());
    qbat_force_test::CompareNative(Restore(element,proposed),native);
    ASSERT_FALSE(HasFailure());
    ASSERT_FALSE(proposed.history.element_active);
    EXPECT_EQ(proposed.diagnostics.removed_now,step==0);
    EXPECT_GT(proposed.diagnostics.unscaled_element_dt_s,0);
    for(const auto& force:proposed.internal_force_n) EXPECT_EQ(force.x*force.x+force.y*force.y+force.z*force.z,0);
    accepted=proposed;
  }
}
} // namespace qbat_resident_test
