// SPDX-License-Identifier: AGPL-3.0-or-later
#include "WorkingSupport.h"
namespace h24_test {
TEST(H24NativeWorkingUnits, FullNativeMillimetreGeometryMaterialAndControlledPrefix) {
  auto x=MillimetreCase();Move(x);const auto reference=WorkingReference(x);c::WorkingResult r;
  ASSERT_EQ(c::EvaluateWorking(reference,x.accepted,x.interval,false,r),s::ForceStatus::Success);
  CompareWorking(x,r,MillimetreNative(MillimetreNativeHistory(x),x));
  auto warped=solid24_test::Brick();warped.position_m[6].x+=.002;warped.position_m[3].z+=.001;
  x=MillimetreCase(warped);Move(x);ASSERT_EQ(c::EvaluateWorking(WorkingReference(x),x.accepted,x.interval,false,r),s::ForceStatus::Success);
  CompareWorking(x,r,MillimetreNative(MillimetreNativeHistory(x),x));
}
TEST(H24NativeWorkingUnits, IndependentlyCarried32MillimetreIntervals) {
  auto x=MillimetreCase();x.accepted.material.internal_energy_density_j_m3=123.25;
  auto history=MillimetreNativeHistory(x);const auto reference=WorkingReference(x);
  for(unsigned step=0;step<32;++step){SCOPED_TRACE(step);Move(x,(step%8<4?1.:-1.)*(.1+.01*step));
    c::WorkingResult r;ASSERT_EQ(c::EvaluateWorking(reference,x.accepted,x.interval,false,r),s::ForceStatus::Success);
    const auto n=MillimetreNative(history,x);CompareWorking(x,r,n);ASSERT_FALSE(HasFailure());
    x.accepted=r.stage.proposed_values;x.interval.base_time_s+=x.interval.dt_s;++x.interval.sample_index;history.values=n.carried;
  }
}
TEST(H24NativeWorkingUnits, NativeMillimetreVolumeFloorIsAppliedBeforeConversion) {
  for(double factor:{.5,2.}) {
    auto x=MillimetreCase(solid24_test::FloorControl(s::WorkingLengthUnit::Millimetre,factor));
    x.accepted.material.internal_energy_density_j_m3=123.25;const auto reference=WorkingReference(x);c::WorkingResult r;
    ASSERT_EQ(c::EvaluateWorking(reference,x.accepted,x.interval,false,r),s::ForceStatus::Success);
    CompareWorking(x,r,MillimetreNative(MillimetreNativeHistory(x),x));
    // SI evaluation with its original literal floor would reduce this by ~1e-9.
    EXPECT_GT(r.material.history.internal_energy_density_j_m3,1.);
    if(factor<1)EXPECT_LT(r.material.history.internal_energy_density_j_m3,100.);
  }
}
TEST(H24NativeWorkingUnits, UnitMismatchAndLateFailureAreAtomic) {
  auto x=MillimetreCase();Move(x);const auto ref=WorkingReference(x);c::WorkingResult r;
  ASSERT_EQ(c::EvaluateWorking(ref,x.accepted,x.interval,false,r),s::ForceStatus::Success);const auto before=Values(r);
  c::WorkingReference invalid;EXPECT_EQ(c::PrepareWorkingReference(x.reference,x.material,{1,1,1},invalid),s::ForceStatus::UnsupportedProfile);
  EXPECT_FALSE(invalid.prepared());auto old=x.accepted;old.controlled_hourglass.force_n[1][2]=std::numeric_limits<double>::max();
  EXPECT_NE(c::EvaluateWorking(ref,old,x.interval,false,r),s::ForceStatus::Success);EXPECT_EQ(Values(r),before);
  EXPECT_EQ(c::EvaluateWorking(ref,x.accepted,x.interval,false,r),s::ForceStatus::Success);EXPECT_EQ(Values(r),before);
}
} // namespace h24_test
