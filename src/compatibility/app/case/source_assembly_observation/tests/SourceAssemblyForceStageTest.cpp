#include "ForceStageOracle.h"

namespace crash::cases::source_assembly_observation::test {
TEST(SourceAssemblyForceStage,ActualSourceZeroDT1IgnoresFiniteAccelerationAndCountsPrimaryOnce) {
    ForceFixture f; ForceStageSummary out; const auto input=f.force_input();
    ASSERT_TRUE(ObserveForceStage(input,&out)); Check(out,Oracle(input));
    KineticSummary old;
    ASSERT_TRUE(ObserveInitial(f.bindings,f.stamp,f.old.view(),f.old_groups.data(),6,Kinetic(f.bindings,f.old),&old));
    EXPECT_EQ(out.native_total,old.native_total); EXPECT_EQ(out.effective_total,old.effective_total);
    EXPECT_EQ(out.groups.primary_translation,old.groups.primary_translation); EXPECT_GT(out.groups.primary_translation,0);
    EXPECT_EQ(out.phase.force_time,0); EXPECT_EQ(out.phase.durations.previous_drift_dt,0);
    EXPECT_EQ(out.phase.durations.kick_dt,.5*H); EXPECT_EQ(out.phase.durations.drift_dt,H);
    EXPECT_EQ(out.base_epoch,0u); EXPECT_EQ(out.enclosing_epoch,1u); EXPECT_EQ(out.enclosing_time,H);
    EXPECT_EQ(out.source.source_instance_id,f.bindings.source_instance_id()); EXPECT_EQ(out.source.group_count,6u);
    f.acceleration.v.back()=std::numeric_limits<double>::quiet_NaN(); const auto saved=Bytes(out);
    EXPECT_FALSE(ObserveForceStage(f.force_input(),&out)); EXPECT_EQ(Bytes(out),saved);
}
TEST(SourceAssemblyForceStage,LaterNonzeroCollocationMatchesIndependentWorldTensorAndSourceSpecificAcceleration) {
    ForceFixture f; f.Later(); const auto input=f.force_input(); ForceStageSummary out;
    ASSERT_TRUE(ObserveForceStage(input,&out)); Check(out,Oracle(input));
    EXPECT_EQ(out.phase.force_time,2*H); EXPECT_EQ(out.phase.input_velocity_time,1.5*H);
    EXPECT_EQ(out.phase.previous_frame_time,H); EXPECT_EQ(out.phase.durations.previous_drift_dt,H);
    EXPECT_EQ(out.phase.durations.kick_dt,H); EXPECT_EQ(out.base_epoch,2u); EXPECT_EQ(out.enclosing_epoch,3u);
    EXPECT_EQ(out.attempt,input.prepared.attempt);
    auto wrong=f.group_acceleration; wrong.back().angular_acceleration=wrong.front().angular_acceleration;
    auto altered=input; altered.group_acceleration=wrong.data(); ForceStageSummary other;
    ASSERT_TRUE(ObserveForceStage(altered,&other)); EXPECT_GT(std::abs(out.groups.rotation-other.groups.rotation),1e-10);
    // Wrong numbers with truthful-looking labels are not live-owner-authenticated
    // by a pure value API; they must change the result, never be silently ignored.
    auto old_frame=f.next_groups; old_frame.back().state.principal_axes=f.old_groups.back().state.principal_axes;
    altered=input; altered.force_groups=old_frame.data(); ASSERT_TRUE(ObserveForceStage(altered,&other));
    EXPECT_GT(std::abs(out.groups.rotation-other.groups.rotation),1e-10);
}
TEST(SourceAssemblyForceStage,SmallGroupReplacementSurvivesLargeOrdinaryEnergyWithoutWholeSumSubtraction) {
    ForceFixture f; f.Later(); ForceStageSummary before,after;
    ASSERT_TRUE(ObserveForceStage(f.force_input(),&before)); ASSERT_GT(std::abs(before.replacement),1e-6);
    f.old.v[3*f.LastOrdinary()]=1e18;
    ASSERT_TRUE(ObserveForceStage(f.force_input(),&after)); Check(after,Oracle(f.force_input()));
    EXPECT_EQ(after.replacement,before.replacement); EXPECT_EQ(after.groups.total,before.groups.total);
    EXPECT_EQ(after.effective_total-after.native_total,0); EXPECT_NE(after.replacement,0);
}
} // namespace crash::cases::source_assembly_observation::test
