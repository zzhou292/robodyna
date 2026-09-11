#include "NativeOracle.h"
#include "Trajectory.h"
namespace rear_force_test {
namespace {
void RunTrajectory(bool rotate) {
  for (bool collapsed : {false,true}) {
    const auto reference = Reference(collapsed);
    const auto material = Material();
    law::History accepted;
    ASSERT_EQ(law::InitializeHistory(reference,material,accepted),s::Status::Success);
    auto native = NativeInitial(reference.input());
    double peak_plastic = 0, peak_q = 0;
    bool unloading = false;
    for (unsigned step = 0; step < 96; ++step) {
      SCOPED_TRACE(step);
      auto interval = Path(reference,step,rotate);
      interval.base_time_s = accepted.stamp().time_s;
      const auto expected = Native(material,native,interval);
      ASSERT_EQ(expected.status,0);
      law::ForceTrial actual;
      ASSERT_EQ(law::EvaluateForce(reference,accepted,interval,material,actual),s::Status::Success);
      ASSERT_TRUE(Agree(actual,expected));
      EXPECT_EQ(actual.diagnostics.caller_degeneracy,collapsed ? 12u : 0u);
      EXPECT_EQ(actual.proposed_history.stamp().sample_index,step+1);
      EXPECT_EQ(actual.proposed_history.stamp().time_s,interval.base_time_s+interval.dt_s);
      const double plastic = actual.proposed_history.data().global.plastic_strain;
      peak_plastic = std::max(peak_plastic,plastic);
      unloading |= step > 10 && actual.point[0].material.plastic_increment == 0;
      peak_q = std::max(peak_q,actual.proposed_history.data().global.bulk_pressure_pa);
      for (unsigned ip = 0; ip < 8; ++ip) {
        EXPECT_EQ(expected.next.point[20*ip+18],native.point[20*ip+18]);
        if (collapsed) EXPECT_EQ(actual.point[ip].selective_volume_increment,0);
      }
      accepted = actual.proposed_history;
      native = expected.next; // No resynchronization from production history.
    }
    EXPECT_GT(peak_plastic,0);
    EXPECT_TRUE(unloading);
    EXPECT_GT(peak_q,0);
    EXPECT_GT(accepted.data().global.plastic_work_j,0);
    EXPECT_GT(accepted.data().global.filtered_rate_per_s,0);
  }
}
}
TEST(Rear18ForceNative, IndependentEightPointLoadUnloadHistoryAndNativeWork) { RunTrajectory(false); }
TEST(Rear18ForceNative, CurrentFrameRotationPreservesSelectedNativeCaller) { RunTrajectory(true); }
TEST(Rear18ForceNative, ElasticNonaffineVolumeAndCollapsedCenterBranch) {
  for (bool collapsed : {false,true}) {
    const auto reference = Reference(collapsed);
    const auto material = Material();
    law::History accepted;
    ASSERT_EQ(law::InitializeHistory(reference,material,accepted),s::Status::Success);
    auto interval = Step(reference,accepted,0,0);
    for (unsigned n = 0; n < 8; ++n) {
      const auto x = reference.input().position_m[n];
      interval.velocity_midpoint_m_s[n] = {.3*x.x*x.y,-.2*x.y*x.z,.1*x.x*x.z};
    }
    const auto expected = Native(material,NativeInitial(reference.input()),interval);
    ASSERT_EQ(expected.status,0);
    law::ForceTrial actual;
    ASSERT_EQ(law::EvaluateForce(reference,accepted,interval,material,actual),s::Status::Success);
    ASSERT_TRUE(Agree(actual,expected));
    unsigned corrected = 0;
    for (const auto& p : actual.point) {
      EXPECT_EQ(p.material.plastic_increment,0);
      corrected += p.storage_volume_factor != 1;
    }
    EXPECT_EQ(corrected == 0,collapsed);
  }
}
}
