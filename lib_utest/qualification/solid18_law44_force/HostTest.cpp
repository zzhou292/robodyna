#include "TestSupport.h"

namespace rear_force_test {
TEST(Rear18Force, EightPointHistoryRetainsReferenceAndActualMaterialIdentity) {
  for (bool collapsed : {false,true}) {
    const auto reference = Reference(collapsed);
    const auto material = Material();
    law::History history;
    ASSERT_EQ(law::InitializeHistory(reference,material,history),s::Status::Success);
    for (unsigned ip = 0; ip < 8; ++ip) {
      EXPECT_EQ(history.data().point[ip].storage_volume_m3,reference.geometry().point[ip].initial_volume_m3);
      EXPECT_EQ(history.data().point[ip].density_kg_m3,material.material.density_kg_m3);
      EXPECT_EQ(history.data().point[ip].material.curve_cursor,0u);
    }
    EXPECT_EQ(law::detail::NativeDegeneracy(reference),collapsed ? 2u : 0u);
    law::ForceTrial trial;
    const auto interval = Step(reference,history);
    ASSERT_EQ(law::EvaluateForce(reference,history,interval,material,trial),s::Status::Success);
    EXPECT_EQ(trial.diagnostics.caller_degeneracy,collapsed ? 12u : 0u);
    EXPECT_GT(trial.diagnostics.minimum_unscaled_dt_s,0);
    EXPECT_GT(trial.diagnostics.raw_stiffness_n_m,0);
    for (const auto& p : trial.point) EXPECT_EQ(p.material.sound_speed_m_s,material.sound_speed_m_s);
  }
}
TEST(Rear18Force, NativeCollapsedBranchSkipsVolumeCorrectionAndCenterForce) {
  for (bool collapsed : {false,true}) {
    const auto reference = Reference(collapsed);
    const auto material = Material();
    law::History history;
    ASSERT_EQ(law::InitializeHistory(reference,material,history),s::Status::Success);
    auto interval = Step(reference,history,0,0);
    for (unsigned n = 0; n < 8; ++n) {
      const auto x = reference.input().position_m[n];
      interval.velocity_midpoint_m_s[n] = {.3*x.x*x.y,-.2*x.y*x.z,.1*x.x*x.z};
    }
    law::ForceTrial trial;
    ASSERT_EQ(law::EvaluateForce(reference,history,interval,material,trial),s::Status::Success);
    unsigned corrected = 0;
    for (const auto& p : trial.point) {
      corrected += p.storage_volume_factor != 1;
      if (collapsed) EXPECT_EQ(p.selective_volume_increment,0);
    }
    if (collapsed) EXPECT_EQ(corrected,0u);
    else EXPECT_GT(corrected,0u);
    s::Vec3 force[8]{};
    law::detail::AccumulateCenterPressure(trial.geometry,trial.diagnostics.caller_degeneracy,7,force);
    double norm = 0;
    for (const auto f : force) norm += std::abs(f.x)+std::abs(f.y)+std::abs(f.z);
    if (collapsed) EXPECT_EQ(norm,0);
    else EXPECT_GT(norm,0);
  }
}
TEST(Rear18Force, LoadingAndUnloadingCarryEightMaterialAndWorkHistories) {
  for (bool collapsed : {false,true}) {
    const auto reference = Reference(collapsed);
    const auto material = Material();
    law::History history;
    ASSERT_EQ(law::InitializeHistory(reference,material,history),s::Status::Success);
    double previous_gamma = 0, peak_plastic = 0;
    for (unsigned step = 0; step < 48; ++step) {
      const double gamma = .025*std::sin((step+1)*.13);
      auto interval = Step(reference,history,gamma,(gamma-previous_gamma)/1e-5);
      previous_gamma = gamma;
      law::ForceTrial trial;
      ASSERT_EQ(law::EvaluateForce(reference,history,interval,material,trial),s::Status::Success);
      EXPECT_GE(trial.proposed_history.data().global.plastic_work_j,history.data().global.plastic_work_j);
      EXPECT_EQ(trial.proposed_history.stamp().sample_index,step+1);
      peak_plastic = std::max(peak_plastic,trial.proposed_history.data().global.plastic_strain);
      history = trial.proposed_history;
    }
    EXPECT_GT(peak_plastic,0);
    EXPECT_GT(history.data().global.filtered_rate_per_s,0);
    EXPECT_GT(history.data().global.plastic_work_j,0);
  }
}
TEST(Rear18Force, LateSourceVelocityAndPointRejectionAreAtomicAndRetryable) {
  const auto reference = Reference(true);
  const auto material = Material();
  law::History history;
  ASSERT_EQ(law::InitializeHistory(reference,material,history),s::Status::Success);
  const auto interval = Step(reference,history);
  law::ForceTrial output;
  ASSERT_EQ(law::EvaluateForce(reference,history,interval,material,output),s::Status::Success);
  const auto bytes = Bytes(output);
  auto bad = interval;
  bad.velocity_midpoint_m_s[7].z = -0.0;
  ASSERT_EQ(law::EvaluateForce(reference,history,bad,material,output),s::Status::InvalidInput);
  EXPECT_EQ(Bytes(output),bytes);
  auto values = history.data();
  values.point[7].material.curve_cursor = material.curve.count;
  law::History unchanged = history;
  EXPECT_EQ(law::PreparePrescribedHistory(reference,material,values,history.stamp(),unchanged),s::Status::InvalidInput);
  EXPECT_EQ(Bytes(unchanged),Bytes(history));
  values = history.data();
  values.point[7].material.stress_pa[5] = 1e200;
  ASSERT_EQ(law::PreparePrescribedHistory(reference,material,values,history.stamp(),unchanged),s::Status::Success);
  EXPECT_EQ(law::EvaluateForce(reference,unchanged,interval,material,output),s::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(output),bytes);
  bad = interval;
  for (auto& position : bad.position_endpoint_m) position.z = 0;
  EXPECT_NE(law::EvaluateForce(reference,history,bad,material,output),s::Status::Success);
  EXPECT_EQ(Bytes(output),bytes);
  ASSERT_EQ(law::EvaluateForce(reference,history,interval,material,output),s::Status::Success);
  EXPECT_EQ(output.proposed_history.stamp().sample_index,1u);
  const auto second = Step(reference,output.proposed_history,.002,1);
  // Accepted history may live in the same public output object.
  ASSERT_EQ(law::EvaluateForce(reference,output.proposed_history,second,material,output),s::Status::Success);
  EXPECT_EQ(output.proposed_history.stamp().sample_index,2u);
}
}
