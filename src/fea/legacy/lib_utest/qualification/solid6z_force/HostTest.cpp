// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <limits>

namespace solid6z_force_test {
TEST(Solid6zForceHost, RecurrentRotatedNonaffinePathAdvancesTrueOnePointHistory) {
  const auto reference = Reference();
  const auto material = Material();
  auto accepted = Initial(reference,material);
  double peak_modes[3][4]{};
  double peak_material_work = 0, peak_hourglass_work = 0;
  for (unsigned step = 0; step < 400; ++step) {
    auto interval = Path(reference,step);
    interval.base_time_s = accepted.stamp().time_s;
    s::ForceTrial trial;
    ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},trial),s::Status::Success) << step;
    EXPECT_EQ(trial.proposed_history.stamp().sample_index,step+1);
    EXPECT_GT(trial.material.unscaled_element_dt_s,0);
    EXPECT_GT(trial.material.raw_stiffness_n_m,0);
    ASSERT_GE(trial.stabilization.effective_shear_modulus_pa,48e6);
    double norm = 0;
    s::Vec3 net;
    for (const auto& force : trial.rhs_force_n) {
      net.x += force.x;
      net.y += force.y;
      net.z += force.z;
      norm += std::abs(force.x)+std::abs(force.y)+std::abs(force.z);
    }
    EXPECT_LE(std::abs(net.x)+std::abs(net.y)+std::abs(net.z),1e-11*std::max(1.0,norm));
    for (unsigned k = 0; k < 3; ++k) {
      for (unsigned mode = 0; mode < 4; ++mode) peak_modes[k][mode] = std::max(peak_modes[k][mode],
          std::abs(trial.proposed_history.data().hourglass_stress_pa[k][mode]));
    }
    peak_material_work = std::max(peak_material_work,std::abs(trial.material.internal_work_j));
    peak_hourglass_work = std::max(peak_hourglass_work,std::abs(
        trial.stabilization.first_work_j+trial.stabilization.second_work_j));
    accepted = trial.proposed_history;
  }
  for (unsigned k = 0; k < 3; ++k) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      SCOPED_TRACE(k);
      SCOPED_TRACE(mode);
      RecordProperty("peak_mode_"+std::to_string(k)+"_"+std::to_string(mode),std::to_string(peak_modes[k][mode]));
      // In the native repeated-slot wedge, mode3 reduces to the affine
      // triangle-edge pattern [1,-1,0,1,-1,0]; projection removes it.
      // Carry all twelve native histories, including these three null rates.
      if (mode == 2) EXPECT_LT(peak_modes[k][mode],1e-6);
      else EXPECT_GT(peak_modes[k][mode],1);
    }
  }
  EXPECT_GT(peak_material_work,1e-5);
  EXPECT_GT(peak_hourglass_work,1e-7);
}
TEST(Solid6zForceHost, RigidTurnUsesTotalGradientWithoutInventingRateHistory) {
  const auto reference = Reference();
  const auto material = Material();
  const auto accepted = Initial(reference,material);
  s::PrescribedInterval interval;
  interval.dt_s = 1e-6;
  const double c = std::cos(1.2), sn = std::sin(1.2);
  for (unsigned n = 0; n < 6; ++n) {
    const auto x = reference.input().position_m[n];
    interval.position_endpoint_m[n] = {c*x.x-sn*x.y,sn*x.x+c*x.y,x.z};
  }
  s::ForceTrial trial;
  ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},trial),s::Status::Success);
  for (double rate : trial.geometry.engineering_rate_per_s) EXPECT_EQ(rate,0);
  for (const auto& component : trial.proposed_history.data().hourglass_stress_pa)
    for (double value : component) EXPECT_EQ(value,0);
  for (const auto& f : trial.rhs_force_n) {
    EXPECT_LT(std::abs(f.x)+std::abs(f.y)+std::abs(f.z),1e-6);
  }
}
TEST(Solid6zForceHost, LateHistoryAndPhaseFailurePreserveWholeOutputThenRetry) {
  const auto reference = Reference();
  const auto material = Material();
  const auto accepted = Initial(reference,material);
  const auto interval = Path(reference,0);
  s::ForceTrial output;
  ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},output),s::Status::Success);
  unsigned char saved[sizeof(output)];
  std::memcpy(saved,&output,sizeof(output));
  auto values = accepted.data();
  values.material.internal_energy_density_j_m3 = std::numeric_limits<double>::max();
  values.hourglass_stress_pa[2][3] = std::copysign(std::numeric_limits<double>::max(),
      output.stabilization.modal_velocity_m_s[2][3]);
  s::History bad;
  ASSERT_EQ(s::PreparePrescribedHistory(reference,material,{},values,{},bad),s::Status::Success);
  EXPECT_NE(s::EvaluateForce(reference,bad,interval,material,{},output),s::Status::Success);
  EXPECT_EQ(std::memcmp(saved,&output,sizeof(output)),0);
  auto phase = interval;
  ++phase.sample_index;
  EXPECT_NE(s::EvaluateForce(reference,accepted,phase,material,{},output),s::Status::Success);
  EXPECT_EQ(std::memcmp(saved,&output,sizeof(output)),0);
  auto unsupported = s::ForceProfile{};
  unsupported.engine_frame = 2;
  EXPECT_EQ(s::InitializeHistory(reference,material,unsupported,bad),s::Status::UnsupportedProfile);
  EXPECT_EQ(s::EvaluateForce(reference,accepted,interval,material,{},output),s::Status::Success);
}
} // namespace solid6z_force_test
