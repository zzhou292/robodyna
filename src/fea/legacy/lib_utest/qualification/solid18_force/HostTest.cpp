// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"

namespace solid18_force_test {
TEST(Solid18Force, EightPointStartupUsesSelectedNativeVolumesAndNoInventedStress) {
  const auto reference = Reference();
  const auto material = Material();
  s::History history;
  ASSERT_EQ(s::InitializeHistory(reference,material,history),s::Status::Success);
  double smallest = std::numeric_limits<double>::max();
  double largest = 0;
  for (unsigned ip = 0; ip < 8; ++ip) {
    const auto& point = history.data().point[ip];
    EXPECT_DOUBLE_EQ(point.storage_volume_m3,reference.geometry().point[ip].initial_volume_m3);
    EXPECT_DOUBLE_EQ(point.initial_volume_m3,point.storage_volume_m3);
    EXPECT_DOUBLE_EQ(point.density_kg_m3,material.density_kg_m3);
    EXPECT_EQ(point.material.plastic_work_j,0);
    for (double stress : point.material.point.stress_pa) EXPECT_EQ(stress,0);
    smallest = std::min(smallest,point.storage_volume_m3);
    largest = std::max(largest,point.storage_volume_m3);
  }
  EXPECT_GT(largest/smallest,1.1);
  EXPECT_DOUBLE_EQ(history.data().global.density_kg_m3,
                   reference.mass().initial_global_density_kg_m3);
}

TEST(Solid18Force, CyclicPathYieldsUnloadsAndRetainsDistinctPointAndVolumeHistory) {
  const auto reference = Reference();
  const auto material = Material();
  s::History history;
  ASSERT_EQ(s::InitializeHistory(reference,material,history),s::Status::Success);
  double peak_work = 0;
  double volume_excursion = 0;
  bool unloading = false;
  double prior_plastic = 0;
  s::ForceTrial result;
  for (unsigned step = 0; step < 400; ++step) {
    ASSERT_EQ(s::EvaluateForce(reference,history,Path(reference,step),material,result),s::Status::Success)
        << "step " << step;
    const auto& next = result.proposed_history.data();
    EXPECT_GE(next.global.plastic_work_j,peak_work);
    peak_work = next.global.plastic_work_j;
    const double plastic = next.point[0].material.point.plastic_strain;
    if (step > 50 && plastic == prior_plastic && result.point[0].engineering_rate_per_s[0] < 0)
      unloading = true;
    prior_plastic = plastic;
    for (unsigned ip = 0; ip < 8; ++ip) {
      volume_excursion = std::max(volume_excursion,
          std::abs(next.point[ip].storage_volume_m3/reference.geometry().point[ip].initial_volume_m3-1));
      EXPECT_DOUBLE_EQ(next.point[ip].initial_volume_m3,reference.geometry().point[ip].initial_volume_m3);
    }
    EXPECT_GT(result.diagnostics.minimum_unscaled_dt_s,0);
    history = result.proposed_history;
  }
  EXPECT_GT(peak_work,.01);
  EXPECT_GT(volume_excursion,1e-6);
  EXPECT_TRUE(unloading);
  EXPECT_NE(history.data().point[0].material.point.plastic_strain,
            history.data().point[7].material.point.plastic_strain);
}

TEST(Solid18Force, PhaseSourceAndLatePointFailurePreserveOutputThenRetry) {
  const auto reference = Reference();
  const auto material = Material();
  s::History initial;
  ASSERT_EQ(s::InitializeHistory(reference,material,initial),s::Status::Success);
  auto interval = Path(reference,0);
  s::ForceTrial clean;
  ASSERT_EQ(s::EvaluateForce(reference,initial,interval,material,clean),s::Status::Success);
  s::ForceTrial output = clean;
  const auto before = solid18_test::Bytes(output);
  auto wrong = interval;
  wrong.sample_index = 2;
  EXPECT_NE(s::EvaluateForce(reference,initial,wrong,material,output),s::Status::Success);
  EXPECT_EQ(solid18_test::Bytes(output),before);
  auto input = reference.input();
  ++input.source_element_id;
  s::Reference other;
  ASSERT_EQ(s::InitializeReference(input,other),s::Status::Success);
  EXPECT_NE(s::EvaluateForce(other,initial,interval,material,output),s::Status::Success);
  EXPECT_EQ(solid18_test::Bytes(output),before);
  auto values = initial.data();
  values.point[7].material.point.stress_pa[0] = 1e308;
  s::History late;
  ASSERT_EQ(s::PreparePrescribedHistory(reference,material,values,{},late),s::Status::Success);
  EXPECT_NE(s::EvaluateForce(reference,late,interval,material,output),s::Status::Success);
  EXPECT_EQ(solid18_test::Bytes(output),before);
  ASSERT_EQ(s::EvaluateForce(reference,initial,interval,material,output),s::Status::Success);
  EXPECT_TRUE(SameHistory(clean.proposed_history,output.proposed_history));
  for (unsigned n = 0; n < 8; ++n) {
    EXPECT_TRUE(s::detail::SameVector(clean.rhs_force_n[n],output.rhs_force_n[n]));
  }
}

TEST(Solid18Force, NativeSelectionIgnoresZeroAndTiesFollowVisitation) {
  const auto reference = Reference();
  const auto material = Material();
  s::History initial;
  ASSERT_EQ(s::InitializeHistory(reference,material,initial),s::Status::Success);
  auto values = initial.data();
  values.global.plastic_strain = .02;
  for (auto& point : values.point) {
    point.material.point.plastic_strain = .01;
    point.material.point.stress_pa[0] = 1e7;
  }
  values.point[7].material.point.plastic_strain = 0;
  s::History prescribed;
  ASSERT_EQ(s::PreparePrescribedHistory(reference,material,values,{},prescribed),s::Status::Success);
  s::ForceTrial result;
  ASSERT_EQ(s::EvaluateForce(reference,prescribed,Path(reference,0),material,result),s::Status::Success);
  EXPECT_EQ(result.diagnostics.selected_point,3u); // Sequence 0,4,2,6,1,5,3,7.
  EXPECT_GT(result.diagnostics.selection_factor,0);
  EXPECT_GT(result.diagnostics.selective_poisson_ratio,material.poisson_ratio);
}
}  // namespace solid18_force_test
