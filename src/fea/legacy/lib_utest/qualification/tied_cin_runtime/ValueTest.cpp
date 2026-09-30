#include "Fixture.h"
#include <type_traits>

namespace cin_runtime_test {
TEST(CinRuntimeValues, ThreeStagesCarryCurrentCoefficientsAndSavedHistoryWithoutReset) {
  Fixture f;
  const auto original_mass = f.mass;
  const auto original_j = f.inertia;
  const auto original_load = f.load;
  for (unsigned step = 0; step < 3; ++step) {
    const auto entry = f.inertia;
    ASSERT_TRUE(cin::PrepareForceTrial(f.View(), f.Force()));
    EXPECT_EQ(f.entry_j, entry);
    for (std::size_t r = 0; r < f.rows.size(); ++r) {
      const auto node = f.rows[r].secondary;
      EXPECT_EQ(f.mass[node], 0);
      EXPECT_EQ(f.inertia[node], 0);
      EXPECT_EQ(f.saved_mass[r], original_mass[node]);
      EXPECT_EQ(f.saved_inertia[r], original_j[node]);
      EXPECT_EQ(f.stif[node], 1e-20);
      EXPECT_EQ(f.stifr[node], 1e-20);
      for (unsigned axis = 0; axis < 6; ++axis) EXPECT_EQ(f.load[axis*f.mass.size()+node], original_load[axis*f.mass.size()+node]);
    }
    ASSERT_TRUE(cin::RecoverMotionTrial(f.View(), f.Motion()));
    for (std::size_t i = 0; i < f.x.size(); ++i) f.x[i] += 1e-5*f.velocity[i];
  }
}
TEST(CinRuntimeValues, ExplicitAllWitnessFlagsAndCoincidentActiveWitnessAreRequired) {
  Fixture f;
  f.rows.back().witnesses = {1, 2};
  f.flags = {1, 2, 1};
  ASSERT_TRUE(cin::PrepareForceTrial(f.View(), f.Force()));
  const auto prior_mass = f.mass;
  f.flags.back() = 2;
  EXPECT_EQ(cin::PrepareForceTrial(f.View(), f.Force()).status, cin::StageStatus::PendingReleaseEligibility);
  EXPECT_EQ(f.mass, prior_mass);
  f.flags.back() = 0;
  EXPECT_EQ(cin::PrepareForceTrial(f.View(), f.Force()).status, cin::StageStatus::PendingReleaseEligibility);
  EXPECT_EQ(f.mass, prior_mass);
  const std::uint32_t first[] = {0, 0, 2};
  auto view = f.View();
  view.first_witness = first;
  f.flags = {1, 2, 1};
  EXPECT_EQ(cin::PrepareForceTrial(view, f.Force()).status, cin::StageStatus::SourceMismatch);
  EXPECT_EQ(f.mass, prior_mass);
}
TEST(CinRuntimeValues, Complete11165ScopeChecksLastEligibilityBeforeAnyTransfer) {
  constexpr std::size_t count = 11165, n = count+4;
  Fixture seed;
  const auto shape = tied_patch_test::Geometry(0);
  std::vector<cin::StageRow> rows(count);
  std::vector<std::uint8_t> dependent(n, 0), activity(count, 1);
  std::vector<double> position(3*n), loads(6*n), mass(n, 1), inertia(n, .01);
  std::vector<double> stif(n, 2), stifr(n, .1), saved_mass(count), saved_inertia(count), entry(n);
  std::vector<tied::Patch> patches(count);
  for (unsigned slot = 0; slot < 4; ++slot) cin::detail::WriteXyz(position.data(), slot, shape.master_position[slot]);
  for (std::size_t r = 0; r < count; ++r) {
    rows[r] = {std::uint32_t(r+4), {0, 1, 2, 3}, {std::uint32_t(r), 1}};
    dependent[r+4] = 1;
    auto x = shape.secondary_position;
    x.z += r*1e-8;
    cin::detail::WriteXyz(position.data(), r+4, x);
    loads[r+4] = .001*(r+1);
  }
  double dmas = 0;
  const cin::StageView view{rows.data(), dependent.data(), n, count, count};
  const cin::ForceTrial force{position.data(), loads.data(), mass.data(), inertia.data(), stif.data(), stifr.data(),
    saved_mass.data(), saved_inertia.data(), &dmas, entry.data(), patches.data(), activity.data()};
  activity.back() = 0;
  const auto failed = cin::PrepareForceTrial(view, force);
  EXPECT_EQ(failed.status, cin::StageStatus::PendingReleaseEligibility);
  EXPECT_EQ(failed.row, count-1);
  EXPECT_EQ(mass.front(), 1);
  EXPECT_EQ(mass.back(), 1);
  activity.back() = 1;
  ASSERT_TRUE(cin::PrepareForceTrial(view, force));
  EXPECT_EQ(mass.front(), 1+.25*count);
  EXPECT_EQ(mass.back(), 0);
  EXPECT_EQ(saved_mass.back(), 1);
  EXPECT_TRUE(patches.back().prepared());
  EXPECT_GT(loads.front(), 0);
}
TEST(CinRuntimeValues, LatePrivateFailureLeavesAcceptedCopyAndRetryRecomputesPatches) {
  Fixture accepted;
  Fixture candidate = accepted;
  const auto prior_mass = accepted.mass;
  const auto prior_j = accepted.inertia;
  const auto last = candidate.rows.back();
  for (const auto node : last.masters) {
    for (unsigned a = 0; a < 3; ++a) candidate.x[3*node+a] = 0;
  }
  const auto report = cin::PrepareForceTrial(candidate.View(), candidate.Force());
  EXPECT_EQ(report.status, cin::StageStatus::InvalidPatch);
  EXPECT_EQ(report.row, 1u);
  EXPECT_EQ(accepted.mass, prior_mass);
  EXPECT_EQ(accepted.inertia, prior_j);
  candidate.x = accepted.x;
  candidate.mass = accepted.mass;
  candidate.inertia = accepted.inertia;
  candidate.load = accepted.load;
  candidate.stif = accepted.stif;
  candidate.stifr = accepted.stifr;
  candidate.saved_mass = accepted.saved_mass;
  candidate.saved_inertia = accepted.saved_inertia;
  candidate.dmas = accepted.dmas;
  ASSERT_TRUE(cin::PrepareForceTrial(candidate.View(), candidate.Force()));
  candidate.a[3*last.masters[2]] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(cin::RecoverMotionTrial(candidate.View(), candidate.Motion()).status, cin::StageStatus::NonfiniteResult);
}
TEST(CinRuntimeValues, TransferUsesForceEntryInertiaAndExactRepeatedSlots) {
  Fixture f;
  const auto row = f.rows.back();
  const auto repeated = row.masters[2];
  const auto original_mass = f.mass;
  const auto original_j = f.inertia;
  ASSERT_EQ(row.masters[3], repeated);
  ASSERT_TRUE(cin::PrepareForceTrial(f.View(), f.Force()));
  EXPECT_EQ(f.mass[repeated], (original_mass[repeated]+.25*original_mass[row.secondary])+.25*original_mass[row.secondary]);
  EXPECT_GT(f.inertia[repeated], original_j[repeated]);
  Fixture zero_j;
  zero_j.inertia[zero_j.rows.back().masters[0]] = 0;
  ASSERT_TRUE(cin::PrepareForceTrial(zero_j.View(), zero_j.Force()));
  EXPECT_GT(zero_j.mass[repeated], f.mass[repeated]);
  EXPECT_EQ(zero_j.inertia[repeated], original_j[repeated]);
}
TEST(CinRuntimeValues, OptionalLayoutChargesBothStateTailsAndPreservesLegacyLayout) {
  static_assert(!std::is_copy_constructible_v<fea::nodal_detail::CinStorage>);
  static_assert(!std::is_copy_assignable_v<fea::nodal_detail::CinStorage>);
  fea::nodal_detail::CinLayout layout;
  fea::NodalCinLimits limits;
  ASSERT_TRUE(layout.Initialize(359785, 11165, 11165, limits, sizeof(fea::nodal_detail::CinStorage)));
  EXPECT_EQ(layout.optional_device_bytes, layout.device_bytes+2*layout.state_values*sizeof(double));
  const auto exact = layout.optional_device_bytes;
  limits.max_device_bytes = exact-1;
  EXPECT_FALSE(layout.Initialize(359785, 11165, 11165, limits, sizeof(fea::nodal_detail::CinStorage)));
  ++limits.max_device_bytes;
  ASSERT_TRUE(layout.Initialize(359785, 11165, 11165, limits, sizeof(fea::nodal_detail::CinStorage)));
  RecordProperty("original_count_optional_device_bytes", std::to_string(exact));
  RecordProperty("original_count_extra_state_values_per_slab", std::to_string(layout.state_values));
}
} // namespace cin_runtime_test
