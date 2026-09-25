// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "HistoryAssertions.h"
namespace type25_friction_test {
TEST(Type25HistoryPhase, EveryLocalRowBranchMatchesPinnedNativeCaller) {
  for (unsigned choice = 0; choice < 7; ++choice) {
    auto row = Row(); n::HistoryPhaseInput input{1, 1, 1};
    switch (choice) {
      case 1: input.secondary_stiffness = 0; break;
      case 2: input.main_stiffness = 0; break;
      case 3: input.local_processor = 2; break;
      case 4: row.irtlm[0] = 0; break;
      case 5: row.irtlm[0] = -7; break;
      case 6: row.irtlm[2] = 32; break;
    }
    n::HistoryPhaseResult actual;
    ASSERT_EQ(n::BeginNativeHistory(row, input, &actual), n::NormalStatus::Ok);
    const auto expected = BeginOracle(row, input);
    EXPECT_EQ(actual.retained_candidate, expected.retained_candidate); SameRow(actual.row, expected.row);
  }
}
TEST(Type25HistoryPhase, ShootingAndInactiveRowsDoNotInventFullResets) {
  const auto row = Row(); n::HistoryPhaseResult actual;
  ASSERT_EQ(n::BeginNativeHistory(row, {0, 1, 1}, &actual), n::NormalStatus::Ok);
  EXPECT_EQ(actual.row.history.staged_force.x, row.history.staged_force.x);
  EXPECT_EQ(actual.row.penetration_auxiliary, row.penetration_auxiliary);
  EXPECT_EQ(actual.row.penetration_offset, row.penetration_offset);
  auto inactive = row; inactive.irtlm[0] = 0;
  ASSERT_EQ(n::BeginNativeHistory(inactive, {1, 1, 1}, &actual), n::NormalStatus::Ok);
  EXPECT_EQ(actual.row.history.normal.damping_half_force, 0);
  EXPECT_EQ(actual.row.penetration_auxiliary, 0);
  EXPECT_EQ(actual.row.penetration_offset, row.penetration_offset);
  EXPECT_EQ(actual.row.history.staged_force.x, row.history.staged_force.x);
}
TEST(Type25HistoryPhase, LeaveUsesExactMarkerAndTimePredicates) {
  for (int marker : {-10, -5, -1, 0, 5}) for (int active : {-7, 0, 7})
    for (double time : {0., n::native_constant::ep20,
                        std::nextafter(n::native_constant::ep20, 0.)}) {
      auto row = Row(); row.irtlm[0] = active; row.irtlm[1] = marker; row.time_s[0] = time;
      auto actual = row;
      ASSERT_EQ(n::EndNativeContact(actual, &actual), n::NormalStatus::Ok);
      SameRow(actual, EndOracle(row));
      EXPECT_EQ(actual.time_s[0], row.time_s[0]);
    }
}
TEST(Type25HistoryPhase, InvalidRangeAndAliasedRetryPreserveWholeRow) {
  const auto row = Row();
  for (auto input : {n::HistoryPhaseInput{-1, 1, 1}, n::HistoryPhaseInput{1, 1, 0},
                    n::HistoryPhaseInput{1, std::numeric_limits<double>::quiet_NaN(), 1}}) {
    n::HistoryPhaseResult actual; actual.row = row; actual.retained_candidate = true;
    EXPECT_EQ(n::BeginNativeHistory(row, input, &actual), n::NormalStatus::InvalidInput);
    SameRow(actual.row, row); EXPECT_TRUE(actual.retained_candidate);
  }
  n::HistoryPhaseResult aliased; aliased.row = row;
  ASSERT_EQ(n::BeginNativeHistory(aliased.row, {1, 1, 1}, &aliased), n::NormalStatus::Ok);
  SameRow(aliased.row, BeginOracle(row, {1, 1, 1}).row);
  auto invalid = row; invalid.irtlm[2] = 0;
  auto before = aliased;
  EXPECT_EQ(n::BeginNativeHistory(invalid, {1, 1, 1}, &aliased), n::NormalStatus::InvalidInput);
  SameRow(aliased.row, before.row);
}
TEST(Type25HistoryPhase, PrescribedClassificationSequenceCombinesNativeHistoryAndResponse) {
  auto actual_row = Row(), native_row = actual_row;
  const double penetration[]{0.002, 0.0022, 0.001, 0., 0.003, 0.0031};
  const double velocity[]{5, 2000, -2000, 0, 20, -20};
  for (unsigned i = 0; i < 6; ++i) {
    n::HistoryPhaseResult begun;
    ASSERT_EQ(n::BeginNativeHistory(actual_row, {1, 1, 1}, &begun), n::NormalStatus::Ok);
    actual_row = begun.row; native_row = BeginOracle(native_row, {1, 1, 1}).row;
    // The fixture prescribes classification; this is not a candidate-search test.
    for (auto* row : {&actual_row, &native_row}) {
      row->irtlm[0] = 7; row->irtlm[1] = penetration[i] == 0 ? -5 : 0;
      row->irtlm[2] = 1; row->irtlm[3] = 1;
      row->time_s[0] = penetration[i] == 0 ? n::native_constant::ep20 : (i + 1) * 1e-5;
    }
    ASSERT_EQ(n::EndNativeContact(actual_row, &actual_row), n::NormalStatus::Ok);
    native_row = EndOracle(native_row); SameRow(actual_row, native_row, false);
    auto c = Basic(); c.input.normal.penetration = penetration[i];
    c.input.normal.time = (i + 1) * 1e-5; c.input.relative_velocity.x = velocity[i];
    c.input.dt12 = i == 0 ? 0.5e-5 : 1e-5; RefreshNormalVelocity(c);
    c.history = actual_row.history; n::NativeFrictionResult actual;
    ASSERT_EQ(n::EvaluateNativeFriction(c.normal_config, c.controls, c.coefficients,
        c.input, c.history, &actual), n::NormalStatus::Ok);
    c.history = native_row.history; const auto expected = Oracle(c);
    Same(actual, expected);
    actual_row.history = actual.history; native_row.history = expected.history;
    SameRow(actual_row, native_row, false);
  }
}
} // namespace type25_friction_test
