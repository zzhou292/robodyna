// SPDX-License-Identifier: MIT
#include "HostSupport.h"

namespace t3_readback_test {
TEST(T3ActivityReadbackHost,AllRolesAndCountsKeepTheFrozenCallAndFreshResults) {
  for (auto count : {1u, 2u, 7u, 129u}) for (bool mapped : {false, true}) {
    State old(count), next(count);
    if (!mapped) { old.physical = nullptr; next.physical = nullptr; }
    const auto a = serial::Activity(old, 0, 0, 0);
    const auto b = candidate::Activity(next, 0, 0, 0);
    Same(a, b);
    EXPECT_EQ(b.status, Status::Success);
    EXPECT_EQ(old.copies, mapped ? 2u : 1u);
    EXPECT_EQ(next.copies, 1u);
    EXPECT_EQ(old.pending_checks, next.pending_checks);
    EXPECT_EQ(next.histories.reads, 1u);
  }
}
TEST(T3ActivityReadbackHost,NoOnePointScopePreservesBothMappedAndLegacyReadCounts) {
  for (bool mapped : {false, true}) {
    State old, next;
    old.histories.one_point = next.histories.one_point = false;
    if (!mapped) { old.physical = nullptr; next.physical = nullptr; }
    Same(serial::Activity(old, 0, 0, 0), candidate::Activity(next, 0, 0, 0));
    EXPECT_EQ(next.copies, mapped ? 1u : 0u);
    EXPECT_EQ(old.copies, next.copies);
  }
}
TEST(T3ActivityReadbackHost,EarlierSectionForceAndRoleErrorsStillDominateOnePointIdentity) {
  for (unsigned failure = 0; failure < 4; ++failure) {
    State old, next;
    for (auto* state : {&old, &next}) {
      auto point = *state->histories.input_sections[4].one_point();
      point.point.reported_thickness_m *= 2;
      state->histories.input_sections[4] = fe::ShellBatchLayeredSection::OnePoint(point);
      if (failure == 0) state->histories.fault = Setup::NonfiniteResult;
      if (failure == 1) state->device[6].internal_force[0].x = INFINITY;
      if (failure == 2) state->histories.input_sections[3] = fe::ShellBatchLayeredSection::Elastic({});
    }
    const auto a = serial::Activity(old, 0, 0, 0);
    const auto b = candidate::Activity(next, 0, 0, 0);
    Same(a, b);
    EXPECT_EQ(b.status, Status::NonfiniteResult);
    if (failure == 1) EXPECT_EQ(b.element, 6u);
    if (failure == 2) EXPECT_EQ(b.element, 3u);
    if (failure == 3) EXPECT_EQ(b.element, 4u);
  }
}
TEST(T3ActivityReadbackHost,MissingSourceAndPendingErrorsKeepOriginalPrecedence) {
  for (unsigned failure = 0; failure < 5; ++failure) {
    State old, next;
    for (auto* state : {&old, &next}) {
      state->fail_pending = 3; // Old second read / new same-call pending-error check.
      if (failure == 0) state->joined_binding = nullptr;
      if (failure == 1) state->histories.catalog_present = false;
      if (failure == 2) state->fail_pending = 1;
      if (failure == 3) state->fail_copy = 1;
    }
    Same(serial::Activity(old, 0, 0, 0), candidate::Activity(next, 0, 0, 0));
    EXPECT_EQ(old.poisoned, next.poisoned);
    EXPECT_EQ(old.pending_checks, next.pending_checks);
  }
}
TEST(T3ActivityReadbackHost,RepeatedCallsCannotReuseStaleStagingOrAFormerSuccess) {
  State state;
  EXPECT_EQ(candidate::Activity(state, 0, 0, 0).status, Status::Success);
  state.device[6].internal_couple[2].z = NAN;
  EXPECT_EQ(candidate::Activity(state, 0, 0, 0).status, Status::NonfiniteResult);
  state.device[6].internal_couple[2].z = 0;
  EXPECT_EQ(candidate::Activity(state, 0, 0, 0).status, Status::Success);
  EXPECT_EQ(state.copies, 3u);
  EXPECT_EQ(state.histories.reads, 3u);
}
TEST(T3ActivityReadbackHost,OnePointLoopStillChecksReferenceTimeEpochAndSavedState) {
  for (unsigned failure = 0; failure < 5; ++failure) {
    State old, next;
    for (auto* state : {&old, &next}) {
      state->physical = nullptr; // Isolate the full one-point wrapper, which remains fresh.
      if (failure == 0) state->binding.reference.input.node_ids[0] += 1;
      if (failure == 1) state->catalog.absent_parameter = 4;
      if (failure == 2) {
        auto point = *state->histories.input_sections[4].one_point();
        point.point.saved.stress[0] = 1;
        state->histories.input_sections[4] = fe::ShellBatchLayeredSection::OnePoint(point);
      }
    }
    const auto time = failure == 3 ? 1.0 : 0.0;
    const auto epoch = failure == 4 ? 1u : 0u;
    const auto a = serial::Activity(old, 0, time, epoch);
    const auto b = candidate::Activity(next, 0, time, epoch);
    Same(a, b);
    EXPECT_EQ(b.status, Status::NonfiniteResult);
    EXPECT_EQ(next.copies, 1u);
  }
}
} // namespace t3_readback_test
