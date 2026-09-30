// SPDX-License-Identifier: MIT
#include "../qbat_resident/ResidentFixture.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace qbat_resident_test {
namespace {
void CheckPreparedActivity(Rig& rig, const Prepared& candidate) {
  const auto& diagnostics = candidate.diagnostics;
  const auto check = [](auto& batch, auto& owner, const auto& token,
      const auto& expected, const auto& truth) {
    using Status = decltype(batch.CopyPreparedParentActivity(
        owner, token, expected, nullptr, 0).status);
    std::vector<std::uint8_t> flags(truth.size(), 19);
    ASSERT_EQ(batch.CopyPreparedParentActivity(owner, token, expected,
        flags.data(), flags.size()).status, Status::Success);
    EXPECT_EQ(flags, truth);
  };
  if (rig.binding->qeph_count()) {
    std::vector<fe::ShellBatchFailureState> history(rig.binding->qeph_count());
    ASSERT_EQ(rig.qeph.CopyPreparedFailureHistory(diagnostics.qeph,
        history.data(), history.size()).status, fe::qeph::BatchStatus::Success);
    std::vector<std::uint8_t> truth;
    for (const auto& row : history) truth.push_back(row.active ? 1 : 0);
    check(rig.qeph, rig.owner, candidate.token, diagnostics.qeph, truth);
  }
  if (rig.binding->t3_count()) {
    std::vector<fe::ShellBatchLayeredSection> history(rig.binding->t3_count());
    std::vector<fe::t3::ForceTrial> forces(history.size());
    ASSERT_EQ(rig.t3.CopyPreparedLayeredSectionHistory(diagnostics.t3,
        history.data(), history.size()).status, fe::t3::BatchStatus::Success);
    ASSERT_EQ(rig.t3.CopyPreparedResults(diagnostics.t3,
        forces.data(), forces.size()).status, fe::t3::BatchStatus::Success);
    std::vector<std::uint8_t> truth;
    for (std::size_t parent = 0; parent < history.size(); ++parent) {
      const auto* point = history[parent].one_point();
      const bool active = point ? point->point.failure.history.point_active :
          forces[parent].proposed_history.data().active;
      truth.push_back(active ? 1 : 0);
    }
    check(rig.t3, rig.owner, candidate.token, diagnostics.t3, truth);
  }
  std::vector<std::uint8_t> truth;
  for (const auto& row : candidate.qbat) truth.push_back(row.history.element_active ? 1 : 0);
  check(rig.qbat, rig.owner, candidate.token, diagnostics.qbat, truth);
}
template<class Batch, class Diagnostics>
void CheckRejected(Batch& batch, Rig& rig, const Prepared& candidate,
    const Diagnostics& expected, std::size_t count) {
  using Status = decltype(batch.CopyPreparedParentActivity(
      rig.owner, candidate.token, expected, nullptr, 0).status);
  std::vector<std::uint8_t> flags(count, 19);
  const auto unchanged = flags;
  EXPECT_EQ(batch.CopyPreparedParentActivity(rig.owner, candidate.token, expected,
      flags.data(), SIZE_MAX).status, Status::ResourceLimit);
  auto stale = expected;
  ++stale.attempt;
  EXPECT_EQ(batch.CopyPreparedParentActivity(rig.owner, candidate.token, stale,
      flags.data(), count).status, Status::StaleTrial);
  fe::FENodalState foreign;
  EXPECT_EQ(batch.CopyPreparedParentActivity(foreign, candidate.token, expected,
      flags.data(), count).status, Status::StaleTrial);
  EXPECT_EQ(batch.CopyPreparedParentActivity(rig.owner, candidate.token, expected,
      reinterpret_cast<std::uint8_t*>(const_cast<fe::NodalTrialToken*>(&candidate.token)), count).status,
      Status::InvalidInput);
  EXPECT_EQ(batch.CopyPreparedParentActivity(rig.owner, candidate.token, expected,
      reinterpret_cast<std::uint8_t*>(&rig.owner), count).status, Status::InvalidInput);
  EXPECT_EQ(batch.CopyPreparedParentActivity(rig.owner, candidate.token, expected,
      reinterpret_cast<std::uint8_t*>(const_cast<Diagnostics*>(&expected)), count).status,
      Status::InvalidInput);
  // This tiny legacy binding uses inline nodes copied into each participant.
  // Its caller-owned node array is not a borrowed retained-source view.
  EXPECT_EQ(batch.CopyPreparedParentActivity(rig.owner, candidate.token, expected,
      reinterpret_cast<std::uint8_t*>(&batch), count).status, Status::InvalidInput);
  EXPECT_EQ(flags, unchanged);
}
}
TEST(ShellPreparedActivityCuda, CompleteFamiliesRequireActualTokenAndProtectAllOutputs) {
  Source source(true);
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope()));
  Prepared candidate;
  ASSERT_TRUE(rig.Prepare(candidate, 1));
  ASSERT_NO_FATAL_FAILURE(CheckPreparedActivity(rig, candidate));
  ASSERT_NO_FATAL_FAILURE(CheckRejected(rig.qeph, rig, candidate,
      candidate.diagnostics.qeph, rig.binding->qeph_count()));
  ASSERT_NO_FATAL_FAILURE(CheckRejected(rig.t3, rig, candidate,
      candidate.diagnostics.t3, rig.binding->t3_count()));
  ASSERT_NO_FATAL_FAILURE(CheckRejected(rig.qbat, rig, candidate,
      candidate.diagnostics.qbat, rig.binding->qbat_count()));
  ASSERT_NO_FATAL_FAILURE(CheckPreparedActivity(rig, candidate));
  // BorrowPrepared rejects a bad token by discarding that owner's attempt.
  // Verify this contract after the non-destructive output checks, then retry
  // the complete attempt rather than treating an invalidated token as live.
  fe::NodalTrialToken invalid;
  std::vector<std::uint8_t> rejected(rig.binding->qbat_count(), 19);
  EXPECT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner, invalid,
      candidate.diagnostics.qbat, rejected.data(), rejected.size()).status, qb::BatchStatus::StaleTrial);
  EXPECT_EQ(rejected, std::vector<std::uint8_t>(rejected.size(), 19));
  rig.Discard();
  std::uint8_t flag = 19;
  EXPECT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner, candidate.token,
      candidate.diagnostics.qbat, &flag, rig.binding->qbat_count()).status, qb::BatchStatus::StaleTrial);
  EXPECT_EQ(flag, 19);
  ASSERT_TRUE(rig.Prepare(candidate, 1));
  ASSERT_NO_FATAL_FAILURE(CheckPreparedActivity(rig, candidate));
  ASSERT_TRUE(rig.Commit(candidate));
}
TEST(ShellPreparedActivityCuda, RemovalUsesTypedOnePointAndQbatCandidateWithoutPublishing) {
  MidlayerSource source(true, 1.e-5);
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope(), false, false, 0x1p-12));
  bool removed_t3 = false, removed_qbat = false;
  for (unsigned step = 0; step < 24; ++step) {
    const auto stamp = rig.owner.accepted();
    std::uint8_t before = 19, after = 19;
    fe::t3::BatchDiagnostics diagnostic;
    ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(stamp, &before, 1, &diagnostic).status,
        fe::t3::BatchStatus::Success);
    Prepared candidate;
    ASSERT_TRUE(rig.Prepare(candidate, 30));
    ASSERT_NO_FATAL_FAILURE(CheckPreparedActivity(rig, candidate));
    ASSERT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner, candidate.token,
        candidate.diagnostics.t3, &after, 1).status, fe::t3::BatchStatus::Success);
    removed_t3 |= after == 0;
    for (const auto& row : candidate.qbat) removed_qbat |= !row.history.element_active;
    std::uint8_t still_accepted = 19;
    ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(stamp, &still_accepted, 1, &diagnostic).status,
        fe::t3::BatchStatus::Success);
    EXPECT_EQ(still_accepted, before);
    EXPECT_TRUE(fe::trial_identity::SameStamp(rig.owner.accepted(), stamp));
    if (step == 0) {
      rig.Discard();
      ASSERT_TRUE(rig.Prepare(candidate, 30));
      ASSERT_NO_FATAL_FAILURE(CheckPreparedActivity(rig, candidate));
    }
    ASSERT_TRUE(rig.Commit(candidate));
  }
  EXPECT_TRUE(removed_t3);
  EXPECT_TRUE(removed_qbat);
}
TEST(ShellPreparedActivityCuda, LastReadFailureInvalidatesOnlyCandidateThenExactRetry) {
  Source source(true);
  Rig rig;
  ASSERT_TRUE(rig.Initialize(source.Scope()));
  const auto stamp = rig.owner.accepted();
  for (const auto fault : {ReadFault::LateNonfinite, ReadFault::InvalidFlag}) {
    Prepared candidate;
    ASSERT_TRUE(rig.Prepare(candidate, 1));
    std::vector<std::uint8_t> flags(rig.binding->qbat_count(), 19);
    Arm(fault, flags.size());
    EXPECT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner, candidate.token,
        candidate.diagnostics.qbat, flags.data(), flags.size()).status, qb::BatchStatus::NonfiniteResult);
    EXPECT_EQ(flags, std::vector<std::uint8_t>(flags.size(), 19));
    EXPECT_EQ(rig.qbat.CopyPreparedParentActivity(rig.owner, candidate.token,
        candidate.diagnostics.qbat, flags.data(), flags.size()).status, qb::BatchStatus::StaleTrial);
    EXPECT_TRUE(fe::trial_identity::SameStamp(rig.owner.accepted(), stamp));
    qb::BatchDiagnostics diagnostic;
    ASSERT_EQ(rig.qbat.CopyAcceptedParentActivity(stamp, flags.data(), flags.size(), &diagnostic).status,
        qb::BatchStatus::Success);
    EXPECT_EQ(flags, std::vector<std::uint8_t>(flags.size(), 1));
    rig.Discard();
  }
  Prepared retry;
  ASSERT_TRUE(rig.Prepare(retry, 1));
  ASSERT_NO_FATAL_FAILURE(CheckPreparedActivity(rig, retry));
  ASSERT_TRUE(rig.Commit(retry));
}
} // namespace qbat_resident_test
