// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/Batch.h"

#include <tuple>

namespace {
namespace native = represented_interval_test;
namespace c = tlfea::contact;
namespace batch = c::represented_interval_crossing;
using Status = c::RepresentedIntervalStatus;

using native::VertexFields;
using native::PathFields;
using native::SameEdge;
using native::SameResult;
using native::SameNativeReport;
using native::SameView;

void SameBatch(const batch::BatchReport& first, const batch::BatchReport& second) {
  EXPECT_EQ(std::tie(first.status, first.input_pair, first.native_called,
                     first.batch_offset, first.prior_work,
                     first.completed_pairs, first.completed_batches),
            std::tie(second.status, second.input_pair, second.native_called,
                     second.batch_offset, second.prior_work,
                     second.completed_pairs, second.completed_batches));
  EXPECT_STREQ(first.message, second.message);
  SameNativeReport(first.native_report, second.native_report);
  SameView(first.results, second.results);
  if (!second.results.complete) EXPECT_EQ(second.results.data, nullptr);
}

struct Roster {
  std::vector<c::RepresentedTrianglePath> paths;
  std::vector<c::RepresentedTrianglePair> pairs;

  explicit Roster(std::size_t count) {
    paths.push_back(native::Static(10, native::BaseTriangle()));
    for (std::size_t index = 0; index < count; ++index) {
      const auto eid = 20 + index;
      switch (index % 4) {
        case 0:
          paths.push_back(native::Static(eid, native::BaseTriangle(1)));
          break;
        case 1:
          paths.push_back(native::Static(eid, native::BaseTriangle()));
          break;
        case 2:
          paths.push_back(native::Path(eid, native::BaseTriangle(1),
                                      native::BaseTriangle(-3)));
          break;
        default:
          paths.push_back(native::Path(eid, native::BaseTriangle(1),
              native::BaseTriangle(-1), 0, c::RepresentedMotion::RigidArc));
          break;
      }
      pairs.push_back({0, static_cast<std::uint32_t>(index + 1)});
    }
  }
};

c::RepresentedIntervalLimits Limits(const Roster& roster, std::size_t capacity) {
  c::RepresentedIntervalLimits result;
  result.max_paths = roster.paths.size();
  result.max_input_pairs = capacity;
  result.max_results = capacity;
  result.max_work_per_pair = 31;
  result.max_total_work = capacity * result.max_work_per_pair;
  return result;
}

struct LegacyResult {
  batch::BatchReport report;
  std::vector<c::RepresentedIntervalResult> values;
};

// Independent oracle: each public call reconstructs and authenticates the full
// roster. Deliberately do not call BatchAccess or its detail::Execute helper.
// Inputs here satisfy the adapter's canonical/private-storage preconditions.
LegacyResult LegacyBatch(c::RepresentedIntervalCrossing& owner,
                         const Roster& roster, std::size_t capacity) {
  LegacyResult result;
  result.values.resize(roster.pairs.size());
  auto& report = result.report;
  std::size_t offset = 0, total_work = 0;
  do {
    const auto count = std::min(capacity, roster.pairs.size() - offset);
    report.batch_offset = offset;
    report.prior_work = total_work;
    report.native_report = owner.Certify(roster.paths.data(), roster.paths.size(),
        roster.pairs.empty() ? nullptr : roster.pairs.data() + offset, count);
    report.native_called = true;
    const auto& native_report = report.native_report;
    if (native_report.status != Status::Ok) {
      report.status = native_report.status;
      report.message = native_report.message;
      report.input_pair = native_report.input_pair < count
          ? offset + native_report.input_pair : SIZE_MAX;
      return result;
    }
    const auto current = owner.results();
    if (!current.complete || current.count != count || (count && !current.data)) {
      ADD_FAILURE() << "Public native oracle did not publish its complete slice";
      return result;
    }
    if (count) std::copy_n(current.data, count, result.values.data() + offset);
    total_work += native_report.work;
    offset += count;
    report.completed_pairs = offset;
    ++report.completed_batches;
  } while (offset < roster.pairs.size());
  report.results = {result.values.data(), result.values.size(), true};
  return result;
}

batch::BatchReport Reused(c::RepresentedIntervalCrossing& owner,
                          const Roster& roster, std::size_t capacity,
                          std::vector<c::RepresentedIntervalResult>& scratch) {
  return batch::BatchAccess::Certify(owner, roster.paths.data(), roster.paths.size(),
      roster.pairs.empty() ? nullptr : roster.pairs.data(), roster.pairs.size(),
      capacity, scratch.data(), scratch.size());
}

void OneFullAuthentication(const batch::BatchReport& report, std::size_t paths) {
  EXPECT_EQ(report.path_roster_work.authentications, 1u);
  EXPECT_EQ(report.path_roster_work.path_rows, paths);
  EXPECT_EQ(report.path_roster_work.vertex_rows, 3 * paths);
  EXPECT_EQ(report.path_roster_work.path_sorts, 1u);
  EXPECT_EQ(report.path_roster_work.vertex_sorts, 1u);
}

void Compare(std::size_t count, std::size_t capacity) {
  SCOPED_TRACE(::testing::Message() << "pairs=" << count << " capacity=" << capacity);
  Roster roster(count);
  auto legacy = native::Owner(Limits(roster, capacity));
  auto reused = native::Owner(Limits(roster, capacity));
  std::vector<c::RepresentedIntervalResult> scratch(count);
  const auto expected = LegacyBatch(legacy, roster, capacity);
  const auto actual = Reused(reused, roster, capacity, scratch);
  ASSERT_EQ(expected.report.status, Status::Ok);
  ASSERT_EQ(actual.status, Status::Ok);
  SameBatch(expected.report, actual);
  SameView(legacy.results(), reused.results());
  OneFullAuthentication(actual, roster.paths.size());
  EXPECT_EQ(actual.completed_batches, (count + capacity - 1) / capacity);
}

TEST(RepresentedIntervalBatchRoster, LegacySlicesMatchAtOneTwoAndFivePairs) {
  for (const std::size_t capacity : {1u, 2u, 5u}) Compare(9, capacity);
}

TEST(RepresentedIntervalBatchRoster, WholeRosterAuthenticatesOnceAt256And257Boundary) {
  Compare(256, 256);
  Compare(257, 256);
}

TEST(RepresentedIntervalBatchRoster, EmptyPairsStillAuthenticateEveryUnusedPath) {
  for (bool malformed : {false, true}) {
    SCOPED_TRACE(malformed);
    Roster roster(5);
    roster.pairs.clear();
    if (malformed)
      roster.paths.back().vertices[0].endpoint[1].x =
          std::numeric_limits<double>::infinity();
    auto legacy = native::Owner(Limits(roster, 1));
    auto reused = native::Owner(Limits(roster, 1));
    std::vector<c::RepresentedIntervalResult> scratch;
    const auto expected = LegacyBatch(legacy, roster, 1);
    const auto actual = Reused(reused, roster, 1, scratch);
    SameBatch(expected.report, actual);
    SameView(legacy.results(), reused.results());
    EXPECT_EQ(actual.path_roster_work.authentications, 1u);
    EXPECT_EQ(actual.native_report.input_paths, roster.paths.size());
    if (malformed) {
      EXPECT_EQ(actual.status, Status::InvalidInput);
      EXPECT_EQ(actual.native_report.input_path, roster.paths.size() - 1);
      EXPECT_EQ(actual.completed_batches, 0u);
    } else {
      EXPECT_EQ(actual.status, Status::Ok);
      EXPECT_EQ(actual.completed_batches, 1u);
      OneFullAuthentication(actual, roster.paths.size());
    }
  }
}

TEST(RepresentedIntervalBatchRoster, DistantIdentityConflictsPreserveNativeDiagnosticOrder) {
  for (unsigned mode = 0; mode < 4; ++mode) {
    SCOPED_TRACE(mode);
    Roster roster(9);
    roster.pairs.resize(1);
    if (mode == 0 || mode == 2 || mode == 3) {
      roster.paths.back() = roster.paths[1];
      roster.paths.back().vertices[0].endpoint[1].x += 1;
    }
    if (mode == 1 || mode == 2)
      roster.paths[8] = native::Static(27, native::BaseTriangle(2), 200);
    if (mode == 3)
      roster.paths[3].vertices[0].endpoint[0].x =
          std::numeric_limits<double>::infinity();
    auto legacy = native::Owner(Limits(roster, 1));
    auto reused = native::Owner(Limits(roster, 1));
    std::vector<c::RepresentedIntervalResult> scratch(1);
    const auto expected = LegacyBatch(legacy, roster, 1);
    const auto actual = Reused(reused, roster, 1, scratch);
    ASSERT_NE(expected.report.status, Status::Ok);
    SameBatch(expected.report, actual);
    EXPECT_EQ(actual.completed_batches, 0u);
    EXPECT_EQ(actual.path_roster_work.authentications, 1u);
    EXPECT_EQ(actual.status, mode == 3 ? Status::InvalidInput : Status::IdentityMismatch);
    if (mode == 1) {
      EXPECT_STREQ(actual.message, "inconsistent vertex trajectory identity");
      EXPECT_EQ(actual.path_roster_work.vertex_sorts, 1u);
    } else if (mode == 3) {
      EXPECT_EQ(actual.native_report.input_path, 3u);
      EXPECT_EQ(actual.path_roster_work.path_sorts, 0u);
    } else {
      EXPECT_EQ(actual.path_roster_work.path_sorts, 1u);
      EXPECT_EQ(actual.path_roster_work.vertex_sorts, 0u);
    }
  }
}

TEST(RepresentedIntervalBatchRoster, MalformedPairsPrecedeUnusedPathAuthentication) {
  for (unsigned mode = 0; mode < 3; ++mode) {
    SCOPED_TRACE(mode);
    Roster roster(3);
    roster.pairs.resize(2);
    auto owner = native::Owner(Limits(roster, 2));
    ASSERT_EQ(owner.Certify(roster.paths.data(), roster.paths.size(),
                           roster.pairs.data(), roster.pairs.size()).status, Status::Ok);
    const auto previous = owner.results();
    const std::vector<c::RepresentedIntervalResult> saved(
        previous.data, previous.data + previous.count);
    roster.paths.back().vertices[0].endpoint[1].x =
        std::numeric_limits<double>::infinity();
    if (mode == 0) std::swap(roster.pairs[0], roster.pairs[1]);
    if (mode == 1)
      roster.pairs[1].second = static_cast<std::uint32_t>(roster.paths.size());
    if (mode == 2) roster.pairs[1] = roster.pairs[0];
    std::vector<c::RepresentedIntervalResult> scratch(2);
    const auto report = Reused(owner, roster, 1, scratch);
    EXPECT_EQ(report.status, mode == 1 ? Status::InvalidInput : Status::IdentityMismatch);
    EXPECT_EQ(report.input_pair, 1u);
    EXPECT_FALSE(report.native_called);
    EXPECT_EQ(report.path_roster_work.authentications, 0u);
    EXPECT_EQ(report.path_roster_work.path_rows, 0u);
    EXPECT_EQ(report.path_roster_work.vertex_rows, 0u);
    EXPECT_EQ(report.path_roster_work.path_sorts, 0u);
    EXPECT_EQ(report.path_roster_work.vertex_sorts, 0u);
    EXPECT_FALSE(report.results.complete);
    EXPECT_EQ(report.results.count, 0u);
    EXPECT_EQ(report.results.data, nullptr);
    EXPECT_EQ(owner.results().data, previous.data);
    SameView({saved.data(), saved.size(), true}, owner.results());
  }
}

TEST(RepresentedIntervalBatchRoster, CompatibleUnusedDuplicateDoesNotChangeResults) {
  Roster roster(3);
  roster.paths.push_back(roster.paths[1]);
  auto legacy = native::Owner(Limits(roster, 1));
  auto reused = native::Owner(Limits(roster, 1));
  std::vector<c::RepresentedIntervalResult> scratch(roster.pairs.size());
  const auto expected = LegacyBatch(legacy, roster, 1);
  const auto actual = Reused(reused, roster, 1, scratch);
  ASSERT_EQ(actual.status, Status::Ok);
  SameBatch(expected.report, actual);
  OneFullAuthentication(actual, roster.paths.size());
}

TEST(RepresentedIntervalBatchRoster, LateWorkFailurePreservesLastNativeSliceAndLimits) {
  Roster roster(2);
  roster.paths[2] = native::Path(21, native::BaseTriangle(1), native::BaseTriangle(-3));
  auto limits = Limits(roster, 1);
  limits.max_total_work = 1;
  auto legacy = native::Owner(limits);
  auto reused = native::Owner(limits);
  std::vector<c::RepresentedIntervalResult> scratch(2);
  const auto expected = LegacyBatch(legacy, roster, 1);
  const auto actual = Reused(reused, roster, 1, scratch);
  ASSERT_EQ(actual.status, Status::ResourceLimit);
  SameBatch(expected.report, actual);
  EXPECT_EQ(actual.completed_batches, 1u);
  EXPECT_EQ(actual.completed_pairs, 1u);
  EXPECT_EQ(actual.input_pair, 1u);
  EXPECT_EQ(actual.prior_work, 1u);
  EXPECT_EQ(actual.native_report.total_work_limit, 1u);
  EXPECT_EQ(actual.native_report.rejected_pair_work, 2u);
  ASSERT_TRUE(reused.results().complete);
  ASSERT_EQ(reused.results().count, 1u);
  SameView(legacy.results(), reused.results());
  SameResult(scratch[0], reused.results().data[0]);
  OneFullAuthentication(actual, roster.paths.size());
}

TEST(RepresentedIntervalBatchRoster, SeparateCallsReauthenticateMutationFailureAndRetry) {
  Roster roster(3);
  auto legacy = native::Owner(Limits(roster, 1));
  auto reused = native::Owner(Limits(roster, 1));
  std::vector<c::RepresentedIntervalResult> scratch(3);
  const auto saved = roster.paths[2];
  for (unsigned attempt = 0; attempt < 5; ++attempt) {
    SCOPED_TRACE(attempt);
    if (attempt == 1)
      roster.paths[2] = native::Static(21, native::BaseTriangle(2));
    if (attempt == 2)
      roster.paths[2] = native::Static(21, native::BaseTriangle(2), 200);
    if (attempt == 4) roster.paths[2] = saved;
    const auto old_view = reused.results();
    std::vector<c::RepresentedIntervalResult> previous;
    if (old_view.count) previous.assign(old_view.data, old_view.data + old_view.count);
    const auto expected = LegacyBatch(legacy, roster, 1);
    const auto actual = Reused(reused, roster, 1, scratch);
    SameBatch(expected.report, actual);
    SameView(legacy.results(), reused.results());
    OneFullAuthentication(actual, roster.paths.size());
    if (attempt == 2 || attempt == 3) {
      EXPECT_EQ(actual.status, Status::IdentityMismatch);
      EXPECT_EQ(reused.results().data, old_view.data);
      SameView({previous.data(), previous.size(), old_view.complete}, reused.results());
    } else {
      ASSERT_EQ(actual.status, Status::Ok);
      EXPECT_EQ(actual.results.data[1].classification, attempt == 1
          ? c::RepresentedIntervalClassification::CertifiedSeparated
          : c::RepresentedIntervalClassification::CertifiedCrossingContact);
    }
  }
}

TEST(RepresentedIntervalBatchRoster, ExpiredNativeScratchIsRejectedWithoutPublicationChange) {
  Roster roster(2);
  auto owner = native::Owner(Limits(roster, 2));
  ASSERT_EQ(owner.Certify(roster.paths.data(), roster.paths.size(),
                         roster.pairs.data(), roster.pairs.size()).status, Status::Ok);
  const auto expired = owner.results();
  ASSERT_EQ(owner.Certify(roster.paths.data(), roster.paths.size(),
                         roster.pairs.data(), roster.pairs.size()).status, Status::Ok);
  const auto current = owner.results();
  ASSERT_NE(expired.data, current.data);
  const std::vector<c::RepresentedIntervalResult> previous(
      current.data, current.data + current.count);
  // Passing the public owner object as output must fail before any write.
  const c::RepresentedIntervalResultView owner_storage{
      reinterpret_cast<const c::RepresentedIntervalResult*>(&owner),
      roster.pairs.size(), false};
  for (const auto view : {expired, current, owner_storage}) {
    const auto report = batch::BatchAccess::Certify(owner,
        roster.paths.data(), roster.paths.size(), roster.pairs.data(),
        roster.pairs.size(), 1,
        const_cast<c::RepresentedIntervalResult*>(view.data), view.count);
    EXPECT_EQ(report.status, Status::InvalidInput);
    EXPECT_FALSE(report.native_called);
    EXPECT_EQ(report.path_roster_work.authentications, 0u);
    EXPECT_FALSE(report.results.complete);
    EXPECT_EQ(report.results.data, nullptr);
    EXPECT_EQ(owner.results().data, current.data);
    SameView({previous.data(), previous.size(), true}, owner.results());
  }
}

TEST(RepresentedIntervalBatchRoster, PerPairWorkExhaustionRemainsAnExplicitUnresolvedResult) {
  Roster roster(3);
  auto limits = Limits(roster, 1);
  limits.max_work_per_pair = 1;
  limits.max_total_work = 1;
  auto legacy = native::Owner(limits);
  auto reused = native::Owner(limits);
  std::vector<c::RepresentedIntervalResult> scratch(3);
  const auto expected = LegacyBatch(legacy, roster, 1);
  const auto actual = Reused(reused, roster, 1, scratch);
  ASSERT_EQ(actual.status, Status::Ok);
  SameBatch(expected.report, actual);
  ASSERT_TRUE(actual.results.complete);
  EXPECT_EQ(actual.results.data[2].classification,
            c::RepresentedIntervalClassification::Unresolved);
  EXPECT_EQ(actual.results.data[2].reason, c::RepresentedIntervalReason::WorkExhausted);
  EXPECT_EQ(actual.results.data[2].work, 1u);
  OneFullAuthentication(actual, roster.paths.size());
}

}  // namespace
