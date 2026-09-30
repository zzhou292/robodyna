// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../represented_interval_crossing/Fixture.h"
#include "lib_src/collision/SelfContactTransactionTypes.h"
#include "lib_src/collision/self_contact_transaction/CrossingBatch.h"

#include <tuple>

namespace {
namespace native = represented_interval_test;
namespace c = tlfea::contact;
namespace batch = c::self_contact_transaction;
using Status = c::RepresentedIntervalStatus;

auto VertexFields(const c::FacetVertexKey& value) {
  return std::tie(value.source_instance_id, value.kind, value.first,
                  value.second, value.numerator, value.denominator,
                  value.level, value.grid_i, value.grid_j);
}

auto PathFields(const c::RepresentedTrianglePathKey& value) {
  return std::tie(value.source_instance_id, value.parent_eid,
                  value.level, value.local_facet);
}

void SameEdge(const c::FacetEdgeKey& first, const c::FacetEdgeKey& second) {
  EXPECT_EQ(first.parent_boundary, second.parent_boundary);
  EXPECT_EQ(first.parent_eid, second.parent_eid);
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint)
    EXPECT_EQ(VertexFields(first.endpoints[endpoint]),
              VertexFields(second.endpoints[endpoint]));
}

void SameResult(const c::RepresentedIntervalResult& first,
                const c::RepresentedIntervalResult& second) {
  for (unsigned side = 0; side < 2; ++side) {
    EXPECT_EQ(PathFields(first.key.paths[side]),
              PathFields(second.key.paths[side]));
    SameEdge(first.feature.edges[side], second.feature.edges[side]);
  }
  EXPECT_EQ(first.feature.kind, second.feature.kind);
  EXPECT_EQ(VertexFields(first.feature.vertex), VertexFields(second.feature.vertex));
  EXPECT_EQ(PathFields(first.feature.face), PathFields(second.feature.face));
  EXPECT_EQ(first.classification, second.classification);
  EXPECT_EQ(first.reason, second.reason);
  EXPECT_EQ(first.geometry, second.geometry);
  EXPECT_EQ(first.witness_time_numerator, second.witness_time_numerator);
  EXPECT_EQ(first.witness_time_depth, second.witness_time_depth);
  EXPECT_EQ(first.work, second.work);
  EXPECT_EQ(first.accepted_event, second.accepted_event);
}

void Incomplete(const batch::CrossingBatchReport& report) {
  EXPECT_FALSE(report.results.complete);
  EXPECT_EQ(report.results.count, 0u);
  EXPECT_EQ(report.results.data, nullptr);
}

void SameProjection(const batch::CrossingBatchReport& report) {
  const auto value = batch::CrossingBatchDiagnostics(report);
  ASSERT_EQ(value.available, report.native_called);
  if (!value.available) return;
  const auto& native_report = report.native_report;
  EXPECT_EQ(value.batch_pair_offset, report.batch_offset);
  EXPECT_EQ(value.prior_batch_work, report.prior_work);
  EXPECT_EQ(value.input_path, native_report.input_path);
  EXPECT_EQ(value.input_pair, native_report.input_pair);
  EXPECT_EQ(value.input_paths, native_report.input_paths);
  EXPECT_EQ(value.input_pairs, native_report.input_pairs);
  EXPECT_EQ(value.unique_pairs, native_report.unique_pairs);
  EXPECT_EQ(value.certified_separated, native_report.certified_separated);
  EXPECT_EQ(value.certified_crossing_contact, native_report.certified_crossing_contact);
  EXPECT_EQ(value.unresolved, native_report.unresolved);
  EXPECT_EQ(value.admitted_work, native_report.work);
  EXPECT_EQ(value.total_work_limit, native_report.total_work_limit);
  EXPECT_EQ(value.rejected_pair_work, native_report.rejected_pair_work);
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

c::RepresentedIntervalLimits Limits(std::size_t pairs, std::size_t total_work) {
  c::RepresentedIntervalLimits result;
  result.max_paths = pairs + 1;
  result.max_input_pairs = pairs;
  result.max_results = pairs;
  result.max_work_per_pair = 31;
  result.max_total_work = total_work;
  return result;
}

void CompareBatches(std::size_t count, std::size_t batch_size) {
  SCOPED_TRACE(::testing::Message() << "pairs=" << count << " batch=" << batch_size);
  Roster roster(count);
  auto whole = native::Owner(Limits(count, count * 31));
  const auto expected_report = whole.Certify(
      roster.paths.data(), roster.paths.size(), roster.pairs.data(), count);
  ASSERT_EQ(expected_report.status, Status::Ok);
  const auto expected = whole.results();
  ASSERT_TRUE(expected.complete);
  ASSERT_EQ(expected.count, count);

  auto split = native::Owner(Limits(count, batch_size * 31));
  std::vector<c::RepresentedIntervalResult> scratch(count);
  const auto report = batch::CertifyCrossingBatches(
      split, roster.paths.data(), roster.paths.size(), roster.pairs.data(),
      count, batch_size, scratch.data(), scratch.size());
  ASSERT_EQ(report.status, Status::Ok);
  ASSERT_TRUE(report.results.complete);
  ASSERT_EQ(report.results.count, count);
  EXPECT_EQ(report.results.data, scratch.data());
  EXPECT_EQ(report.completed_pairs, count);
  EXPECT_EQ(report.completed_batches, (count + batch_size - 1) / batch_size);
  EXPECT_EQ(report.prior_work + report.native_report.work, expected_report.work);
  for (std::size_t pair = 0; pair < count; ++pair) {
    SCOPED_TRACE(pair);
    SameResult(expected.data[pair], report.results.data[pair]);
  }
  SameProjection(report);
}

TEST(SelfContactCrossingBatch, AdmissionDivisionPreservesZeroAndOverflowBoundaries) {
  EXPECT_EQ(batch::RawCrossingBatchPairCapacity(4096, 4095, 1u << 20), 256u);
  EXPECT_EQ(batch::RawCrossingBatchPairCapacity(2, 4095, 1u << 20), 2u);
  EXPECT_EQ(batch::RawCrossingBatchPairCapacity(4096, 4095, 1), 1u);
  EXPECT_EQ(batch::RawCrossingBatchPairCapacity(SIZE_MAX, SIZE_MAX, SIZE_MAX), 1u);
  EXPECT_EQ(batch::RawCrossingBatchPairCapacity(SIZE_MAX, 1, SIZE_MAX), SIZE_MAX);
  EXPECT_EQ(batch::RawCrossingBatchPairCapacity(0, 1, 1), 0u);
  EXPECT_EQ(batch::RawCrossingBatchPairCapacity(1, 0, 1), 0u);
  EXPECT_EQ(batch::RawCrossingBatchPairCapacity(1, 1, 0), 0u);
}

TEST(SelfContactCrossingBatch, MixedNativeFieldsMatchAtSingleAndMultiplePairBatches) {
  for (const std::size_t size : {1u, 2u, 5u}) CompareBatches(9, size);
}

TEST(SelfContactCrossingBatch, NativeFieldsMatchAt256And257PairBoundary) {
  CompareBatches(256, 256);
  CompareBatches(257, 256);
}

TEST(SelfContactCrossingBatch, FullPathContradictionIsCheckedBeforeFirstBatch) {
  Roster roster(2);
  // The second pair uses the same immutable source vertices as the first,
  // but declares a different position. A one-pair slice must still see it.
  roster.paths[2] = native::Static(21, native::BaseTriangle(2), 200);
  auto owner = native::Owner(Limits(2, 31));
  std::vector<c::RepresentedIntervalResult> scratch(2);
  const auto report = batch::CertifyCrossingBatches(owner, roster.paths.data(),
      roster.paths.size(), roster.pairs.data(), 2, 1, scratch.data(), 2);
  EXPECT_EQ(report.status, Status::IdentityMismatch);
  EXPECT_TRUE(report.native_called);
  EXPECT_EQ(report.completed_pairs, 0u);
  EXPECT_EQ(report.completed_batches, 0u);
  EXPECT_EQ(report.native_report.input_paths, 3u);
  EXPECT_STREQ(report.message, "inconsistent vertex trajectory identity");
  Incomplete(report);
  SameProjection(report);
}

TEST(SelfContactCrossingBatch, LateNativeWorkFailureRetainsOffsetAndOriginalReport) {
  Roster roster(2);
  roster.paths[2] = native::Path(21, native::BaseTriangle(1), native::BaseTriangle(-3));
  auto owner = native::Owner(Limits(2, 1));
  std::vector<c::RepresentedIntervalResult> scratch(2);
  const auto report = batch::CertifyCrossingBatches(owner, roster.paths.data(),
      roster.paths.size(), roster.pairs.data(), 2, 1, scratch.data(), 2);
  ASSERT_EQ(report.status, Status::ResourceLimit);
  EXPECT_TRUE(report.native_called);
  EXPECT_EQ(report.batch_offset, 1u);
  EXPECT_EQ(report.input_pair, 1u);
  EXPECT_EQ(report.prior_work, 1u);
  EXPECT_EQ(report.completed_pairs, 1u);
  EXPECT_EQ(report.completed_batches, 1u);
  EXPECT_EQ(report.native_report.status, Status::ResourceLimit);
  EXPECT_EQ(report.native_report.input_path, SIZE_MAX);
  EXPECT_EQ(report.native_report.input_pair, 0u);
  EXPECT_EQ(report.native_report.input_paths, 3u);
  EXPECT_EQ(report.native_report.input_pairs, 1u);
  EXPECT_EQ(report.native_report.unique_pairs, 1u);
  EXPECT_EQ(report.native_report.work, 0u);
  EXPECT_EQ(report.native_report.total_work_limit, 1u);
  EXPECT_EQ(report.native_report.rejected_pair_work, 2u);
  EXPECT_STREQ(report.message, report.native_report.message);
  Incomplete(report);
  SameProjection(report);
  // Native failure retains its previous single-pair publication, but the
  // adapter must not expose that prefix as a complete two-pair result.
  ASSERT_TRUE(owner.results().complete);
  ASSERT_EQ(owner.results().count, 1u);
  SameResult(scratch[0], owner.results().data[0]);
}

TEST(SelfContactCrossingBatch, NoncanonicalAndOutOfRangeRostersRejectBeforeCalls) {
  Roster roster(2);
  auto owner = native::Owner(Limits(2, 62));
  std::vector<c::RepresentedIntervalResult> scratch(2);
  for (unsigned mode = 0; mode < 4; ++mode) {
    SCOPED_TRACE(mode);
    auto pairs = roster.pairs;
    if (mode == 0) std::swap(pairs[0], pairs[1]);
    if (mode == 1) pairs[1] = pairs[0];
    if (mode == 2) pairs[0] = {0, 0};
    if (mode == 3) pairs[0].second = 3;
    const auto report = batch::CertifyCrossingBatches(owner, roster.paths.data(),
        roster.paths.size(), pairs.data(), 2, 1, scratch.data(), 2);
    EXPECT_EQ(report.status, mode == 3 ? Status::InvalidInput : Status::IdentityMismatch);
    EXPECT_FALSE(report.native_called);
    EXPECT_EQ(report.completed_pairs, 0u);
    Incomplete(report);
    SameProjection(report);
  }
}

TEST(SelfContactCrossingBatch, PrivateStorageCapacityAndAliasFailuresDoNotPublish) {
  Roster roster(2);
  auto owner = native::Owner(Limits(2, 62));
  std::vector<c::RepresentedIntervalResult> scratch(2);
  for (unsigned mode = 0; mode < 7; ++mode) {
    SCOPED_TRACE(mode);
    auto* output = scratch.data();
    std::size_t capacity = 2, admitted = 1;
    if (mode == 0) capacity = 1;
    if (mode == 1) admitted = 0;
    if (mode == 2) admitted = 3;
    if (mode == 3) output = nullptr;
    if (mode == 4) output = reinterpret_cast<c::RepresentedIntervalResult*>(roster.paths.data());
    if (mode == 5) output = reinterpret_cast<c::RepresentedIntervalResult*>(roster.pairs.data());
    if (mode == 6) capacity = SIZE_MAX;
    const auto report = batch::CertifyCrossingBatches(owner, roster.paths.data(),
        roster.paths.size(), roster.pairs.data(), 2, admitted, output, capacity);
    EXPECT_EQ(report.status, mode < 3 ? Status::ResourceLimit : Status::InvalidInput);
    EXPECT_FALSE(report.native_called);
    Incomplete(report);
  }
  ASSERT_EQ(owner.Certify(roster.paths.data(), roster.paths.size(),
                         roster.pairs.data(), 2).status, Status::Ok);
  const auto previous = owner.results();
  const auto report = batch::CertifyCrossingBatches(owner, roster.paths.data(),
      roster.paths.size(), roster.pairs.data(), 2, 1,
      const_cast<c::RepresentedIntervalResult*>(previous.data), previous.count);
  EXPECT_EQ(report.status, Status::InvalidInput);
  EXPECT_FALSE(report.native_called);
  Incomplete(report);
  EXPECT_EQ(owner.results().data, previous.data);
}

TEST(SelfContactCrossingBatch, EmptyRosterStillAuthenticatesPathsAndOwner) {
  Roster roster(1);
  c::RepresentedIntervalCrossing absent;
  auto report = batch::CertifyCrossingBatches(absent, nullptr, 0, nullptr, 0, 1, nullptr, 0);
  EXPECT_EQ(report.status, Status::NotInitialized);
  Incomplete(report);
  auto owner = native::Owner(Limits(1, 31));
  report = batch::CertifyCrossingBatches(owner, roster.paths.data(),
      roster.paths.size(), nullptr, 0, 1, nullptr, 0);
  ASSERT_EQ(report.status, Status::Ok);
  EXPECT_TRUE(report.results.complete);
  EXPECT_EQ(report.results.count, 0u);
  EXPECT_EQ(report.completed_batches, 1u);
  EXPECT_EQ(report.native_report.input_paths, 2u);
  SameProjection(report);
  roster.paths[1].vertices[0].endpoint[0].x = std::numeric_limits<double>::infinity();
  report = batch::CertifyCrossingBatches(owner, roster.paths.data(),
      roster.paths.size(), nullptr, 0, 1, nullptr, 0);
  EXPECT_EQ(report.status, Status::InvalidInput);
  EXPECT_TRUE(report.native_called);
  Incomplete(report);
}

}  // namespace
