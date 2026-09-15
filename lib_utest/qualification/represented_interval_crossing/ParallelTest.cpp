// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <array>

namespace represented_interval_test {
namespace {

struct Observation {
  std::array<unsigned char, sizeof(ct::RepresentedIntervalReport)> report;
  std::vector<unsigned char> results;
  bool complete = false;

  bool operator==(const Observation& other) const {
    return report == other.report && results == other.results &&
           complete == other.complete;
  }
};

Observation Observe(const ct::RepresentedIntervalReport& report,
                    const ct::RepresentedIntervalCrossing& crossing) {
  Observation result;
  std::memcpy(result.report.data(), &report, sizeof(report));
  const auto view = crossing.results();
  result.complete = view.complete;
  result.results.resize(view.count * sizeof(*view.data));
  if (!result.results.empty())
    std::memcpy(result.results.data(), view.data, result.results.size());
  return result;
}

ct::RepresentedIntervalLimits WorkerLimits(
    std::size_t paths, std::size_t pairs, unsigned workers) {
  ct::RepresentedIntervalLimits limits;
  limits.max_paths = paths;
  limits.max_input_pairs = pairs;
  limits.max_results = pairs;
  limits.max_work_per_pair = 64;
  limits.max_total_work = 64 * pairs;
  limits.max_depth = 12;
  limits.max_host_bytes = 64u << 20;
  limits.worker_count = workers;
  return limits;
}

Observation Evaluate(
    unsigned workers,
    const std::vector<ct::RepresentedTrianglePath>& paths,
    const std::vector<ct::RepresentedTrianglePair>& pairs,
    ct::RepresentedIntervalLimits limits = {}) {
  limits.max_paths = std::max(limits.max_paths, paths.size());
  limits.max_input_pairs = std::max(limits.max_input_pairs, pairs.size());
  limits.max_results = std::max(limits.max_results, pairs.size());
  limits.worker_count = workers;
  ct::RepresentedIntervalCrossing crossing;
  EXPECT_EQ(crossing.Initialize(limits).status, S::Ok);
  const auto report = crossing.Certify(
      paths.data(), paths.size(), pairs.data(), pairs.size());
  return Observe(report, crossing);
}

std::array<ct::Vec3, 3> DiagonalSeparated(double shift = 0) {
  return {{{1.5 + shift, 1.5, 0},
           {3.5 + shift, 1.5, 0},
           {1.5 + shift, 3.5, 0}}};
}

}  // namespace

TEST(RepresentedIntervalParallel,
     WorkerCountsMatchMixedCertificatesAndVariedPairOrder) {
  const std::array<ct::Vec3, 3> line{
      ct::Vec3{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
  auto arc = Static(60, BaseTriangle(4), 600);
  arc.motion = ct::RepresentedMotion::RigidArc;
  const std::vector<ct::RepresentedTrianglePath> paths{
      Static(10, BaseTriangle(), 100),
      Static(20, BaseTriangle(2), 200),
      Path(30, BaseTriangle(1), BaseTriangle(-1), 300),
      Path(40, BaseTriangle(1), BaseTriangle(-2), 400),
      Static(50, line, 500), arc};
  const std::vector<ct::RepresentedTrianglePair> pairs{
      {1, 0}, {0, 2}, {3, 0}, {0, 4}, {5, 0},
      {2, 0}, {0, 1}};
  auto limits = WorkerLimits(paths.size(), pairs.size(), 1);
  limits.max_work_per_pair = 32;
  limits.max_total_work = 32 * pairs.size();

  const auto serial = Evaluate(1, paths, pairs, limits);
  EXPECT_EQ(serial, Evaluate(2, paths, pairs, limits));
  EXPECT_EQ(serial, Evaluate(4, paths, pairs, limits));

  auto reversed = pairs;
  std::reverse(reversed.begin(), reversed.end());
  const auto permuted = Evaluate(4, paths, reversed, limits);
  EXPECT_EQ(permuted.results, serial.results);
  EXPECT_TRUE(permuted.complete);
  for (unsigned repetition = 0; repetition < 32; ++repetition) {
    auto varied = pairs;
    const auto shift = repetition % varied.size();
    std::rotate(varied.begin(), varied.begin() + shift, varied.end());
    if (repetition & 1)
      std::reverse(varied.begin(), varied.end());
    const unsigned workers[]{1, 2, 4};
    const auto observation = Evaluate(
        workers[repetition % 3], paths, varied, limits);
    EXPECT_EQ(observation.results, serial.results);
    EXPECT_TRUE(observation.complete);
  }
}

TEST(RepresentedIntervalParallel,
     CanonicalTotalWorkFailureMatchesSerialAndRollsBackExactly) {
  ct::RepresentedIntervalLimits limits;
  limits.max_paths = 3;
  limits.max_input_pairs = 2;
  limits.max_results = 2;
  limits.max_work_per_pair = 3;
  limits.max_total_work = 5;
  limits.max_depth = 20;
  limits.max_host_bytes = 64u << 20;
  const std::vector<ct::RepresentedTrianglePath> expensive{
      Static(10, BaseTriangle()),
      Path(20, DiagonalSeparated(), DiagonalSeparated(.125)),
      Path(30, DiagonalSeparated(.125), DiagonalSeparated(.25))};
  const std::vector<ct::RepresentedTrianglePair> pairs{
      {0, 1}, {0, 2}};

  const auto serial = Evaluate(1, expensive, pairs, limits);
  EXPECT_EQ(serial, Evaluate(2, expensive, pairs, limits));
  EXPECT_EQ(serial, Evaluate(4, expensive, pairs, limits));
  ct::RepresentedIntervalReport serial_report;
  std::memcpy(&serial_report, serial.report.data(), sizeof(serial_report));
  EXPECT_EQ(serial_report.status, S::ResourceLimit);
  EXPECT_EQ(serial_report.input_pair, 1u);
  EXPECT_EQ(serial_report.work, 3u);

  for (unsigned workers : {1u, 2u, 4u}) {
    SCOPED_TRACE(workers);
    limits.worker_count = workers;
    ct::RepresentedIntervalCrossing crossing;
    ASSERT_EQ(crossing.Initialize(limits).status, S::Ok);
    const std::vector<ct::RepresentedTrianglePath> initial{
        Static(10, BaseTriangle()), Static(20, BaseTriangle(1))};
    const ct::RepresentedTrianglePair initial_pair{0, 1};
    ASSERT_EQ(crossing.Certify(
                  initial.data(), initial.size(), &initial_pair, 1).status,
              S::Ok);
    const auto before_view = crossing.results();
    const auto before = Bytes(before_view.data, before_view.count);
    const auto failed = crossing.Certify(
        expensive.data(), expensive.size(), pairs.data(), pairs.size());
    EXPECT_EQ(failed.status, S::ResourceLimit);
    EXPECT_EQ(failed.input_pair, 1u);
    const auto after = crossing.results();
    EXPECT_EQ(after.data, before_view.data);
    EXPECT_EQ(Bytes(after.data, after.count), before);
  }
}

TEST(RepresentedIntervalParallel,
     InvalidInputReportsAndPriorPublicationMatchExactly) {
  std::vector<ct::RepresentedTrianglePath> paths{
      Static(10, BaseTriangle()), Static(20, BaseTriangle(1))};
  paths[1].vertices[2].endpoint[1].z =
      std::numeric_limits<double>::quiet_NaN();
  const std::vector<ct::RepresentedTrianglePair> pairs{{0, 1}};
  const auto limits = WorkerLimits(paths.size(), pairs.size(), 1);
  const auto serial = Evaluate(1, paths, pairs, limits);
  EXPECT_EQ(serial, Evaluate(2, paths, pairs, limits));
  EXPECT_EQ(serial, Evaluate(4, paths, pairs, limits));
}

TEST(RepresentedIntervalParallel,
     PreflightBoundsPersistentWorkerStorageAndDestruction) {
  auto one = WorkerLimits(8, 8, 1);
  auto four = WorkerLimits(8, 8, 4);
  const auto one_plan = ct::RepresentedIntervalCrossing::Preflight(one);
  const auto four_plan = ct::RepresentedIntervalCrossing::Preflight(four);
  ASSERT_EQ(one_plan.report.status, S::Ok);
  ASSERT_EQ(four_plan.report.status, S::Ok);
  EXPECT_EQ(four_plan.forecast.worker_count, 4u);
  EXPECT_EQ(four_plan.forecast.pair_status_capacity, 8u);
  EXPECT_GT(four_plan.forecast.pair_status_bytes, 0u);
  EXPECT_GT(four_plan.forecast.worker_metadata_bytes,
            one_plan.forecast.worker_metadata_bytes);
  EXPECT_GT(four_plan.forecast.dfs_frame_bytes,
            one_plan.forecast.dfs_frame_bytes);
  EXPECT_GT(four_plan.forecast.exact_scratch_bytes,
            one_plan.forecast.exact_scratch_bytes);
  EXPECT_GT(four_plan.forecast.worker_stack_bytes,
            one_plan.forecast.worker_stack_bytes);
  EXPECT_EQ(four_plan.forecast.startup_host_bytes,
            four_plan.forecast.owned_host_bytes);
  EXPECT_LT(four_plan.forecast.owned_host_bytes, 512u << 20);

  four.max_host_bytes = four_plan.forecast.owned_host_bytes - 1;
  EXPECT_EQ(ct::RepresentedIntervalCrossing::Preflight(four).report.status,
            S::ResourceLimit);
  four = WorkerLimits(8, 8, 0);
  EXPECT_EQ(ct::RepresentedIntervalCrossing::Preflight(four).report.status,
            S::InvalidInput);
  four = WorkerLimits(8, 8, 8);
  EXPECT_EQ(ct::RepresentedIntervalCrossing::Preflight(four).report.status,
            S::Ok);
  four.worker_count = 9;
  EXPECT_EQ(ct::RepresentedIntervalCrossing::Preflight(four).report.status,
            S::InvalidInput);

  const std::vector<ct::RepresentedTrianglePath> paths{
      Static(10, BaseTriangle()), Static(20, BaseTriangle(1))};
  const ct::RepresentedTrianglePair pair{0, 1};
  for (unsigned iteration = 0; iteration < 12; ++iteration) {
    auto limits = WorkerLimits(2, 1, 1 + iteration % 4);
    ct::RepresentedIntervalCrossing crossing;
    ASSERT_EQ(crossing.Initialize(limits).status, S::Ok);
    const auto first = crossing.Certify(
        paths.data(), paths.size(), &pair, 1);
    const auto first_bytes = Bytes(
        crossing.results().data, crossing.results().count);
    const auto second = crossing.Certify(
        paths.data(), paths.size(), &pair, 1);
    EXPECT_EQ(std::memcmp(&first, &second, sizeof(first)), 0);
    EXPECT_EQ(first_bytes, Bytes(
        crossing.results().data, crossing.results().count));
  }
}

}  // namespace represented_interval_test
