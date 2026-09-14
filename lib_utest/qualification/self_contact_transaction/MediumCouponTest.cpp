// SPDX-License-Identifier: MIT
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include "lib_src/collision/RepresentedIntervalCrossing.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>

namespace {
namespace c = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;

constexpr std::size_t DecisionCount = 262144;
constexpr std::size_t ChunkCapacity = 257;
constexpr std::uint64_t ExpectedPolicyDigest =
    7261953295680066653ull;
constexpr std::size_t GeometryPairCount = 10240;
constexpr std::size_t GeometryTriangleCount = 2 * GeometryPairCount;
constexpr double GeometryHalfThickness = 0.01;

c::RepresentedTrianglePathKey Path(std::uint64_t parent) {
  return {17, parent, 0, 0};
}

c::RepresentedIntervalPairKey Pair(std::size_t ordinal) {
  return {{Path(1000 + ordinal),
           Path(1000 + DecisionCount + ordinal)}};
}

c::FacetVertexKey Vertex(std::uint64_t id) {
  c::FacetVertexKey result;
  result.source_instance_id = 17;
  result.first = id;
  result.denominator = 1;
  return result;
}

c::FacetEdgeKey Edge(std::uint64_t first, std::uint64_t second,
                     std::uint64_t parent) {
  c::FacetEdgeKey result;
  result.parent_boundary = parent == 0;
  result.parent_eid = parent;
  result.endpoints[0] = Vertex(first);
  result.endpoints[1] = Vertex(second);
  if (c::fixed_triangle_features::Compare(
          result.endpoints[1], result.endpoints[0]) < 0)
    std::swap(result.endpoints[0], result.endpoints[1]);
  return result;
}

sct::AcceptedEventCertificate VertexFace(
    std::uint32_t first_owner, std::uint32_t second_owner) {
  sct::AcceptedEventCertificate result;
  result.event.feature.vertex_face.vertex = Vertex(100);
  result.event.feature.vertex_face.target.SetFace({17, 20, 0, 0});
  result.event.vertex_use = 2;
  result.event.facet_use = 3;
  result.event.classification.kind = c::SelfContactPairKind::VertexFace;
  result.event.classification.status =
      c::SelfContactPairStatus::AdmittedVertexFace;
  result.event.classification.parent[0] = first_owner;
  result.event.classification.parent[1] = second_owner;
  result.event.classification.active[0] = true;
  result.event.classification.active[1] = true;
  result.event.classification.candidate_directed_area_m2 = {1, 1, 1, 0};
  result.event.classification.admitted_force_area_m2 = {1, 1, 1, 0};
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
    result.event.endpoints[endpoint].count = 3;
    result.event.endpoints[endpoint].nodes[0] = 1;
    result.event.endpoints[endpoint].nodes[1] = 2;
    result.event.endpoints[endpoint].nodes[2] = 3;
    result.event.endpoints[endpoint].weights[0] = 1;
  }
  result.discovery.key = result.event.feature;
  result.discovery.triangles[0] = {17, 10, 0, 0};
  result.discovery.triangles[1] = {17, 20, 0, 0};
  result.discovery.face_weights[0] = 1;
  result.vertex_facet = 2;
  result.target_facet = 3;
  return result;
}

sct::AcceptedEventCertificate EdgeEdge(
    std::uint32_t first_owner, std::uint32_t second_owner) {
  sct::AcceptedEventCertificate result;
  result.kind = sct::AcceptedEventCertificateKind::EdgeEdge;
  result.event.feature.SetEdgeEdge();
  result.event.feature.edge_edge.edges[0] = Edge(1, 2, 10);
  result.event.feature.edge_edge.edges[1] = Edge(3, 4, 20);
  result.event.edge_use[0] = 4;
  result.event.edge_use[1] = 5;
  result.event.classification.kind = c::SelfContactPairKind::EdgeEdge;
  result.event.classification.edge_edge_case =
      c::SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum;
  result.event.classification.status =
      c::SelfContactPairStatus::AdmittedEdgeEdge;
  result.event.classification.parent[0] = first_owner;
  result.event.classification.parent[1] = second_owner;
  result.event.classification.active[0] = true;
  result.event.classification.active[1] = true;
  result.event.classification.candidate_directed_area_m2 = {1, 1, 1, 0};
  result.event.classification.admitted_force_area_m2 = {1, 1, 1, 0};
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
    result.event.endpoints[endpoint].count = 3;
    result.event.endpoints[endpoint].nodes[0] = 3 * endpoint;
    result.event.endpoints[endpoint].nodes[1] = 3 * endpoint + 1;
    result.event.endpoints[endpoint].nodes[2] = 3 * endpoint + 2;
    result.event.endpoints[endpoint].weights[0] = .5;
    result.event.endpoints[endpoint].weights[1] = .5;
  }
  result.discovery.key = result.event.feature;
  result.discovery.triangles[0] = {17, 10, 0, 0};
  result.discovery.triangles[1] = {17, 20, 0, 0};
  result.discovery.edge_parameters[0] = .5;
  result.discovery.edge_parameters[1] = .5;
  result.edge_facet[0] = 1;
  result.edge_facet[1] = 2;
  return result;
}

c::CurrentFixedTriangle Triangle(std::size_t ordinal, c::Vec3 shift) {
  c::CurrentFixedTriangle result;
  result.key = {17, ordinal + 1, 0, 0};
  result.vertices[0] = c::Add({0, 0, 0}, shift);
  result.vertices[1] = c::Add({1, 0, 1}, shift);
  result.vertices[2] = c::Add({0, 1, 1}, shift);
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    result.vertex_keys[vertex] = Vertex(3 * ordinal + vertex + 1);
    result.edge_keys[vertex] = Edge(
        3 * ordinal + vertex + 1,
        3 * ordinal + (vertex + 1) % 3 + 1,
        result.key.parent_eid);
  }
  return result;
}

c::SelfContactSweptParentBounds Bounds(
    const c::CurrentFixedTriangle& triangle) {
  c::SelfContactSweptParentBounds result{
      triangle.vertices[0], triangle.vertices[0]};
  for (unsigned vertex = 1; vertex < 3; ++vertex) {
    result.lower.x = std::min(result.lower.x, triangle.vertices[vertex].x);
    result.lower.y = std::min(result.lower.y, triangle.vertices[vertex].y);
    result.lower.z = std::min(result.lower.z, triangle.vertices[vertex].z);
    result.upper.x = std::max(result.upper.x, triangle.vertices[vertex].x);
    result.upper.y = std::max(result.upper.y, triangle.vertices[vertex].y);
    result.upper.z = std::max(result.upper.z, triangle.vertices[vertex].z);
  }
  result.lower.x = std::nextafter(
      result.lower.x - GeometryHalfThickness, -INFINITY);
  result.lower.y = std::nextafter(
      result.lower.y - GeometryHalfThickness, -INFINITY);
  result.lower.z = std::nextafter(
      result.lower.z - GeometryHalfThickness, -INFINITY);
  result.upper.x = std::nextafter(
      result.upper.x + GeometryHalfThickness, INFINITY);
  result.upper.y = std::nextafter(
      result.upper.y + GeometryHalfThickness, INFINITY);
  result.upper.z = std::nextafter(
      result.upper.z + GeometryHalfThickness, INFINITY);
  return result;
}

void MakeStaticPath(const c::CurrentFixedTriangle& triangle,
                    c::RepresentedTrianglePath* output) {
  output->key = {triangle.key.source_instance_id, triangle.key.parent_eid,
                 triangle.key.level, triangle.key.local_facet};
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    output->vertices[vertex].key = triangle.vertex_keys[vertex];
    output->vertices[vertex].endpoint[0] = triangle.vertices[vertex];
    output->vertices[vertex].endpoint[1] = triangle.vertices[vertex];
    output->edge_keys[vertex] = triangle.edge_keys[vertex];
  }
}

struct GeometryMetrics {
  std::size_t streamed_pairs = 0;
  std::size_t same_rigid = 0;
  std::size_t aabb_separated = 0;
  std::size_t slab_separated = 0;
  std::size_t exact_discovery_pairs = 0;
  std::size_t exact_discovery_tasks = 0;
  std::size_t feature_events = 0;
  std::size_t intersection_events = 0;
  std::size_t candidate_direct_separation = 0;
  std::size_t crossing_pairs = 0;
  std::size_t crossing_work = 0;
  std::uint64_t exact_discovery_us = 0;
  std::uint64_t crossing_us = 0;
  std::uint64_t elapsed_us = 0;
};

struct GeometryStorage {
  std::unique_ptr<c::CurrentFixedTriangle[]> triangles{
      new c::CurrentFixedTriangle[GeometryTriangleCount]};
  std::array<c::FixedTrianglePair, ChunkCapacity> discovery_pairs;
  std::array<c::RepresentedTrianglePath, 2 * ChunkCapacity> paths;
  std::array<c::RepresentedTrianglePair, ChunkCapacity> crossing_pairs;
};

GeometryMetrics RunGeometryPipeline(
    GeometryStorage& storage, bool use_slab_filter,
    c::FixedTriangleFeatureDiscovery* discovery,
    c::RepresentedIntervalCrossing* crossing) {
  const auto started = std::chrono::steady_clock::now();
  GeometryMetrics metrics;
  sct::MotionSupport linear;
  sct::MotionSupport rigid;
  rigid.motion = c::SelfContactFacetMotion::CompleteRigidGroup;
  rigid.complete_rigid_group = 7;

  auto flush = [&](std::size_t count) {
    if (!count) return;
    const auto discovery_started = std::chrono::steady_clock::now();
    const auto found = discovery->Discover(
        storage.triangles.get(), GeometryTriangleCount,
        storage.discovery_pairs.data(), count);
    metrics.exact_discovery_us += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - discovery_started).count());
    ASSERT_EQ(found.status, c::FixedTriangleDiscoveryStatus::Ok)
        << found.message << " pair=" << found.input_pair
        << " task=" << found.input_task << " reason="
        << static_cast<unsigned>(found.arithmetic_reason);
    metrics.exact_discovery_pairs += count;
    metrics.exact_discovery_tasks += found.feature_tasks;
    const auto features = discovery->features();
    ASSERT_TRUE(features.complete);
    for (std::size_t feature = 0; feature < features.count; ++feature)
      metrics.feature_events +=
          features.data[feature].distance_m <=
          2 * GeometryHalfThickness +
              features.data[feature].representation_error_m;
    const auto intersections = discovery->intersections();
    ASSERT_TRUE(intersections.complete);
    metrics.intersection_events += intersections.count;

    for (std::size_t pair = 0; pair < count; ++pair) {
      const auto value = storage.discovery_pairs[pair];
      MakeStaticPath(storage.triangles[value.first],
                     storage.paths.data() + 2 * pair);
      MakeStaticPath(storage.triangles[value.second],
                     storage.paths.data() + 2 * pair + 1);
      storage.crossing_pairs[pair] = {
          static_cast<std::uint32_t>(2 * pair),
          static_cast<std::uint32_t>(2 * pair + 1)};
    }
    const auto crossing_started = std::chrono::steady_clock::now();
    const auto crossed = crossing->Certify(
        storage.paths.data(), 2 * count,
        storage.crossing_pairs.data(), count);
    metrics.crossing_us += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - crossing_started).count());
    ASSERT_EQ(crossed.status, c::RepresentedIntervalStatus::Ok);
    metrics.crossing_pairs += crossed.input_pairs;
    metrics.crossing_work += crossed.work;
  };

  std::size_t pending = 0;
  for (std::size_t pair = 0; pair < GeometryPairCount; ++pair) {
    ++metrics.streamed_pairs;
    const auto first = static_cast<std::uint32_t>(2 * pair);
    const auto second = first + 1;
    const auto residue = pair % 8;
    const auto& first_motion = residue == 0 ? rigid : linear;
    const auto& second_motion = residue == 0 ? rigid : linear;
    const auto action = sct::ClassifyCandidatePairMotion(
        first_motion, Bounds(storage.triangles[first]),
        second_motion, Bounds(storage.triangles[second]));
    if (action == sct::PairMotionAction::ExcludedSameRigidGroup) {
      ++metrics.same_rigid;
      continue;
    }
    if (action == sct::PairMotionAction::CertifiedLinearSeparation) {
      ++metrics.aabb_separated;
      ++metrics.candidate_direct_separation;
      continue;
    }
    EXPECT_EQ(action, sct::PairMotionAction::LinearNodalV1);
    if (use_slab_filter) {
      bool valid = false;
      if (sct::CertifiedSweptFacetSlabSeparation(
              storage.triangles[first], storage.triangles[first],
              GeometryHalfThickness,
              storage.triangles[second], storage.triangles[second],
              GeometryHalfThickness, &valid)) {
        ++metrics.slab_separated;
        ++metrics.candidate_direct_separation;
        continue;
      }
      EXPECT_TRUE(valid);
    }
    storage.discovery_pairs[pending++] = {first, second};
    if (pending == ChunkCapacity) {
      flush(pending);
      pending = 0;
    }
  }
  flush(pending);
  metrics.elapsed_us = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - started).count());
  return metrics;
}

struct FixedStorage {
  std::array<c::SelfContactCandidatePolicyOutcome, ChunkCapacity> outcomes;
  std::array<sct::AcceptedEventCertificate, 8> ledger;
  std::array<std::uint32_t, 17> hash;
  std::array<c::SelfContactForceEvent, 8> events;
};

static_assert(sizeof(FixedStorage) < 512u * 1024u);

TEST(SelfContactTransactionMediumCoupon,
     CompleteDeterministicDecisionStreamUsesFixedStorage) {
  const c::SelfContactSweptParentBounds inflated{
      {0, 0, 0}, {1, 1, 1}};
  const c::SelfContactSweptParentBounds strictly_separated{
      {std::nextafter(1.0, INFINITY), 0, 0}, {2, 1, 1}};
  const c::SelfContactSweptParentBounds touching{
      {1, 0, 0}, {2, 1, 1}};
  sct::MotionSupport linear;
  sct::MotionSupport rigid;
  rigid.motion = c::SelfContactFacetMotion::CompleteRigidGroup;
  rigid.complete_rigid_group = 7;
  rigid.rigid_groups[0] = 7;
  rigid.rigid_group_count = 1;

  FixedStorage storage{};
  c::SelfContactCandidatePolicySummary summary;
  c::RepresentedIntervalPairKey previous;
  bool have_previous = false;
  std::size_t input_count = 0;
  std::size_t chunk_count = 0;
  for (std::size_t begin = 0; begin < DecisionCount;
       begin += ChunkCapacity) {
    const auto count =
        std::min(ChunkCapacity, DecisionCount - begin);
    for (std::size_t local = 0; local < count; ++local) {
      const auto ordinal = begin + local;
      const auto residue = ordinal % 5;
      auto& outcome = storage.outcomes[local];
      outcome = {};
      outcome.pair = Pair(ordinal);
      if (have_previous)
        ASSERT_LT(sct::Compare(previous, outcome.pair), 0);
      previous = outcome.pair;
      have_previous = true;

      sct::PairMotionAction action;
      if (residue == 0)
        action = sct::ClassifyCandidatePairMotion(
            linear, inflated, linear, strictly_separated);
      else if (residue == 1)
        action = sct::ClassifyCandidatePairMotion(
            linear, inflated, linear, touching);
      else if (residue == 2)
        action = sct::ClassifyCandidatePairMotion(
            linear, inflated, linear, inflated);
      else if (residue == 3)
        action = sct::ClassifyCandidatePairMotion(
            rigid, inflated, rigid, inflated);
      else
        action = sct::ClassifyCandidatePairMotion(
            linear, strictly_separated, linear, inflated);

      if (action == sct::PairMotionAction::CertifiedLinearSeparation) {
        outcome.disposition =
            c::SelfContactCandidateDisposition::CertifiedSeparated;
        ++summary.motion_certified_linear_separated;
      } else if (action ==
                 sct::PairMotionAction::ExcludedSameRigidGroup) {
        outcome.disposition =
            c::SelfContactCandidateDisposition::ExcludedSameRigidGroup;
        ++summary.motion_excluded_same_rigid_group;
      } else {
        ASSERT_EQ(action, sct::PairMotionAction::LinearNodalV1);
        outcome.disposition = residue == 1
            ? c::SelfContactCandidateDisposition::
                RepresentedByAcceptedVertexFace
            : c::SelfContactCandidateDisposition::
                RepresentedByAcceptedEdgeEdge;
        outcome.accepted_event = residue == 1 ? 0 : 2;
        outcome.source_order = outcome.accepted_event;
        ++summary.exact_crossing_pairs;
      }
      ++input_count;
    }
    sct::FoldPolicyOutcomes(
        storage.outcomes.data(), count, &summary);
    ++chunk_count;
  }
  summary.complete = true;

  EXPECT_EQ(input_count, DecisionCount);
  EXPECT_EQ(chunk_count,
            (DecisionCount + ChunkCapacity - 1) / ChunkCapacity);
  EXPECT_EQ(summary.outcomes, DecisionCount);
  EXPECT_EQ(summary.motion_certified_linear_separated, 104857u);
  EXPECT_EQ(summary.motion_excluded_same_rigid_group, 52429u);
  EXPECT_EQ(summary.exact_crossing_pairs, 104858u);
  EXPECT_EQ(summary.certified_separated, 104857u);
  EXPECT_EQ(summary.excluded_same_rigid_group, 52429u);
  EXPECT_EQ(summary.represented_by_accepted_vf, 52429u);
  EXPECT_EQ(summary.represented_by_accepted_ee, 52429u);
  EXPECT_EQ(summary.digest, ExpectedPolicyDigest);
  EXPECT_EQ(summary.outcomes,
            summary.motion_certified_linear_separated +
                summary.motion_excluded_same_rigid_group +
                summary.exact_crossing_pairs);
  EXPECT_LT(sizeof(storage), 512u * 1024u);
}

TEST(SelfContactTransactionMediumCoupon,
     OwnerAwareVfAndEeDeduplicateAcrossChunks) {
  FixedStorage storage{};
  storage.hash.fill(UINT32_MAX);
  std::size_t ledger_count = 0;
  const std::array<sct::AcceptedEventCertificate, 2> first_chunk{
      VertexFace(2, 3), EdgeEdge(6, 7)};
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      first_chunk.data(), first_chunk.size(),
      storage.ledger.data(), storage.ledger.size(),
      storage.hash.data(), storage.hash.size(),
      &ledger_count).status, c::SelfContactTransactionStatus::Ok);
  const std::array<sct::AcceptedEventCertificate, 4> second_chunk{
      VertexFace(2, 3), EdgeEdge(6, 7),
      VertexFace(4, 5), EdgeEdge(8, 9)};
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      second_chunk.data(), second_chunk.size(),
      storage.ledger.data(), storage.ledger.size(),
      storage.hash.data(), storage.hash.size(),
      &ledger_count).status, c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(ledger_count, 4u);
  ASSERT_EQ(sct::FinalizeAcceptedEventLedger(
      storage.ledger.data(), ledger_count,
      storage.events.data(), storage.events.size()).status,
      c::SelfContactTransactionStatus::Ok);

  std::size_t vf = 0;
  std::size_t ee = 0;
  for (std::size_t event = 0; event < ledger_count; ++event) {
    EXPECT_EQ(storage.events[event].source_order, event);
    if (event)
      EXPECT_LT(c::CompareSelfContactForceEventIdentity(
                    storage.events[event - 1],
                    storage.events[event]), 0);
    if (storage.events[event].feature.kind ==
        c::FixedTriangleCandidateKind::VertexFace)
      ++vf;
    else
      ++ee;
  }
  EXPECT_EQ(vf, 2u);
  EXPECT_EQ(ee, 2u);
}

TEST(SelfContactTransactionMediumCoupon,
     RealisticGeometryTraversalMeasuresBoundedExactWork) {
  GeometryStorage storage;
  for (std::size_t pair = 0; pair < GeometryPairCount; ++pair) {
    const auto first = 2 * pair;
    storage.triangles[first] = Triangle(first, {});
    c::Vec3 shift;
    if (pair % 8 == 1)
      shift = {2, 0, 0};
    else if (pair % 8 >= 2 && pair % 8 <= 5)
      shift = {-0.0625, -0.0625, 0.0625};
    else
      shift = {-0.00390625, -0.00390625, 0.00390625};
    storage.triangles[first + 1] = Triangle(first + 1, shift);
  }

  c::FixedTriangleFeatureLimits discovery_limits;
  discovery_limits.max_input_pairs = ChunkCapacity;
  discovery_limits.max_triangle_references = 2 * ChunkCapacity;
  discovery_limits.max_vertex_references = 6 * ChunkCapacity;
  discovery_limits.max_edge_references = 6 * ChunkCapacity;
  discovery_limits.max_raw_feature_candidates = 15 * ChunkCapacity;
  discovery_limits.max_feature_candidates = 15 * ChunkCapacity;
  discovery_limits.max_raw_intersections = ChunkCapacity;
  discovery_limits.max_intersections = ChunkCapacity;
  discovery_limits.max_host_bytes = 128u << 20;
  c::FixedTriangleFeatureDiscovery discovery;
  ASSERT_EQ(discovery.Initialize(discovery_limits).status,
            c::FixedTriangleDiscoveryStatus::Ok);

  c::RepresentedIntervalLimits crossing_limits;
  crossing_limits.max_paths = 2 * ChunkCapacity;
  crossing_limits.max_input_pairs = ChunkCapacity;
  crossing_limits.max_results = ChunkCapacity;
  crossing_limits.max_work_per_pair = 255;
  crossing_limits.max_total_work = 255 * ChunkCapacity;
  crossing_limits.max_depth = 12;
  crossing_limits.max_host_bytes = 128u << 20;
  c::RepresentedIntervalCrossing crossing;
  ASSERT_EQ(crossing.Initialize(crossing_limits).status,
            c::RepresentedIntervalStatus::Ok);
  EXPECT_LT(discovery.forecast().owned_host_bytes +
                crossing.forecast().owned_host_bytes +
                sizeof(c::CurrentFixedTriangle) * GeometryTriangleCount +
                sizeof(storage.discovery_pairs) + sizeof(storage.paths) +
                sizeof(storage.crossing_pairs),
            512u << 20);

  const auto baseline = RunGeometryPipeline(
      storage, false, &discovery, &crossing);
  std::cout << "geometry_baseline"
            << " streamed_pairs=" << baseline.streamed_pairs
            << " same_rigid=" << baseline.same_rigid
            << " aabb_separated=" << baseline.aabb_separated
            << " exact_discovery_pairs="
            << baseline.exact_discovery_pairs
            << " exact_discovery_tasks="
            << baseline.exact_discovery_tasks
            << " feature_events=" << baseline.feature_events
            << " intersection_events=" << baseline.intersection_events
            << " candidate_direct_separation="
            << baseline.candidate_direct_separation
            << " crossing_pairs=" << baseline.crossing_pairs
            << " crossing_work=" << baseline.crossing_work
            << " exact_discovery_us=" << baseline.exact_discovery_us
            << " crossing_us=" << baseline.crossing_us
            << " elapsed_us=" << baseline.elapsed_us << '\n';
  EXPECT_EQ(baseline.streamed_pairs, GeometryPairCount);
  EXPECT_EQ(baseline.same_rigid, GeometryPairCount / 8);
  EXPECT_EQ(baseline.aabb_separated, GeometryPairCount / 8);
  EXPECT_EQ(baseline.exact_discovery_pairs,
            6 * GeometryPairCount / 8);
  EXPECT_EQ(baseline.exact_discovery_tasks,
            15 * baseline.exact_discovery_pairs);
  EXPECT_EQ(baseline.crossing_pairs,
            baseline.exact_discovery_pairs);

  const auto optimized = RunGeometryPipeline(
      storage, true, &discovery, &crossing);
  std::cout << "geometry_optimized"
            << " streamed_pairs=" << optimized.streamed_pairs
            << " same_rigid=" << optimized.same_rigid
            << " aabb_separated=" << optimized.aabb_separated
            << " slab_separated=" << optimized.slab_separated
            << " exact_discovery_pairs="
            << optimized.exact_discovery_pairs
            << " exact_discovery_tasks="
            << optimized.exact_discovery_tasks
            << " feature_events=" << optimized.feature_events
            << " intersection_events=" << optimized.intersection_events
            << " candidate_direct_separation="
            << optimized.candidate_direct_separation
            << " crossing_pairs=" << optimized.crossing_pairs
            << " crossing_work=" << optimized.crossing_work
            << " exact_discovery_us=" << optimized.exact_discovery_us
            << " crossing_us=" << optimized.crossing_us
            << " elapsed_us=" << optimized.elapsed_us << '\n';
  EXPECT_EQ(optimized.streamed_pairs, GeometryPairCount);
  EXPECT_EQ(optimized.same_rigid, GeometryPairCount / 8);
  EXPECT_EQ(optimized.aabb_separated, GeometryPairCount / 8);
  EXPECT_EQ(optimized.slab_separated, GeometryPairCount / 2);
  EXPECT_EQ(optimized.exact_discovery_pairs,
            GeometryPairCount / 4);
  EXPECT_EQ(optimized.exact_discovery_tasks,
            15 * optimized.exact_discovery_pairs);
  EXPECT_EQ(optimized.feature_events, baseline.feature_events);
  EXPECT_EQ(optimized.intersection_events,
            baseline.intersection_events);
  EXPECT_EQ(optimized.candidate_direct_separation,
            optimized.aabb_separated + optimized.slab_separated);
  EXPECT_EQ(optimized.crossing_pairs,
            optimized.exact_discovery_pairs);
  EXPECT_EQ(
      optimized.same_rigid + optimized.aabb_separated +
          optimized.slab_separated + optimized.exact_discovery_pairs,
      optimized.streamed_pairs);
  EXPECT_LE(3 * optimized.exact_discovery_tasks,
            baseline.exact_discovery_tasks);
  EXPECT_LE(3 * optimized.crossing_work,
            baseline.crossing_work);
  EXPECT_LT(optimized.elapsed_us,
            3 * baseline.elapsed_us / 4 + 10000);
}

TEST(SelfContactTransactionMediumCoupon,
     SlabCertificateKeepsTouchAndCrossingInExactTraversal) {
  auto first = Triangle(0, {});
  auto second = Triangle(1, {});
  first.vertices[0] = {0, 0, 0};
  first.vertices[1] = {1, 0, 0};
  first.vertices[2] = {0, 1, 0};
  second.vertices[0] = {0, 0, 0.02};
  second.vertices[1] = {1, 0, 0.02};
  second.vertices[2] = {0, 1, 0.02};

  bool valid = false;
  EXPECT_FALSE(sct::CertifiedSweptFacetSlabSeparation(
      first, first, 0.01, second, second, 0.01, &valid));
  EXPECT_TRUE(valid);

  auto separated = second;
  for (auto& vertex : separated.vertices)
    vertex.z = 0.021;
  EXPECT_TRUE(sct::CertifiedSweptFacetSlabSeparation(
      first, first, 0.01, separated, separated, 0.01, &valid));
  EXPECT_TRUE(valid);

  auto crossed = separated;
  for (auto& vertex : crossed.vertices)
    vertex.z = -0.021;
  EXPECT_FALSE(sct::CertifiedSweptFacetSlabSeparation(
      first, first, 0.01, separated, crossed, 0.01, &valid));
  EXPECT_TRUE(valid);

  EXPECT_FALSE(sct::CertifiedSweptFacetSlabSeparation(
      first, first, NAN, separated, separated, 0.01, &valid));
  EXPECT_FALSE(valid);
}

}  // namespace
