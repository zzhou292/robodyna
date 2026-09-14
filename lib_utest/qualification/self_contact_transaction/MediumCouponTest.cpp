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
#include <limits>
#include <memory>

namespace {
namespace c = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;

constexpr std::size_t DecisionCount = 262144;
constexpr std::size_t ChunkCapacity = 257;
constexpr std::uint64_t ExpectedPolicyDigest =
    7261953295680066653ull;
constexpr std::size_t GeometryPairCount = 12800;
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

void SetEdgeAxisSeparatedGeometry(
    c::CurrentFixedTriangle* triangle) {
  triangle->vertices[0] = {-0.25, -1, -0.5};
  triangle->vertices[1] = {0.25, -0.5, -0.5};
  triangle->vertices[2] = {1, 1, 0.25};
}

void SetVertexEdgeAxisSeparatedGeometry(
    c::CurrentFixedTriangle* first,
    c::CurrentFixedTriangle* second) {
  first->vertices[0] = {
      -0.7617271175557994, -0.9854918156065067,
      -0.5687134555287552};
  first->vertices[1] = {
      -0.7158959051133422, 0.4952591442811507,
      -0.938635023308946};
  first->vertices[2] = {
      -0.6630144249741934, -0.3375527983865929,
      -0.4822709565245622};
  second->vertices[0] = {
      -0.699936476378623, 0.07336418345847506,
      -1.4577573753456794};
  second->vertices[1] = {
      -0.5297783238983161, -1.0961427534549402,
      -0.9422850565170009};
  second->vertices[2] = {
      -0.7545209025989622, -0.7210003196383497,
      -0.6793528800202036};
}

void SetVertexVertexAxisSeparatedGeometry(
    c::CurrentFixedTriangle* first,
    c::CurrentFixedTriangle* second) {
  first->vertices[0] = {0, 0, 0};
  first->vertices[1] = {
      -0.05620322369963638, 0.016125484251891486,
      0.10471122170959381};
  first->vertices[2] = {
      -0.054441030886089786, -0.15764697103506448,
      -0.16959380631479207};
  second->vertices[0] = {
      0.022402691787493437, 0.020240403023466674,
      0.0057644742154427343};
  second->vertices[1] = {
      0.0016407555274286429, 0.10513512125221264,
      0.17366994826653179};
  second->vertices[2] = {
      0.16934954727284365, 0.12577252251265236,
      0.0056955278101172291};
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

c::FixedContactFacet FacetOf(
    const c::CurrentFixedTriangle& triangle) {
  c::FixedContactFacet result;
  result.source_instance_id = triangle.key.source_instance_id;
  result.source.source_parent_id = triangle.key.parent_eid;
  result.level = triangle.key.level;
  result.local_facet = triangle.key.local_facet;
  for (unsigned local = 0; local < 3; ++local) {
    result.vertex_keys[local] = triangle.vertex_keys[local];
    result.edge_keys[local] = triangle.edge_keys[local];
  }
  return result;
}

void RebuildEdges(c::CurrentFixedTriangle* triangle) {
  for (unsigned edge = 0; edge < 3; ++edge) {
    const auto first = triangle->vertex_keys[edge].first;
    const auto second =
        triangle->vertex_keys[(edge + 1) % 3].first;
    triangle->edge_keys[edge] =
        Edge(first, second, triangle->key.parent_eid);
  }
}

struct GeometryMetrics {
  std::size_t streamed_pairs = 0;
  std::size_t same_rigid = 0;
  std::size_t aabb_separated = 0;
  std::size_t face_axis_separated = 0;
  std::size_t edge_axis_separated = 0;
  std::size_t vertex_edge_axis_separated = 0;
  std::size_t vertex_vertex_axis_separated = 0;
  std::size_t exact_discovery_pairs = 0;
  std::size_t potential_tasks = 0;
  std::size_t local_masked_tasks = 0;
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

enum class GeometryFilter {
  CoordinateOnly,
  FaceAxes,
  EdgeAxes,
  VertexEdgeAxes,
  VertexVertexAxes,
};

struct GeometryStorage {
  std::unique_ptr<c::CurrentFixedTriangle[]> triangles{
      new c::CurrentFixedTriangle[GeometryTriangleCount]};
  std::unique_ptr<c::FixedContactFacet[]> descriptors{
      new c::FixedContactFacet[GeometryTriangleCount]};
  std::array<c::FixedTrianglePair, ChunkCapacity> discovery_pairs;
  std::array<c::FixedTriangleFeatureTaskMask, ChunkCapacity> task_masks;
  std::array<c::RepresentedTrianglePath, 2 * ChunkCapacity> paths;
  std::array<c::RepresentedTrianglePair, ChunkCapacity> crossing_pairs;
};

GeometryMetrics RunGeometryPipeline(
    GeometryStorage& storage, GeometryFilter filter,
    c::FixedTriangleFeatureDiscovery* discovery,
    c::RepresentedIntervalCrossing* crossing,
    bool local_mask = false) {
  const auto started = std::chrono::steady_clock::now();
  GeometryMetrics metrics;
  sct::MotionSupport linear;
  sct::MotionSupport rigid;
  rigid.motion = c::SelfContactFacetMotion::CompleteRigidGroup;
  rigid.complete_rigid_group = 7;

  auto flush = [&](std::size_t count) {
    if (!count) return;
    if (local_mask) {
      const auto built = sct::BuildLocalFeatureTaskMasks(
          storage.descriptors.get(), GeometryTriangleCount,
          storage.discovery_pairs.data(), count,
          storage.task_masks.data(), storage.task_masks.size());
      ASSERT_EQ(built.status, c::SelfContactTransactionStatus::Ok)
          << built.message << " pair=" << built.pair;
    }
    const auto discovery_started = std::chrono::steady_clock::now();
    const auto found = local_mask
        ? discovery->DiscoverMasked(
              storage.triangles.get(), GeometryTriangleCount,
              storage.discovery_pairs.data(), count,
              storage.task_masks.data())
        : discovery->Discover(
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
    metrics.potential_tasks += found.potential_tasks;
    metrics.local_masked_tasks += found.local_masked_tasks;
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
    const auto residue = pair % 20;
    const auto& first_motion = residue < 2 ? rigid : linear;
    const auto& second_motion = residue < 2 ? rigid : linear;
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
    if (filter != GeometryFilter::CoordinateOnly) {
      auto axis_limit = sct::FacetPrismAxisLimit::FaceNormal;
      if (filter == GeometryFilter::EdgeAxes)
        axis_limit = sct::FacetPrismAxisLimit::EdgeCross;
      else if (filter == GeometryFilter::VertexEdgeAxes)
        axis_limit = sct::FacetPrismAxisLimit::VertexEdge;
      else if (filter == GeometryFilter::VertexVertexAxes)
        axis_limit = sct::FacetPrismAxisLimit::VertexVertex;
      bool valid = false;
      sct::FacetPrismSeparationAxis separated_axis =
          sct::FacetPrismSeparationAxis::None;
      if (sct::CertifiedLinearFacetPrismSeparation(
              storage.triangles[first], storage.triangles[first],
              GeometryHalfThickness,
              storage.triangles[second], storage.triangles[second],
              GeometryHalfThickness,
              axis_limit,
              &separated_axis, &valid)) {
        switch (separated_axis) {
          case sct::FacetPrismSeparationAxis::FaceNormal:
            ++metrics.face_axis_separated;
            break;
          case sct::FacetPrismSeparationAxis::EdgeCross:
            ++metrics.edge_axis_separated;
            break;
          case sct::FacetPrismSeparationAxis::VertexEdge:
            ++metrics.vertex_edge_axis_separated;
            break;
          case sct::FacetPrismSeparationAxis::VertexVertex:
            ++metrics.vertex_vertex_axis_separated;
            break;
          case sct::FacetPrismSeparationAxis::None:
            ADD_FAILURE() << "Separated prism has no axis category";
            break;
        }
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

void PrintGeometryMetrics(
    const char* name, const GeometryMetrics& metrics) {
  std::cout << name
            << " streamed_pairs=" << metrics.streamed_pairs
            << " same_rigid=" << metrics.same_rigid
            << " aabb_separated=" << metrics.aabb_separated
            << " face_axis_separated="
            << metrics.face_axis_separated
            << " edge_axis_separated="
            << metrics.edge_axis_separated
            << " vertex_edge_axis_separated="
            << metrics.vertex_edge_axis_separated
            << " vertex_vertex_axis_separated="
            << metrics.vertex_vertex_axis_separated
            << " exact_discovery_pairs="
            << metrics.exact_discovery_pairs
            << " potential_tasks=" << metrics.potential_tasks
            << " local_masked_tasks="
            << metrics.local_masked_tasks
            << " exact_discovery_tasks="
            << metrics.exact_discovery_tasks
            << " feature_events=" << metrics.feature_events
            << " intersection_events=" << metrics.intersection_events
            << " candidate_direct_separation="
            << metrics.candidate_direct_separation
            << " crossing_pairs=" << metrics.crossing_pairs
            << " crossing_work=" << metrics.crossing_work
            << " exact_discovery_us=" << metrics.exact_discovery_us
            << " crossing_us=" << metrics.crossing_us
            << " elapsed_us=" << metrics.elapsed_us << '\n';
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
    const auto residue = pair % 20;
    c::Vec3 shift;
    if (residue >= 2 && residue <= 3)
      shift = {2, 0, 0};
    else if (residue >= 4 && residue <= 11)
      shift = {-0.0625, -0.0625, 0.0625};
    else
      shift = {-0.00390625, -0.00390625, 0.00390625};
    storage.triangles[first + 1] = Triangle(first + 1, shift);
    if (residue >= 12 && residue <= 13)
      SetEdgeAxisSeparatedGeometry(
          storage.triangles.get() + first + 1);
    else if (residue >= 14 && residue <= 15)
      SetVertexEdgeAxisSeparatedGeometry(
          storage.triangles.get() + first,
          storage.triangles.get() + first + 1);
    else if (residue >= 16 && residue <= 17)
      SetVertexVertexAxisSeparatedGeometry(
          storage.triangles.get() + first,
          storage.triangles.get() + first + 1);
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
                sizeof(c::FixedContactFacet) * GeometryTriangleCount +
                sizeof(storage.discovery_pairs) + sizeof(storage.paths) +
                sizeof(storage.crossing_pairs) +
                sizeof(storage.task_masks),
            512u << 20);

  const auto baseline = RunGeometryPipeline(
      storage, GeometryFilter::CoordinateOnly, &discovery, &crossing);
  PrintGeometryMetrics("geometry_baseline", baseline);
  EXPECT_EQ(baseline.streamed_pairs, GeometryPairCount);
  EXPECT_EQ(baseline.same_rigid, GeometryPairCount / 10);
  EXPECT_EQ(baseline.aabb_separated, GeometryPairCount / 10);
  EXPECT_EQ(baseline.exact_discovery_pairs,
            4 * GeometryPairCount / 5);
  EXPECT_EQ(baseline.exact_discovery_tasks,
            15 * baseline.exact_discovery_pairs);
  EXPECT_EQ(baseline.crossing_pairs,
            baseline.exact_discovery_pairs);

  const auto face_axes = RunGeometryPipeline(
      storage, GeometryFilter::FaceAxes, &discovery, &crossing);
  PrintGeometryMetrics("geometry_face_axes", face_axes);
  EXPECT_EQ(face_axes.streamed_pairs, GeometryPairCount);
  EXPECT_EQ(face_axes.same_rigid, GeometryPairCount / 10);
  EXPECT_EQ(face_axes.aabb_separated, GeometryPairCount / 10);
  EXPECT_EQ(face_axes.face_axis_separated,
            2 * GeometryPairCount / 5);
  EXPECT_EQ(face_axes.edge_axis_separated, 0u);
  EXPECT_EQ(face_axes.vertex_edge_axis_separated, 0u);
  EXPECT_EQ(face_axes.vertex_vertex_axis_separated, 0u);
  EXPECT_EQ(face_axes.exact_discovery_pairs,
            2 * GeometryPairCount / 5);
  EXPECT_EQ(face_axes.exact_discovery_tasks,
            15 * face_axes.exact_discovery_pairs);
  EXPECT_EQ(face_axes.feature_events, baseline.feature_events);
  EXPECT_EQ(face_axes.intersection_events,
            baseline.intersection_events);
  EXPECT_EQ(face_axes.candidate_direct_separation,
            face_axes.aabb_separated +
                face_axes.face_axis_separated);
  EXPECT_EQ(face_axes.crossing_pairs,
            face_axes.exact_discovery_pairs);
  EXPECT_EQ(
      face_axes.same_rigid + face_axes.aabb_separated +
          face_axes.face_axis_separated +
          face_axes.exact_discovery_pairs,
      face_axes.streamed_pairs);
  EXPECT_EQ(2 * face_axes.exact_discovery_tasks,
            baseline.exact_discovery_tasks);
  EXPECT_EQ(2 * face_axes.crossing_work,
            baseline.crossing_work);

  const auto edge_axes = RunGeometryPipeline(
      storage, GeometryFilter::EdgeAxes,
      &discovery, &crossing);
  PrintGeometryMetrics("geometry_edge_axes", edge_axes);
  EXPECT_EQ(edge_axes.face_axis_separated,
            2 * GeometryPairCount / 5);
  EXPECT_EQ(edge_axes.edge_axis_separated,
            GeometryPairCount / 10);
  EXPECT_EQ(edge_axes.vertex_edge_axis_separated, 0u);
  EXPECT_EQ(edge_axes.vertex_vertex_axis_separated, 0u);
  EXPECT_EQ(edge_axes.exact_discovery_pairs,
            3 * GeometryPairCount / 10);
  EXPECT_EQ(edge_axes.feature_events, baseline.feature_events);
  EXPECT_EQ(edge_axes.intersection_events,
            baseline.intersection_events);
  EXPECT_EQ(edge_axes.candidate_direct_separation,
            edge_axes.aabb_separated +
                edge_axes.face_axis_separated +
                edge_axes.edge_axis_separated);
  EXPECT_EQ(
      edge_axes.same_rigid + edge_axes.aabb_separated +
          edge_axes.face_axis_separated +
          edge_axes.edge_axis_separated +
          edge_axes.exact_discovery_pairs,
      edge_axes.streamed_pairs);

  const auto vertex_edge_axes = RunGeometryPipeline(
      storage, GeometryFilter::VertexEdgeAxes,
      &discovery, &crossing);
  PrintGeometryMetrics(
      "geometry_vertex_edge_axes", vertex_edge_axes);
  EXPECT_EQ(vertex_edge_axes.face_axis_separated,
            2 * GeometryPairCount / 5);
  EXPECT_EQ(vertex_edge_axes.edge_axis_separated,
            GeometryPairCount / 10);
  EXPECT_EQ(vertex_edge_axes.vertex_edge_axis_separated,
            GeometryPairCount / 10);
  EXPECT_EQ(vertex_edge_axes.vertex_vertex_axis_separated, 0u);
  EXPECT_EQ(vertex_edge_axes.exact_discovery_pairs,
            GeometryPairCount / 5);
  EXPECT_EQ(vertex_edge_axes.feature_events,
            baseline.feature_events);
  EXPECT_EQ(vertex_edge_axes.intersection_events,
            baseline.intersection_events);
  EXPECT_EQ(
      vertex_edge_axes.candidate_direct_separation,
      vertex_edge_axes.aabb_separated +
          vertex_edge_axes.face_axis_separated +
          vertex_edge_axes.edge_axis_separated +
          vertex_edge_axes.vertex_edge_axis_separated);

  const auto vertex_vertex_axes = RunGeometryPipeline(
      storage, GeometryFilter::VertexVertexAxes,
      &discovery, &crossing);
  PrintGeometryMetrics(
      "geometry_vertex_vertex_axes", vertex_vertex_axes);
  EXPECT_EQ(vertex_vertex_axes.face_axis_separated,
            2 * GeometryPairCount / 5);
  EXPECT_EQ(vertex_vertex_axes.edge_axis_separated,
            GeometryPairCount / 10);
  EXPECT_EQ(vertex_vertex_axes.vertex_edge_axis_separated,
            GeometryPairCount / 10);
  EXPECT_EQ(vertex_vertex_axes.vertex_vertex_axis_separated,
            GeometryPairCount / 10);
  EXPECT_EQ(vertex_vertex_axes.exact_discovery_pairs,
            GeometryPairCount / 10);
  EXPECT_EQ(vertex_vertex_axes.feature_events,
            baseline.feature_events);
  EXPECT_EQ(vertex_vertex_axes.intersection_events,
            baseline.intersection_events);
  EXPECT_EQ(
      vertex_vertex_axes.candidate_direct_separation,
      vertex_vertex_axes.aabb_separated +
          vertex_vertex_axes.face_axis_separated +
          vertex_vertex_axes.edge_axis_separated +
          vertex_vertex_axes.vertex_edge_axis_separated +
          vertex_vertex_axes.vertex_vertex_axis_separated);
  EXPECT_EQ(
      vertex_vertex_axes.same_rigid +
          vertex_vertex_axes.candidate_direct_separation +
          vertex_vertex_axes.exact_discovery_pairs,
      vertex_vertex_axes.streamed_pairs);
  EXPECT_EQ(vertex_vertex_axes.crossing_pairs,
            vertex_vertex_axes.exact_discovery_pairs);
  EXPECT_EQ(2 * vertex_vertex_axes.exact_discovery_tasks,
            vertex_edge_axes.exact_discovery_tasks);
  EXPECT_EQ(2 * vertex_vertex_axes.crossing_work,
            vertex_edge_axes.crossing_work);

  // Add only canonical topology after the geometric filter comparison:
  // half of the final exact pairs share one vertex and half share one edge.
  // The unmasked pass is the observation oracle; the local-mask pass must
  // remove only its 6/11 local tasks and preserve every nonlocal contact and
  // exact triangle-intersection observation.
  for (std::size_t pair = 0; pair < GeometryPairCount; ++pair) {
    const auto first = 2 * pair;
    const auto second = first + 1;
    const auto residue = pair % 20;
    if (residue == 18) {
      storage.triangles[second].vertex_keys[0] =
          storage.triangles[first].vertex_keys[0];
      storage.triangles[second].vertices[0] =
          storage.triangles[first].vertices[0];
      RebuildEdges(storage.triangles.get() + second);
    } else if (residue == 19) {
      storage.triangles[second].vertex_keys[0] =
          storage.triangles[first].vertex_keys[0];
      storage.triangles[second].vertex_keys[1] =
          storage.triangles[first].vertex_keys[1];
      storage.triangles[second].vertices[0] =
          storage.triangles[first].vertices[0];
      storage.triangles[second].vertices[1] =
          storage.triangles[first].vertices[1];
      RebuildEdges(storage.triangles.get() + second);
      storage.triangles[first].edge_keys[0] = Edge(
          storage.triangles[first].vertex_keys[0].first,
          storage.triangles[first].vertex_keys[1].first, 0);
      storage.triangles[second].edge_keys[0] =
          storage.triangles[first].edge_keys[0];
    }
  }
  for (std::size_t triangle = 0;
       triangle < GeometryTriangleCount; ++triangle)
    storage.descriptors[triangle] =
        FacetOf(storage.triangles[triangle]);

  const auto local_oracle = RunGeometryPipeline(
      storage, GeometryFilter::VertexVertexAxes,
      &discovery, &crossing);
  const auto local_mask = RunGeometryPipeline(
      storage, GeometryFilter::VertexVertexAxes,
      &discovery, &crossing, true);
  PrintGeometryMetrics("geometry_local_mask", local_mask);
  EXPECT_EQ(local_oracle.exact_discovery_pairs,
            GeometryPairCount / 10);
  EXPECT_EQ(local_oracle.potential_tasks,
            15 * local_oracle.exact_discovery_pairs);
  EXPECT_EQ(local_oracle.local_masked_tasks, 0u);
  EXPECT_EQ(local_mask.potential_tasks,
            local_oracle.potential_tasks);
  EXPECT_EQ(local_mask.local_masked_tasks,
            17 * GeometryPairCount / 20);
  EXPECT_EQ(local_mask.exact_discovery_tasks,
            local_mask.potential_tasks -
                local_mask.local_masked_tasks);
  EXPECT_EQ(local_mask.feature_events,
            local_oracle.feature_events);
  EXPECT_EQ(local_mask.intersection_events,
            local_oracle.intersection_events);
  EXPECT_EQ(local_mask.crossing_work,
            local_oracle.crossing_work);
}

TEST(SelfContactTransactionMediumCoupon,
     PrismCertificateKeepsUnresolvedGeometryInExactTraversal) {
  auto first = Triangle(0, {});
  auto second = Triangle(1, {});
  first.vertices[0] = {0, 0, 0};
  first.vertices[1] = {1, 0, 0};
  first.vertices[2] = {0, 1, 0};
  second.vertices[0] = {0, 0, 0.02};
  second.vertices[1] = {1, 0, 0.02};
  second.vertices[2] = {0, 1, 0.02};

  bool valid = false;
  sct::FacetPrismSeparationAxis axis =
      sct::FacetPrismSeparationAxis::None;
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      first, first, 0.01, second, second, 0.01,
      sct::FacetPrismAxisLimit::VertexVertex,
      &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);

  auto separated = second;
  for (auto& vertex : separated.vertices)
    vertex.z = 0.021;
  EXPECT_TRUE(sct::CertifiedLinearFacetPrismSeparation(
      first, first, 0.01, separated, separated, 0.01,
      sct::FacetPrismAxisLimit::VertexVertex,
      &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::FaceNormal);

  auto crossed = separated;
  for (auto& vertex : crossed.vertices)
    vertex.z = -0.021;
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      first, first, 0.01, separated, crossed, 0.01,
      sct::FacetPrismAxisLimit::VertexVertex,
      &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);

  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      first, first, NAN, separated, separated, 0.01,
      sct::FacetPrismAxisLimit::VertexVertex,
      &axis, &valid));
  EXPECT_FALSE(valid);
}

TEST(SelfContactTransactionMediumCoupon,
     ClosestFeatureAxesAreIncrementalAndSweepConservative) {
  auto vertex_edge_first = Triangle(0, {});
  auto vertex_edge_second = Triangle(1, {});
  SetVertexEdgeAxisSeparatedGeometry(
      &vertex_edge_first, &vertex_edge_second);
  bool valid = false;
  sct::FacetPrismSeparationAxis axis =
      sct::FacetPrismSeparationAxis::None;

  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      vertex_edge_first, vertex_edge_first, GeometryHalfThickness,
      vertex_edge_second, vertex_edge_second, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::EdgeCross, &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);
  EXPECT_TRUE(sct::CertifiedLinearFacetPrismSeparation(
      vertex_edge_first, vertex_edge_first, GeometryHalfThickness,
      vertex_edge_second, vertex_edge_second, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::VertexEdge, &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::VertexEdge);
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      vertex_edge_first, vertex_edge_second, GeometryHalfThickness,
      vertex_edge_second, vertex_edge_second, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::VertexVertex, &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);

  auto vertex_vertex_first = Triangle(2, {});
  auto vertex_vertex_second = Triangle(3, {});
  SetVertexVertexAxisSeparatedGeometry(
      &vertex_vertex_first, &vertex_vertex_second);
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      vertex_vertex_first, vertex_vertex_first, GeometryHalfThickness,
      vertex_vertex_second, vertex_vertex_second, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::VertexEdge, &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);
  EXPECT_TRUE(sct::CertifiedLinearFacetPrismSeparation(
      vertex_vertex_first, vertex_vertex_first, GeometryHalfThickness,
      vertex_vertex_second, vertex_vertex_second, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::VertexVertex, &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::VertexVertex);
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      vertex_vertex_first, vertex_vertex_second, GeometryHalfThickness,
      vertex_vertex_second, vertex_vertex_second, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::VertexVertex, &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);
}

TEST(SelfContactTransactionMediumCoupon,
     EdgeCrossAxisCertifiesOnlyStrictFiniteSeparation) {
  auto first = Triangle(0, {});
  auto second = Triangle(1, {});
  SetEdgeAxisSeparatedGeometry(&second);
  bool valid = false;
  sct::FacetPrismSeparationAxis axis =
      sct::FacetPrismSeparationAxis::None;

  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      first, first, GeometryHalfThickness,
      second, second, GeometryHalfThickness,
      false, &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);
  EXPECT_TRUE(sct::CertifiedLinearFacetPrismSeparation(
      first, first, GeometryHalfThickness,
      second, second, GeometryHalfThickness,
      true, &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::EdgeCross);

  auto crossing = first;
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      first, first, GeometryHalfThickness,
      second, crossing, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::VertexVertex,
      &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);

  auto nearly_parallel = first;
  nearly_parallel.vertices[2].z =
      std::nextafter(nearly_parallel.vertices[2].z, INFINITY);
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      first, first, GeometryHalfThickness,
      nearly_parallel, nearly_parallel, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::VertexVertex,
      &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);

  auto degenerate = first;
  degenerate.vertices[1] = degenerate.vertices[0];
  degenerate.vertices[2] = degenerate.vertices[0];
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      degenerate, degenerate, GeometryHalfThickness,
      degenerate, degenerate, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::VertexVertex,
      &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);

  const double tiny = std::numeric_limits<double>::denorm_min();
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    degenerate.vertices[vertex] = {
        (vertex & 1) ? tiny : 0,
        (vertex & 2) ? tiny : 0,
        tiny};
  }
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      degenerate, degenerate, tiny,
      degenerate, degenerate, tiny,
      sct::FacetPrismAxisLimit::VertexVertex,
      &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);

  const double huge = std::numeric_limits<double>::max();
  degenerate.vertices[0] = {huge, huge, huge};
  degenerate.vertices[1] = {-huge, huge, huge};
  degenerate.vertices[2] = {huge, -huge, huge};
  EXPECT_FALSE(sct::CertifiedLinearFacetPrismSeparation(
      degenerate, degenerate, GeometryHalfThickness,
      degenerate, degenerate, GeometryHalfThickness,
      sct::FacetPrismAxisLimit::VertexVertex,
      &axis, &valid));
  EXPECT_TRUE(valid);
  EXPECT_EQ(axis, sct::FacetPrismSeparationAxis::None);
}

}  // namespace
