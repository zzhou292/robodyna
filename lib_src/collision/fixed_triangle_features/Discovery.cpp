// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../FixedTriangleFeatureDiscovery.h"

#include "Geometry.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <new>

namespace tlfea::contact {
namespace ft = fixed_triangle_features;
namespace {

bool CheckedBytes(std::size_t count, std::size_t width,
                  std::size_t* total) noexcept {
  if (count && width > (std::numeric_limits<std::size_t>::max() - *total) /
                           count)
    return false;
  *total += count * width;
  return true;
}

bool CheckedProduct(std::size_t count, std::size_t width,
                    std::size_t* result) noexcept {
  if (count && width > std::numeric_limits<std::size_t>::max() / count)
    return false;
  *result = count * width;
  return true;
}

bool Disjoint(const void* a, std::size_t a_bytes, const void* b,
              std::size_t b_bytes) noexcept {
  if (!a || !b)
    return false;
  const auto x = reinterpret_cast<std::uintptr_t>(a);
  const auto y = reinterpret_cast<std::uintptr_t>(b);
  return a_bytes <= UINTPTR_MAX - x && b_bytes <= UINTPTR_MAX - y &&
         (x + a_bytes <= y || y + b_bytes <= x);
}

bool Same(Vec3 a, Vec3 b) noexcept {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

struct TriangleLedgerEntry {
  CurrentFixedTriangle value;
};

struct VertexLedgerEntry {
  FacetVertexKey key;
  Vec3 value;
};

struct EdgeLedgerEntry {
  FacetEdgeKey key;
  Vec3 endpoints[2];
};

struct PairIntersectionStage {
  FixedTriangleIntersection value;
  bool intersects = false;
};

bool TriangleLedgerLess(const TriangleLedgerEntry& a,
                        const TriangleLedgerEntry& b) noexcept {
  return ft::Compare(a.value.key, b.value.key) < 0;
}

bool VertexLedgerLess(const VertexLedgerEntry& a,
                      const VertexLedgerEntry& b) noexcept {
  return ft::Compare(a.key, b.key) < 0;
}

bool EdgeLedgerLess(const EdgeLedgerEntry& a,
                    const EdgeLedgerEntry& b) noexcept {
  return ft::Compare(a.key, b.key) < 0;
}

bool SameCandidateValue(const FixedTriangleFeatureCandidate& a,
                        const FixedTriangleFeatureCandidate& b) noexcept {
  if (!ft::SameFeatureKey(a, b) || a.distance_m != b.distance_m ||
      a.representation_error_m != b.representation_error_m)
    return false;
  for (unsigned i = 0; i < 2; ++i) {
    if (!Same(a.points[i], b.points[i]) ||
        a.edge_parameters[i] != b.edge_parameters[i])
      return false;
  }
  if (a.key.kind == FixedTriangleCandidateKind::VertexFace) {
    const unsigned target_a = a.local_features[0] == 3 ? 0 : 1;
    const unsigned target_b = b.local_features[0] == 3 ? 0 : 1;
    if (ft::Compare(a.triangles[target_a], b.triangles[target_b]) == 0)
      for (unsigned i = 0; i < 3; ++i)
        if (a.face_weights[i] != b.face_weights[i])
          return false;
  }
  return true;
}

Vec3 EdgeEndpoint(const CurrentFixedTriangle& triangle,
                  const FacetVertexKey& key) noexcept {
  for (unsigned i = 0; i < 3; ++i)
    if (ft::Compare(triangle.vertex_keys[i], key) == 0)
      return triangle.vertices[i];
  return {};
}

const char* Message(FixedTriangleDiscoveryStatus status) noexcept {
  switch (status) {
    case FixedTriangleDiscoveryStatus::Ok:
      return "OK";
    case FixedTriangleDiscoveryStatus::AlreadyInitialized:
      return "Feature discovery is already initialized";
    case FixedTriangleDiscoveryStatus::NotInitialized:
      return "Feature discovery is not initialized";
    case FixedTriangleDiscoveryStatus::InvalidInput:
      return "Invalid fixed-triangle geometry or topology";
    case FixedTriangleDiscoveryStatus::OutOfRange:
      return "Triangle pair index is out of range";
    case FixedTriangleDiscoveryStatus::DegenerateTriangle:
      return "Consumed current triangle is degenerate";
    case FixedTriangleDiscoveryStatus::NonFiniteResult:
      return "Fixed-triangle arithmetic is not representable";
    case FixedTriangleDiscoveryStatus::ResourceLimit:
      return "Complete fixed-triangle candidate inventory exceeds capacity";
    case FixedTriangleDiscoveryStatus::IdentityMismatch:
      return "Repeated immutable feature identity disagrees";
  }
  return "Unknown fixed-triangle discovery status";
}

}  // namespace

struct FixedTriangleFeatureDiscovery::Impl {
  FixedTriangleFeatureLimits limits;
  FixedTriangleFeatureForecast forecast;
  std::unique_ptr<TriangleLedgerEntry[]> triangle_ledger;
  std::unique_ptr<VertexLedgerEntry[]> vertex_ledger;
  std::unique_ptr<EdgeLedgerEntry[]> edge_ledger;
  std::unique_ptr<FixedTriangleFeatureCandidate[]> raw_features;
  std::unique_ptr<FixedTriangleFeatureCandidate[]> features;
  std::unique_ptr<PairIntersectionStage[]> pair_intersections;
  std::unique_ptr<FixedTriangleIntersection[]> raw_intersections;
  std::unique_ptr<FixedTriangleIntersection[]> intersections;
  std::size_t feature_count = 0;
  std::size_t intersection_count = 0;
  bool complete = false;

  bool InputDisjoint(const void* input,
                     std::size_t bytes) const noexcept {
    if (!input || !bytes)
      return false;
    const auto separated = [input, bytes](const auto& storage,
                                          std::size_t count) {
      return !count ||
             Disjoint(input, bytes, storage.get(),
                      count * sizeof(*storage.get()));
    };
    return Disjoint(input, bytes, this, sizeof(*this)) &&
           separated(triangle_ledger, limits.max_triangle_references) &&
           separated(vertex_ledger, limits.max_vertex_references) &&
           separated(edge_ledger, limits.max_edge_references) &&
           separated(raw_features, limits.max_raw_feature_candidates) &&
           separated(features, limits.max_feature_candidates) &&
           separated(pair_intersections, limits.max_input_pairs) &&
           separated(raw_intersections, limits.max_raw_intersections) &&
           separated(intersections, limits.max_intersections);
  }

  void PublishEmpty() noexcept {
    feature_count = 0;
    intersection_count = 0;
    complete = true;
  }
};

FixedTriangleFeatureDiscovery::FixedTriangleFeatureDiscovery() noexcept =
    default;
FixedTriangleFeatureDiscovery::~FixedTriangleFeatureDiscovery() = default;

FixedTriangleFeaturePreflight FixedTriangleFeatureDiscovery::Preflight(
    FixedTriangleFeatureLimits limits) noexcept {
  FixedTriangleFeaturePreflight result;
  result.forecast.triangle_ledger_capacity =
      limits.max_triangle_references;
  result.forecast.vertex_ledger_capacity =
      limits.max_vertex_references;
  result.forecast.edge_ledger_capacity =
      limits.max_edge_references;
  result.forecast.raw_feature_capacity = limits.max_raw_feature_candidates;
  result.forecast.feature_publication_capacity =
      limits.max_feature_candidates;
  result.forecast.pair_intersection_capacity = limits.max_input_pairs;
  result.forecast.raw_intersection_capacity = limits.max_raw_intersections;
  result.forecast.intersection_publication_capacity =
      limits.max_intersections;
  std::size_t bytes = sizeof(Impl);
  std::size_t maximum_tasks = 0;
  std::size_t maximum_triangle_references = 0;
  const bool pair_product_ok =
      CheckedProduct(limits.max_input_pairs, 15, &maximum_tasks) &&
      CheckedProduct(limits.max_input_pairs, 2,
                     &maximum_triangle_references);
  const bool arithmetic_ok =
      pair_product_ok &&
      CheckedBytes(limits.max_triangle_references,
                   sizeof(TriangleLedgerEntry), &bytes) &&
      CheckedBytes(limits.max_vertex_references,
                   sizeof(VertexLedgerEntry), &bytes) &&
      CheckedBytes(limits.max_edge_references,
                   sizeof(EdgeLedgerEntry), &bytes) &&
      CheckedBytes(limits.max_raw_feature_candidates,
                   sizeof(FixedTriangleFeatureCandidate), &bytes) &&
      CheckedBytes(limits.max_feature_candidates,
                   sizeof(FixedTriangleFeatureCandidate), &bytes) &&
      CheckedBytes(limits.max_input_pairs,
                   sizeof(PairIntersectionStage), &bytes) &&
      CheckedBytes(limits.max_raw_intersections,
                   sizeof(FixedTriangleIntersection), &bytes) &&
      CheckedBytes(limits.max_intersections,
                   sizeof(FixedTriangleIntersection), &bytes);
  result.forecast.owned_host_bytes = arithmetic_ok ? bytes : SIZE_MAX;
  if (!limits.max_input_pairs || !limits.max_triangle_references ||
      !limits.max_vertex_references || !limits.max_edge_references ||
      !limits.max_raw_feature_candidates || !limits.max_raw_intersections ||
      !pair_product_ok ||
      limits.max_raw_feature_candidates >
          maximum_tasks ||
      limits.max_raw_intersections > limits.max_input_pairs ||
      limits.max_feature_candidates >
          limits.max_raw_feature_candidates ||
      limits.max_intersections > limits.max_raw_intersections) {
    result.report.status = FixedTriangleDiscoveryStatus::InvalidInput;
  } else if (!arithmetic_ok || bytes > limits.max_host_bytes) {
    result.report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
  }
  result.report.message = Message(result.report.status);
  return result;
}

FixedTriangleDiscoveryReport FixedTriangleFeatureDiscovery::Initialize(
    FixedTriangleFeatureLimits limits) noexcept {
  if (impl_) {
    FixedTriangleDiscoveryReport report;
    report.status = FixedTriangleDiscoveryStatus::AlreadyInitialized;
    report.message = Message(report.status);
    return report;
  }
  const auto preflight = Preflight(limits);
  if (preflight.report.status != FixedTriangleDiscoveryStatus::Ok)
    return preflight.report;
  auto next = std::unique_ptr<Impl>(new (std::nothrow) Impl);
  if (!next) {
    FixedTriangleDiscoveryReport report;
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Feature discovery control allocation failed";
    return report;
  }
  next->limits = limits;
  next->forecast = preflight.forecast;
  next->triangle_ledger.reset(new (std::nothrow)
      TriangleLedgerEntry[limits.max_triangle_references]);
  next->vertex_ledger.reset(new (std::nothrow)
      VertexLedgerEntry[limits.max_vertex_references]);
  next->edge_ledger.reset(new (std::nothrow)
      EdgeLedgerEntry[limits.max_edge_references]);
  next->raw_features.reset(new (std::nothrow)
      FixedTriangleFeatureCandidate[limits.max_raw_feature_candidates]);
  if (limits.max_feature_candidates)
    next->features.reset(new (std::nothrow)
        FixedTriangleFeatureCandidate[limits.max_feature_candidates]);
  next->pair_intersections.reset(new (std::nothrow)
      PairIntersectionStage[limits.max_input_pairs]);
  next->raw_intersections.reset(new (std::nothrow)
      FixedTriangleIntersection[limits.max_raw_intersections]);
  if (limits.max_intersections)
    next->intersections.reset(new (std::nothrow)
        FixedTriangleIntersection[limits.max_intersections]);
  if (!next->triangle_ledger || !next->vertex_ledger ||
      !next->edge_ledger || !next->raw_features ||
      (limits.max_feature_candidates && !next->features) ||
      !next->pair_intersections || !next->raw_intersections ||
      (limits.max_intersections && !next->intersections)) {
    FixedTriangleDiscoveryReport report;
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Feature discovery bounded arena allocation failed";
    return report;
  }
  impl_ = std::move(next);
  return {};
}

FixedTriangleDiscoveryReport FixedTriangleFeatureDiscovery::Discover(
    const CurrentFixedTriangle* triangles, std::size_t triangle_count,
    const FixedTrianglePair* pairs, std::size_t pair_count) noexcept {
  FixedTriangleDiscoveryReport report;
  if (!impl_) {
    report.status = FixedTriangleDiscoveryStatus::NotInitialized;
    report.message = Message(report.status);
    return report;
  }
  // All previously borrowed views expire at this entry.  Staging remains
  // separate, so every failure below preserves the last complete publication.
  if (pair_count == 0) {
    impl_->PublishEmpty();
    return report;
  }
  if (!triangles || !triangle_count || !pairs) {
    report.status = FixedTriangleDiscoveryStatus::InvalidInput;
    report.message = Message(report.status);
    return report;
  }
  std::size_t triangle_bytes = 0;
  std::size_t pair_bytes = 0;
  if (!CheckedProduct(triangle_count, sizeof(*triangles),
                      &triangle_bytes) ||
      !CheckedProduct(pair_count, sizeof(*pairs), &pair_bytes) ||
      !impl_->InputDisjoint(triangles, triangle_bytes) ||
      !impl_->InputDisjoint(pairs, pair_bytes) ||
      !Disjoint(triangles, triangle_bytes, this, sizeof(*this)) ||
      !Disjoint(pairs, pair_bytes, this, sizeof(*this))) {
    report.status = FixedTriangleDiscoveryStatus::InvalidInput;
    report.message = "Fixed-triangle input range aliases owned storage";
    return report;
  }
  if (pair_count > impl_->limits.max_input_pairs) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }
  if (!CheckedProduct(pair_count, 2, &report.triangle_references)) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Fixed-triangle reference count overflow";
    return report;
  }
  if (report.triangle_references >
      impl_->limits.max_triangle_references) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Fixed-triangle ledger capacity exceeded";
    return report;
  }

  std::size_t triangle_write = 0;
  for (std::size_t i = 0; i < pair_count; ++i) {
    if (pairs[i].first >= triangle_count ||
        pairs[i].second >= triangle_count) {
      report.status = FixedTriangleDiscoveryStatus::OutOfRange;
      report.input_pair = i;
      report.message = Message(report.status);
      return report;
    }
    impl_->triangle_ledger[triangle_write++].value =
        triangles[pairs[i].first];
    impl_->triangle_ledger[triangle_write++].value =
        triangles[pairs[i].second];
  }
  std::sort(impl_->triangle_ledger.get(),
            impl_->triangle_ledger.get() + triangle_write,
            TriangleLedgerLess);
  std::size_t unique_triangles = 0;
  for (std::size_t i = 0; i < triangle_write; ++i) {
    if (unique_triangles &&
        ft::Compare(impl_->triangle_ledger[unique_triangles - 1].value.key,
                    impl_->triangle_ledger[i].value.key) == 0) {
      if (!ft::SameTriangleValue(
              impl_->triangle_ledger[unique_triangles - 1].value,
              impl_->triangle_ledger[i].value)) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    const auto status =
        ft::ValidateTriangle(impl_->triangle_ledger[i].value);
    if (status != FixedTriangleDiscoveryStatus::Ok) {
      report.status = status;
      report.arithmetic_reason =
          FixedTriangleArithmeticReason::TriangleValidation;
      report.message = Message(status);
      return report;
    }
    if (unique_triangles != i)
      impl_->triangle_ledger[unique_triangles] =
          impl_->triangle_ledger[i];
    ++unique_triangles;
  }
  report.triangles = unique_triangles;
  if (!CheckedProduct(unique_triangles, 3,
                      &report.vertex_references) ||
      !CheckedProduct(unique_triangles, 3,
                      &report.edge_references)) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Fixed feature ledger count overflow";
    return report;
  }
  if (report.vertex_references >
          impl_->limits.max_vertex_references ||
      report.edge_references > impl_->limits.max_edge_references) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Fixed feature ledger capacity exceeded";
    return report;
  }

  std::size_t vertex_write = 0;
  std::size_t edge_write = 0;
  for (std::size_t i = 0; i < unique_triangles; ++i) {
    const auto& triangle = impl_->triangle_ledger[i].value;
    for (unsigned local = 0; local < 3; ++local) {
      impl_->vertex_ledger[vertex_write++] =
          {triangle.vertex_keys[local], triangle.vertices[local]};
      impl_->edge_ledger[edge_write++] = {
          triangle.edge_keys[local],
          {EdgeEndpoint(triangle,
                        triangle.edge_keys[local].endpoints[0]),
           EdgeEndpoint(triangle,
                        triangle.edge_keys[local].endpoints[1])}};
    }
  }
  std::sort(impl_->vertex_ledger.get(),
            impl_->vertex_ledger.get() + vertex_write,
            VertexLedgerLess);
  std::size_t unique_vertices = 0;
  for (std::size_t i = 0; i < vertex_write; ++i) {
    if (unique_vertices &&
        ft::Compare(impl_->vertex_ledger[unique_vertices - 1].key,
                    impl_->vertex_ledger[i].key) == 0) {
      if (!Same(impl_->vertex_ledger[unique_vertices - 1].value,
                impl_->vertex_ledger[i].value)) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_vertices != i)
      impl_->vertex_ledger[unique_vertices] =
          impl_->vertex_ledger[i];
    ++unique_vertices;
  }
  report.vertices = unique_vertices;

  std::sort(impl_->edge_ledger.get(),
            impl_->edge_ledger.get() + edge_write, EdgeLedgerLess);
  std::size_t unique_edges = 0;
  for (std::size_t i = 0; i < edge_write; ++i) {
    if (unique_edges &&
        ft::Compare(impl_->edge_ledger[unique_edges - 1].key,
                    impl_->edge_ledger[i].key) == 0) {
      const auto& old = impl_->edge_ledger[unique_edges - 1];
      const auto& next = impl_->edge_ledger[i];
      if (!Same(old.endpoints[0], next.endpoints[0]) ||
          !Same(old.endpoints[1], next.endpoints[1])) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_edges != i)
      impl_->edge_ledger[unique_edges] = impl_->edge_ledger[i];
    ++unique_edges;
  }
  report.edges = unique_edges;

  for (std::size_t i = 0; i < pair_count; ++i) {
    const std::size_t count = ft::CountPairFeatureCandidates(
        triangles[pairs[i].first], triangles[pairs[i].second]);
    if (count > std::numeric_limits<std::size_t>::max() -
                    report.raw_feature_candidates) {
      report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
      report.input_pair = i;
      report.message = "Fixed-triangle feature count overflow";
      return report;
    }
    report.raw_feature_candidates += count;
  }
  if (report.raw_feature_candidates >
      impl_->limits.max_raw_feature_candidates) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }

  std::size_t feature_write = 0;
  for (std::size_t i = 0; i < pair_count; ++i) {
    auto& intersection = impl_->pair_intersections[i];
    auto status = ft::ClassifyPairIntersection(
        triangles[pairs[i].first], triangles[pairs[i].second],
        &intersection.value, &intersection.intersects);
    if (status != FixedTriangleDiscoveryStatus::Ok) {
      report.status = status;
      report.input_pair = i;
      report.arithmetic_reason =
          FixedTriangleArithmeticReason::IntersectionPredicate;
      report.message = Message(status);
      return report;
    }
    ft::PairFeatureResult pair;
    status = ft::EvaluatePairFeaturesOnce(
        triangles[pairs[i].first], triangles[pairs[i].second],
        impl_->raw_features.get() + feature_write,
        report.raw_feature_candidates - feature_write, &pair);
    report.feature_tasks += pair.feature_tasks;
    if (status != FixedTriangleDiscoveryStatus::Ok) {
      report.status = status;
      report.input_pair = i;
      report.input_task =
          pair.feature_tasks ? pair.feature_tasks - 1 : SIZE_MAX;
      report.arithmetic_reason = pair.arithmetic_reason;
      report.message = Message(status);
      return report;
    }
    const std::size_t expected = ft::CountPairFeatureCandidates(
        triangles[pairs[i].first], triangles[pairs[i].second]);
    if (pair.feature_tasks != 15 || pair.feature_count != expected) {
      report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
      report.input_pair = i;
      report.message = "Fixed-triangle task materialization disagrees";
      return report;
    }
    feature_write += pair.feature_count;
    if (intersection.intersects) {
      if (report.raw_intersections == SIZE_MAX) {
        report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
        report.input_pair = i;
        report.message = "Fixed-triangle intersection count overflow";
        return report;
      }
      ++report.raw_intersections;
    }
  }
  if (feature_write != report.raw_feature_candidates) {
    report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
    report.message = "Complete fixed-triangle feature count disagrees";
    return report;
  }
  if (report.raw_intersections >
      impl_->limits.max_raw_intersections) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }

  std::size_t intersection_write = 0;
  for (std::size_t i = 0; i < pair_count; ++i)
    if (impl_->pair_intersections[i].intersects)
      impl_->raw_intersections[intersection_write++] =
          impl_->pair_intersections[i].value;

  std::sort(impl_->raw_features.get(),
            impl_->raw_features.get() + feature_write, ft::FeatureLess);
  std::size_t unique_features = 0;
  for (std::size_t i = 0; i < feature_write; ++i) {
    if (unique_features &&
        ft::SameFeatureKey(impl_->raw_features[unique_features - 1],
                           impl_->raw_features[i])) {
      if (!SameCandidateValue(impl_->raw_features[unique_features - 1],
                              impl_->raw_features[i])) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_features != i)
      impl_->raw_features[unique_features] = impl_->raw_features[i];
    ++unique_features;
  }

  std::sort(impl_->raw_intersections.get(),
            impl_->raw_intersections.get() + intersection_write,
            ft::IntersectionLess);
  std::size_t unique_intersections = 0;
  for (std::size_t i = 0; i < intersection_write; ++i) {
    if (unique_intersections &&
        ft::SameIntersectionPair(
            impl_->raw_intersections[unique_intersections - 1],
            impl_->raw_intersections[i])) {
      const auto& old =
          impl_->raw_intersections[unique_intersections - 1];
      const auto& next = impl_->raw_intersections[i];
      if (old.kind != next.kind ||
          old.local_exclusion != next.local_exclusion) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_intersections != i)
      impl_->raw_intersections[unique_intersections] =
          impl_->raw_intersections[i];
    ++unique_intersections;
  }

  report.feature_candidates = unique_features;
  report.intersections = unique_intersections;
  if (unique_features > impl_->limits.max_feature_candidates ||
      unique_intersections > impl_->limits.max_intersections) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }
  for (std::size_t i = 0; i < unique_features; ++i)
    impl_->features[i] = impl_->raw_features[i];
  for (std::size_t i = 0; i < unique_intersections; ++i)
    impl_->intersections[i] = impl_->raw_intersections[i];
  impl_->feature_count = unique_features;
  impl_->intersection_count = unique_intersections;
  impl_->complete = true;
  return report;
}

FixedTriangleFeatureForecast FixedTriangleFeatureDiscovery::forecast()
    const noexcept {
  return impl_ ? impl_->forecast : FixedTriangleFeatureForecast{};
}

FixedTriangleFeatureView FixedTriangleFeatureDiscovery::features()
    const noexcept {
  if (!impl_ || !impl_->complete)
    return {};
  return {impl_->feature_count ? impl_->features.get() : nullptr,
          impl_->feature_count, true};
}

FixedTriangleIntersectionView FixedTriangleFeatureDiscovery::intersections()
    const noexcept {
  if (!impl_ || !impl_->complete)
    return {};
  return {impl_->intersection_count ? impl_->intersections.get() : nullptr,
          impl_->intersection_count, true};
}

}  // namespace tlfea::contact
