// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../FixedTriangleFeatureDiscovery.h"

#include "Geometry.h"

#include <algorithm>
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

bool Same(Vec3 a, Vec3 b) noexcept {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool SameCandidateValue(const FixedTriangleFeatureCandidate& a,
                        const FixedTriangleFeatureCandidate& b) noexcept {
  if (!ft::SameFeatureTask(a, b) || a.distance_m != b.distance_m)
    return false;
  for (unsigned i = 0; i < 2; ++i) {
    if (!Same(a.points[i], b.points[i]) ||
        a.edge_parameters[i] != b.edge_parameters[i])
      return false;
  }
  for (unsigned i = 0; i < 3; ++i)
    if (a.face_weights[i] != b.face_weights[i])
      return false;
  return true;
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
  std::unique_ptr<FixedTriangleFeatureCandidate[]> features;
  std::unique_ptr<FixedTriangleIntersection[]> intersections;
  std::size_t feature_count = 0;
  std::size_t intersection_count = 0;
  bool complete = false;

  void Revoke() noexcept {
    feature_count = 0;
    intersection_count = 0;
    complete = false;
  }
};

FixedTriangleFeatureDiscovery::FixedTriangleFeatureDiscovery() noexcept =
    default;
FixedTriangleFeatureDiscovery::~FixedTriangleFeatureDiscovery() = default;

FixedTriangleFeaturePreflight FixedTriangleFeatureDiscovery::Preflight(
    FixedTriangleFeatureLimits limits) noexcept {
  FixedTriangleFeaturePreflight result;
  result.forecast.raw_feature_capacity =
      limits.max_raw_feature_candidates;
  result.forecast.raw_intersection_capacity =
      limits.max_raw_intersections;
  std::size_t bytes = sizeof(Impl);
  const bool pair_product_ok =
      limits.max_input_pairs <=
      std::numeric_limits<std::size_t>::max() / 15;
  const bool arithmetic_ok =
      CheckedBytes(limits.max_raw_feature_candidates,
                   sizeof(FixedTriangleFeatureCandidate), &bytes) &&
      CheckedBytes(limits.max_raw_intersections,
                   sizeof(FixedTriangleIntersection), &bytes);
  result.forecast.owned_host_bytes = arithmetic_ok ? bytes : SIZE_MAX;
  if (!limits.max_input_pairs || !limits.max_raw_feature_candidates ||
      !limits.max_raw_intersections || !pair_product_ok ||
      limits.max_raw_feature_candidates >
          15 * limits.max_input_pairs ||
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
  if (impl_)
    return {FixedTriangleDiscoveryStatus::AlreadyInitialized, SIZE_MAX, 0, 0,
            0, 0, 0,
            Message(FixedTriangleDiscoveryStatus::AlreadyInitialized)};
  const auto preflight = Preflight(limits);
  if (preflight.report.status != FixedTriangleDiscoveryStatus::Ok)
    return preflight.report;
  auto next = std::unique_ptr<Impl>(new (std::nothrow) Impl);
  if (!next)
    return {FixedTriangleDiscoveryStatus::ResourceLimit, SIZE_MAX, 0, 0, 0, 0,
            0, "Feature discovery control allocation failed"};
  next->limits = limits;
  next->forecast = preflight.forecast;
  next->features.reset(new (std::nothrow)
                           FixedTriangleFeatureCandidate[
                               limits.max_raw_feature_candidates]);
  next->intersections.reset(
      new (std::nothrow)
          FixedTriangleIntersection[limits.max_raw_intersections]);
  if (!next->features || !next->intersections)
    return {FixedTriangleDiscoveryStatus::ResourceLimit, SIZE_MAX, 0, 0, 0, 0,
            0, "Feature discovery bounded arena allocation failed"};
  impl_ = std::move(next);
  return {};
}

FixedTriangleDiscoveryReport FixedTriangleFeatureDiscovery::Discover(
    const CurrentFixedTriangle* triangles, std::size_t triangle_count,
    const FixedTrianglePair* pairs, std::size_t pair_count) noexcept {
  if (!impl_)
    return {FixedTriangleDiscoveryStatus::NotInitialized, SIZE_MAX, 0, 0, 0, 0,
            0, Message(FixedTriangleDiscoveryStatus::NotInitialized)};
  impl_->Revoke();
  FixedTriangleDiscoveryReport report;
  if (pair_count > impl_->limits.max_input_pairs) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }
  if (pair_count == 0) {
    impl_->complete = true;
    return report;
  }
  if (!triangles || !triangle_count || !pairs) {
    report.status = FixedTriangleDiscoveryStatus::InvalidInput;
    report.message = Message(report.status);
    return report;
  }

  for (std::size_t i = 0; i < pair_count; ++i) {
    if (pairs[i].first >= triangle_count ||
        pairs[i].second >= triangle_count) {
      report.status = FixedTriangleDiscoveryStatus::OutOfRange;
      report.input_pair = i;
      report.message = Message(report.status);
      return report;
    }
    ft::PairResult pair;
    const auto status = ft::EvaluatePair(triangles[pairs[i].first],
                                         triangles[pairs[i].second], &pair);
    if (status != FixedTriangleDiscoveryStatus::Ok) {
      report.status = status;
      report.input_pair = i;
      report.message = Message(status);
      return report;
    }
    if (pair.feature_task_count >
        std::numeric_limits<std::size_t>::max() -
            report.feature_tasks) {
      report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
      report.input_pair = i;
      report.message = "Fixed-triangle feature task count overflow";
      return report;
    }
    report.feature_tasks += pair.feature_task_count;
    if (pair.feature_count >
        std::numeric_limits<std::size_t>::max() -
            report.raw_feature_candidates) {
      report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
      report.input_pair = i;
      report.message = "Fixed-triangle feature count overflow";
      return report;
    }
    report.raw_feature_candidates += pair.feature_count;
    if (pair.intersects) {
      if (report.raw_intersections == SIZE_MAX) {
        report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
        report.input_pair = i;
        report.message = "Fixed-triangle intersection count overflow";
        return report;
      }
      ++report.raw_intersections;
    }
  }
  if (report.raw_feature_candidates >
          impl_->limits.max_raw_feature_candidates ||
      report.raw_intersections > impl_->limits.max_raw_intersections) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }

  std::size_t feature_write = 0;
  std::size_t intersection_write = 0;
  for (std::size_t i = 0; i < pair_count; ++i) {
    ft::PairResult pair;
    const auto status = ft::EvaluatePair(triangles[pairs[i].first],
                                         triangles[pairs[i].second], &pair);
    if (status != FixedTriangleDiscoveryStatus::Ok) {
      report.status = status;
      report.input_pair = i;
      report.message = Message(status);
      return report;
    }
    for (std::size_t j = 0; j < pair.feature_count; ++j)
      impl_->features[feature_write++] = pair.features[j];
    if (pair.intersects)
      impl_->intersections[intersection_write++] = pair.intersection;
  }

  std::sort(impl_->features.get(),
            impl_->features.get() + feature_write, ft::FeatureLess);
  std::size_t unique_features = 0;
  for (std::size_t i = 0; i < feature_write; ++i) {
    if (unique_features &&
        ft::SameFeatureTask(impl_->features[unique_features - 1],
                            impl_->features[i])) {
      if (!SameCandidateValue(impl_->features[unique_features - 1],
                              impl_->features[i])) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_features != i)
      impl_->features[unique_features] = impl_->features[i];
    ++unique_features;
  }

  std::sort(impl_->intersections.get(),
            impl_->intersections.get() + intersection_write,
            ft::IntersectionLess);
  std::size_t unique_intersections = 0;
  for (std::size_t i = 0; i < intersection_write; ++i) {
    if (unique_intersections &&
        ft::SameIntersectionPair(
            impl_->intersections[unique_intersections - 1],
            impl_->intersections[i])) {
      const auto& old = impl_->intersections[unique_intersections - 1];
      const auto& next = impl_->intersections[i];
      if (old.kind != next.kind ||
          old.local_exclusion != next.local_exclusion) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_intersections != i)
      impl_->intersections[unique_intersections] =
          impl_->intersections[i];
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
