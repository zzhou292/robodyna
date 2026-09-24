// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossing.h"
#include "ExactProjectionDomain.h"
#include "NormalReuseQualification.h"

namespace tlfea::contact::represented_interval_crossing {
struct ExactPathReuseCounters {
  std::size_t single_sample_intervals = 0;
  std::size_t vertex_face_tests = 0;
  std::size_t edge_edge_tests = 0;
  std::size_t dominated_edge_loops = 0;
  bool saturated = false;
};
struct ExactPathReuseResult {
  RepresentedIntervalResult result;
  NormalReuseCounters exact;
  ExactPathReuseCounters reused;
};
struct ExactPathReuseComparison {
  RepresentedIntervalStatus status = RepresentedIntervalStatus::InvalidInput;
  ExactProjectionDomainReport domain;
  ExactPathReuseResult original, current;
};
// Private value qualification. Both fixed compile-time executors share the
// same predicates and canonical publication; no runtime mode or receipt exists.
ExactPathReuseComparison CompareExactPathReuse(const RepresentedTrianglePath&,
    const RepresentedTrianglePath&, RepresentedIntervalLimits) noexcept;
}  // namespace tlfea::contact::represented_interval_crossing
