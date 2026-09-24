// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include <stdexcept>
namespace facet_filter_benchmark {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void Scene::Initialize(std::size_t count, std::string_view pattern) {
  Require(!::testing::UnitTest::GetInstance()->ad_hoc_test_result().Failed(),
          "Existing binding fixture reported a setup assertion failure");
  Require(count && count <= 4096, "Pair count must be 1..4096");
  Require(source.uses.prepared(), "Active-use fixture is not prepared");
  Require(source.source.surface.prepared() && source.source.facets.prepared() &&
      source.source.catalog.prepared() && source.source.domain.node_count() &&
      source.positions.size() == 3 * source.source.domain.node_count(),
      "Source geometry/material/domain fixture is not ready");
  const auto uses = source.uses.facet_uses();
  const auto parents = source.uses.parents();
  Require(uses.size() >= 6 && !parents.empty(), "Fixture has insufficient authenticated facets");
  descriptors.resize(uses.size()); accepted.resize(uses.size()); prepared.resize(uses.size());
  motion.resize(uses.size()); bounds.resize(uses.size());
  for (std::size_t i = 0; i < uses.size(); ++i) {
    Require(uses[i].parent < parents.size(), "Fixture facet parent is invalid");
    const auto described = source.source.facets.Describe(parents[uses[i].parent].surface_parent,
        uses[i].local_facet, &descriptors[i]);
    Require(described.status == c::FixedContactFacetStatus::Ok, described.message);
    motion[i].parent = uses[i].parent;
    motion[i].certified_affine = true;
  }
  const auto evaluated = sct::EvaluateCompleteTriangles(descriptors.data(), descriptors.size(),
      source.Positions(), accepted.data());
  Require(evaluated.status == c::SelfContactTransactionStatus::Ok, evaluated.message);
  prepared = accepted;
  // A deliberately broad common box encloses the small fixture. It prevents
  // an AABB early exit so this measures prism-query orchestration consistently.
  for (const auto& triangle : accepted)
    for (const auto point : triangle.vertices)
      Require(c::IsFinite(point.x) && c::IsFinite(point.y) && c::IsFinite(point.z) &&
          std::abs(point.x) < 1 && std::abs(point.y) < 1 && std::abs(point.z) < 1,
          "Fixture escaped the declared conservative common bounds");
  std::fill(bounds.begin(), bounds.end(), c::SelfContactSweptParentBounds{{-1,-1,-1},{1,1,1}});
  // Synthetic routing declarations only. No nonlinear proof or rigid support
  // acceptance is claimed by this component benchmark.
  motion[2].certified_affine = false;
  motion[2].motion = c::SelfContactFacetMotion::CompleteRigidGroup;
  motion[2].complete_rigid_group = 2;
  motion[3].motion = motion[4].motion = c::SelfContactFacetMotion::CompleteRigidGroup;
  motion[3].complete_rigid_group = motion[4].complete_rigid_group = 7;
  Require(pattern == "all_linear" || pattern == "alternating" ||
      pattern == "short_islands" || pattern == "no_linear", "Unknown routing pattern");
  pairs.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    bool linear = pattern == "all_linear" || (pattern == "alternating" && !(i & 1)) ||
        (pattern == "short_islands" && i % 10 < 8);
    if (linear) pairs.push_back({0, (i & 2) ? 5u : 1u});
    else if (i & 1) pairs.push_back({0, 2});
    else pairs.push_back({3, 4});
    const auto pair = pairs.back();
    const auto action = sct::ClassifyCandidatePairMotion(motion[pair.first], bounds[pair.first],
        motion[pair.second], bounds[pair.second]);
    linear_rows += action == sct::PairMotionAction::LinearNodalV1;
    nonlinear_rows += action == sct::PairMotionAction::UnsupportedRigidArc;
    excluded_rows += action == sct::PairMotionAction::ExcludedSameRigidGroup;
  }
  Require(linear_rows + nonlinear_rows + excluded_rows == count, "Unexpected synthetic routing action");
  Require(!::testing::UnitTest::GetInstance()->ad_hoc_test_result().Failed(),
          "Fixture setup assertions failed");
}
Row Scene::Scalar(std::size_t ordinal) const {
  const auto pair = pairs[ordinal];
  Row row;
  row.action = sct::ClassifyCandidatePairMotion(motion[pair.first], bounds[pair.first],
      motion[pair.second], bounds[pair.second]);
  if (row.action == sct::PairMotionAction::LinearNodalV1) {
    const auto parents = source.uses.parents();
    bool valid = false;
    row.numerical.separated = c::CertifiedLinearFacetPrismSeparation(
        accepted[pair.first], prepared[pair.first],
        parents[motion[pair.first].parent].reference_half_thickness_m,
        accepted[pair.second], prepared[pair.second],
        parents[motion[pair.second].parent].reference_half_thickness_m,
        c::SelfContactFacetPrismAxisLimit::VertexVertex, &row.numerical.axis, &valid);
    row.numerical.status = valid ? c::SelfContactFacetFilterStatus::Ok : c::SelfContactFacetFilterStatus::InvalidInput;
  }
  return row;
}
}  // namespace facet_filter_benchmark
