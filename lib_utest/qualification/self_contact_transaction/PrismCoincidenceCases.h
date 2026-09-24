// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
// Included after ContinuousLocalTest's shared triangle/captured Yaris helpers.
namespace prism_test {
using Limit = c::SelfContactFacetPrismAxisLimit;
using Axis = c::SelfContactFacetPrismSeparationAxis;
namespace q = c::self_contact_filters;
constexpr std::array<Limit, 4> Limits{{Limit::FaceNormal, Limit::EdgeCross,
                                    Limit::VertexEdge, Limit::VertexVertex}};
c::CurrentFixedTriangle Shift(c::CurrentFixedTriangle value, c::Vec3 delta) {
  for (auto& point : value.vertices) point = c::Add(point, delta);
  return value;
}
c::CurrentFixedTriangle Flat(std::uint64_t id = 10) {
  return Triangle(id, {id, id + 1, id + 2}, {{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}});
}
q::PrismComparison Compare(
    const c::CurrentFixedTriangle& a, const c::CurrentFixedTriangle& ap,
    const c::CurrentFixedTriangle& b, const c::CurrentFixedTriangle& bp,
    Limit limit = Limit::VertexVertex, double ah = .01, double bh = .02) {
  const auto result = q::ComparePrismHullCoincidence(a, ap, ah, b, bp, bh, limit);
  EXPECT_EQ(result.current.separated, result.original.separated);
  EXPECT_EQ(result.current.valid, result.original.valid);
  EXPECT_EQ(result.current.axis, result.original.axis);
  bool valid = false;
  Axis axis = Axis::VertexVertex;
  const auto separated = c::CertifiedLinearFacetPrismSeparation(a, ap, ah, b, bp, bh, limit, &axis, &valid);
  EXPECT_EQ(separated, result.current.separated);
  EXPECT_EQ(valid, result.current.valid); EXPECT_EQ(axis, result.current.axis);
  EXPECT_LE(result.current.counts.coordinate_tests, 36u);
  EXPECT_LE(result.current.counts.axis_tests, 148u);
  return result;
}
void Coincident(const q::PrismComparison& result) {
  EXPECT_TRUE(result.current.valid);
  EXPECT_FALSE(result.current.separated);
  EXPECT_EQ(result.current.axis, Axis::None);
  EXPECT_TRUE(result.current.counts.hull_coincidence);
  EXPECT_EQ(result.current.counts.axis_tests, 0u);
  EXPECT_GT(result.original.counts.axis_tests, 0u);
}
}  // namespace prism_test

TEST(SelfContactPrismCoincidence, StaticSharedPointRetainsAllAxisLimitResults) {
  using namespace prism_test;
  const auto a = Flat(), b = Triangle(20, {40, 41, 42}, {{{0, 0, 0}, {0, -1, 1}, {0, -1, -1}}});
  const unsigned expected[]{4, 13, 31, 40};
  for (unsigned limit = 0; limit < Limits.size(); ++limit) {
    const auto compared = Compare(a, a, b, b, Limits[limit]);
    Coincident(compared); EXPECT_EQ(compared.original.counts.axis_tests, expected[limit]);
  }
  const auto accepted = c::ClassifyAcceptedFacetPair(a, .01, UINT32_MAX, b, .02, UINT32_MAX);
  EXPECT_EQ(accepted.status, c::SelfContactFacetFilterStatus::Ok);
  EXPECT_EQ(accepted.category, c::SelfContactFacetFilterCategory::ExactRemaining);
}
TEST(SelfContactPrismCoincidence, EveryEndpointCombinationIsOnlyANonseparationFact) {
  using namespace prism_test;
  for (unsigned first_endpoint = 0; first_endpoint < 2; ++first_endpoint)
    for (unsigned second_endpoint = 0; second_endpoint < 2; ++second_endpoint) {
      c::CurrentFixedTriangle a[]{Flat(), Shift(Flat(), {10, 0, 0})};
      c::CurrentFixedTriangle b[]{Shift(Flat(30), {20, 2, 2}), Shift(Flat(30), {30, 2, 2})};
      b[second_endpoint].vertices[1] = a[first_endpoint].vertices[2];
      const auto compared = Compare(a[0], a[1], b[0], b[1]);
      Coincident(compared); EXPECT_EQ(compared.original.counts.axis_tests, 148u);
    }
}
TEST(SelfContactPrismCoincidence, CrossTimeHullCoincidenceDoesNotClaimPhysicalIntersection) {
  using namespace prism_test;
  const auto a = Flat(), ap = Shift(a, {10, 0, 0});
  const auto b = Shift(Flat(30), {10, 0, 0}), bp = Shift(b, {10, 0, 0});
  // A and B remain ten units apart under a common translation. Their existing
  // time-uncorrelated swept hulls coincide at A's end/B's beginning, however.
  // Returning unresolved leaves actual continuous geometry to the native path.
  const auto compared = Compare(a, ap, b, bp);
  Coincident(compared); EXPECT_EQ(compared.original.counts.axis_tests, 148u);
}
TEST(SelfContactPrismCoincidence, SignedZeroAndDifferentSourceKeysStillShareProjectionValues) {
  using namespace prism_test;
  auto a = Flat(), b = Flat(1000);
  a.vertices[0] = {-0., 0., -0.}; b.vertices[0] = {0., -0., 0.};
  EXPECT_NE(a.vertex_keys[0].first, b.vertex_keys[0].first);
  Coincident(Compare(a, a, b, b));
}
TEST(SelfContactPrismCoincidence, AdjacentNonzeroCoordinatesDoNotBecomeCoincidences) {
  using namespace prism_test;
  const auto a = Flat();
  for (double height : {std::numeric_limits<double>::denorm_min(), .5}) {
    const auto b = Shift(Flat(30), {0, 0, height});
    for (auto limit : Limits) {
      const auto compared = Compare(a, a, b, b, limit);
      EXPECT_FALSE(compared.current.counts.hull_coincidence);
      EXPECT_EQ(compared.current.counts.coordinate_tests, 36u);
      EXPECT_EQ(compared.current.counts.axis_tests, compared.original.counts.axis_tests);
      EXPECT_EQ(compared.current.separated, height == .5);
    }
  }
}
TEST(SelfContactPrismCoincidence, CoincidenceNeverMasksInvalidOtherVerticesThicknessOrEnum) {
  using namespace prism_test;
  const auto a = Flat(), b = Flat(30);
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto infinity = std::numeric_limits<double>::infinity();
  const auto invalid = [](const q::PrismComparison& compared) {
    EXPECT_FALSE(compared.current.valid || compared.current.separated);
    EXPECT_EQ(compared.current.axis, Axis::None);
    EXPECT_EQ(compared.current.counts.coordinate_tests, 0u);
    EXPECT_EQ(compared.current.counts.axis_tests, 0u);
  };
  for (double thickness : {0., -1., nan, infinity}) {
    invalid(Compare(a, a, b, b, Limit::VertexVertex, thickness, .01));
    invalid(Compare(a, a, b, b, Limit::VertexVertex, .01, thickness));
  }
  invalid(Compare(a, a, b, b, static_cast<Limit>(255)));
  for (unsigned endpoint = 0; endpoint < 4; ++endpoint)
    for (double bad : {nan, infinity, -infinity}) {
      c::CurrentFixedTriangle endpoints[]{a, a, b, b};
      endpoints[endpoint].vertices[2].z = bad; // The first point still coincides.
      invalid(Compare(endpoints[0], endpoints[1], endpoints[2], endpoints[3]));
    }
  Axis axis = Axis::EdgeCross;
  EXPECT_FALSE(c::CertifiedLinearFacetPrismSeparation(a, a, .01, b, b, .01,
      Limit::VertexVertex, &axis, nullptr));
  EXPECT_EQ(axis, Axis::EdgeCross); // Original null-valid early return is prior to axis initialization.
  bool valid = false;
  EXPECT_FALSE(c::CertifiedLinearFacetPrismSeparation(a, a, .01, b, b, .01,
      Limit::VertexVertex, nullptr, &valid));
  EXPECT_TRUE(valid);
}
TEST(SelfContactPrismCoincidence, ExtremeFiniteGeometryRetainsOverflowAsUnresolved) {
  using namespace prism_test;
  const auto m = std::numeric_limits<double>::max();
  const auto a = Triangle(10, {1, 2, 3}, {{{m, m, m}, {-m, m, m}, {m, -m, m}}});
  const auto ap = Triangle(10, {1, 2, 3}, {{{m, m, m}, {-m, m, -m}, {m, -m, -m}}});
  const auto b = Triangle(20, {4, 5, 6}, {{{m, m, m}, {m, m, -m}, {-m, -m, m}}});
  const auto bp = Triangle(20, {4, 5, 6}, {{{m, m, m}, {-m, -m, -m}, {m, -m, m}}});
  for (double thickness : {.01, m}) Coincident(Compare(a, ap, b, bp, Limit::VertexVertex, thickness, thickness));
}
TEST(SelfContactPrismCoincidence, CapturedYarisFamilyEliminatesAxesWithoutChangingAnyResult) {
  using namespace prism_test;
  // Reuse the exact failure8 coordinate bits and source-pinned fixture. Its
  // original half-thickness is 0.0005 m on both sides; source IDs remain generic.
  const CapturedConeGeometry source;
  std::size_t pairs = 0, original_axes = 0, current_axes = 0;
  for (const auto& coordinates : ConePermutations)
    for (unsigned signs = 0; signs < 8; ++signs) {
      const ConeAxisTransform transform{coordinates, {{signs & 1 ? -1 : 1,
          signs & 2 ? -1 : 1, signs & 4 ? -1 : 1}}};
      for (const auto& vertices : ConePermutations) {
        const auto a = TransformConeTriangle(source.first, transform, vertices);
        const auto ap = TransformConeTriangle(source.first_prepared, transform, vertices);
        const auto b = TransformConeTriangle(source.second, transform, vertices);
        const auto bp = TransformConeTriangle(source.second_prepared, transform, vertices);
        const auto compared = Compare(a, ap, b, bp, Limit::VertexVertex, .0005, .0005);
        Coincident(compared); ++pairs;
        original_axes += compared.original.counts.axis_tests;
        current_axes += compared.current.counts.axis_tests;
      }
    }
  EXPECT_EQ(pairs, 288u); EXPECT_EQ(original_axes, 148 * pairs); EXPECT_EQ(current_axes, 0u);
  // Operation counts on this bounded captured family, not a vehicle hit-rate
  // census or elapsed-time speedup measurement.
}
