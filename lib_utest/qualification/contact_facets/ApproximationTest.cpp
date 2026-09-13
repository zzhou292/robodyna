// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <cfloat>
#include <cmath>
#include <limits>

namespace facet_test {
// Chosen source ordinates are dyadic with at most53 significant bits; template
// and probe factors add at most10 bits. These scalar z comparisons are therefore
// exact in the independent >=64-bit-significand host oracle (no libm norm).
static_assert(LDBL_MANT_DIG >= 64, "This exact dyadic qualifier needs an extended host significand");

TEST(FixedFacetApproximation, CurrentWarpBoundContainsExactDyadicPatchMinusEveryFacet) {
  Fixture fixture;
  const auto parent = fixture.Parent(100);
  const ct::Vec3 native[]{{0,0,0},{2,0,0},{2,2,1},{0,2,0}};
  const auto positions = fixture.Positions(parent, native);
  for (unsigned level = 0; level <= 2; ++level) {
    ct::FixedContactFacetBinding binding;
    ASSERT_EQ(binding.Initialize(fixture.surface, {{}, level}).status, S::Ok);
    ct::FacetApproximationBound bound;
    ASSERT_EQ(binding.Approximation(parent, View(positions), &bound), ct::Status::kOk);
    const double exact_maximum = std::ldexp(1., -2 - 2 * static_cast<int>(level));
    EXPECT_GE(bound.bilinear_error_upper_m, exact_maximum);
    EXPECT_LE(bound.bilinear_error_upper_m, exact_maximum * (1 + 32 * DBL_EPSILON));
    EXPECT_EQ(bound.vertex_roundoff_upper_m, 0);
    long double measured = 0;
    for (unsigned local = 0; local < binding.facet_count(parent); ++local) {
      ct::FixedContactFacet facet;
      ASSERT_EQ(binding.Describe(parent, local, &facet).status, S::Ok);
      for (unsigned a = 0; a <= 4; ++a) for (unsigned b = 0; a + b <= 4; ++b) {
        const long double lambda[]{a / 4.L, b / 4.L, (4 - a - b) / 4.L};
        long double u = 0, v = 0, linear = 0;
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
          const auto& point = facet.vertices[vertex];
          u += lambda[vertex] * (point.weights[1] + point.weights[2]);
          v += lambda[vertex] * (point.weights[2] + point.weights[3]);
          linear += lambda[vertex] * point.weights[2];
        }
        const long double error = std::fabs(u * v - linear);
        measured = std::max(measured, error);
        EXPECT_LE(error, static_cast<long double>(bound.total_error_upper_m));
      }
    }
    EXPECT_EQ(measured, static_cast<long double>(exact_maximum));
  }
}

TEST(FixedFacetApproximation, AffineAndRigidTransformsPreserveBoundsWithoutVelocityReadsOrThicknessChanges) {
  Fixture fixture;
  const auto parent = fixture.Parent(100);
  ct::FixedContactFacetBinding binding;
  ASSERT_EQ(binding.Initialize(fixture.surface, {{}, 2}).status, S::Ok);
  ct::Vec3 points[]{{0,0,0},{2,0,0},{2,2,0},{0,2,0}};
  auto positions = fixture.Positions(parent, points);
  const auto& source = fixture.surface.parents()[parent];
  for (std::size_t node = 0; node < positions.size() / 3; ++node) {
    bool consumed = false;
    for (unsigned i = 0; i < 4; ++i) consumed |= node == source.q4.nodes[i];
    if (!consumed) positions[3 * node] = std::numeric_limits<double>::quiet_NaN();
  }
  ct::FacetApproximationBound bound;
  ASSERT_EQ(binding.Approximation(parent, View(positions), &bound), ct::Status::kOk);
  EXPECT_EQ(bound.total_error_upper_m, 0);
  points[2].z = 1;
  positions = fixture.Positions(parent, points);
  ASSERT_EQ(binding.Approximation(parent, View(positions), &bound), ct::Status::kOk);
  const auto original = bound;
  for (auto& point : points) point = {-point.y + 8, point.x - 4, point.z + 2};
  positions = fixture.Positions(parent, points);
  ASSERT_EQ(binding.Approximation(parent, View(positions), &bound), ct::Status::kOk);
  EXPECT_EQ(bound.bilinear_error_upper_m, original.bilinear_error_upper_m);
  // Existing interval multiplication may pad an exact non-power-of-two product;
  // a certified upper bound need not be the minimal error of this dyadic case.
  EXPECT_GE(bound.vertex_roundoff_upper_m, 0);
  EXPECT_LT(bound.vertex_roundoff_upper_m, 64 * DBL_EPSILON * 8);
  EXPECT_EQ(source.reference_half_thickness_m, .001);
  const auto triangle = fixture.Parent(200);
  positions = fixture.Positions(triangle, points);
  ASSERT_EQ(binding.Approximation(triangle, View(positions), &bound), ct::Status::kOk);
  EXPECT_EQ(bound.bilinear_error_upper_m, 0);
}

TEST(FixedFacetApproximation, ThreeComponentNormAndCanceledRoundedCrossTermKeepExactBounds) {
  Fixture fixture;
  const auto parent = fixture.Parent(100);
  ct::FixedContactFacetBinding binding;
  ASSERT_EQ(binding.Initialize(fixture.surface, {{}, 2}).status, S::Ok);
  const ct::Vec3 diagonal[]{{0,0,0},{2,0,0},{3,4,2},{0,2,0}};
  auto positions = fixture.Positions(parent, diagonal);
  ct::FacetApproximationBound bound;
  ASSERT_EQ(binding.Approximation(parent, View(positions), &bound), ct::Status::kOk);
  // D=(1,2,2) has exact norm3; no square-root oracle is required.
  EXPECT_GE(bound.bilinear_error_upper_m, 3. / 64);
  EXPECT_LE(bound.bilinear_error_upper_m, (3. / 64) * (1 + 32 * DBL_EPSILON));
  const ct::Vec3 canceled[]{{1e16,0,0},{-1e16,0,0},{-1e16,0,0},{1e16-2,0,0}};
  const double rounded = (canceled[0].x-canceled[1].x) + (canceled[2].x-canceled[3].x);
  EXPECT_EQ(rounded, 0);
  const long double exact = (static_cast<long double>(canceled[0].x)-canceled[1].x) +
      (static_cast<long double>(canceled[2].x)-canceled[3].x);
  EXPECT_EQ(exact, 2);
  positions = fixture.Positions(parent, canceled);
  ASSERT_EQ(binding.Approximation(parent, View(positions), &bound), ct::Status::kOk);
  EXPECT_GE(static_cast<long double>(bound.bilinear_error_upper_m), exact / 64);
}

TEST(FixedFacetApproximation, RepresentedVertexRoundingHasAnIndependentExactEnclosure) {
  Fixture fixture;
  const auto parent = fixture.Parent(100);
  const ct::Vec3 points[]{{0,0,std::nextafter(1.,2.)},{2,0,0},{2,2,0},{0,2,0}};
  const auto positions = fixture.Positions(parent, points);
  ct::FixedContactFacetBinding binding;
  ASSERT_EQ(binding.Initialize(fixture.surface, {{}, 2}).status, S::Ok);
  ct::FacetApproximationBound bound;
  ASSERT_EQ(binding.Approximation(parent, View(positions), &bound), ct::Status::kOk);
  EXPECT_GT(bound.vertex_roundoff_upper_m, 0);
  long double maximum = 0;
  for (unsigned local = 0; local < binding.facet_count(parent); ++local) {
    ct::FixedContactFacet facet;
    ASSERT_EQ(binding.Describe(parent, local, &facet).status, S::Ok);
    for (const auto& vertex : facet.vertices) {
      ct::Vec3 represented;
      ASSERT_EQ(ct::EvaluateWeightedSurfacePosition(View(positions), vertex, &represented), ct::Status::kOk);
      const long double exact = static_cast<long double>(points[0].z) * vertex.weights[0];
      const long double error = std::fabs(exact - represented.z);
      maximum = std::max(maximum, error);
      EXPECT_LE(error, static_cast<long double>(bound.vertex_roundoff_upper_m));
    }
  }
  EXPECT_GT(maximum, 0);
}

TEST(FixedFacetApproximation, LateInvalidAndUnrepresentableBoundsPreserveThenRetry) {
  Fixture fixture;
  const auto parent = fixture.Parent(100);
  ct::FixedContactFacetBinding binding;
  ASSERT_EQ(binding.Initialize(fixture.surface, {{}, 2}).status, S::Ok);
  ct::FacetApproximationBound output{3,4,5};
  const auto old = qbat_binding_test::Bytes(output);
  const ct::Vec3 points[]{{0,0,0},{2,0,0},{2,2,1},{0,2,0}};
  auto positions = fixture.Positions(parent, points);
  auto bad = positions;
  bad[3 * fixture.surface.parents()[parent].q4.nodes[3]] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(binding.Approximation(parent, View(bad), &output), ct::Status::kInvalidArgument);
  EXPECT_EQ(qbat_binding_test::Bytes(output), old);
  for (double value : {DBL_MAX, std::numeric_limits<double>::denorm_min()}) {
    const ct::Vec3 extreme[]{{value,0,0},{-value,0,0},{value,0,0},{-value,0,0}};
    bad = fixture.Positions(parent, extreme);
    EXPECT_EQ(binding.Approximation(parent, View(bad), &output), ct::Status::kNonFiniteResult);
    EXPECT_EQ(qbat_binding_test::Bytes(output), old);
  }
  EXPECT_EQ(binding.Approximation(SIZE_MAX, View(positions), &output), ct::Status::kOutOfRange);
  EXPECT_EQ(binding.Approximation(parent, View(positions), reinterpret_cast<ct::FacetApproximationBound*>(positions.data())),
      ct::Status::kInvalidArgument);
  EXPECT_EQ(binding.Approximation(parent, View(positions), &output), ct::Status::kOk);
}
TEST(FixedFacetApproximation, CompleteSummaryMatchesEveryParentAndPreservesFailureOutput) {
  Fixture fixture;
  const auto parent = fixture.Parent(100);
  ct::FixedContactFacetBinding binding;
  ASSERT_EQ(binding.Initialize(fixture.surface, {{}, 2}).status, S::Ok);
  const ct::Vec3 points[]{{0,0,0},{2,0,0},{3,4,2},{0,2,0}};
  auto positions = fixture.Positions(parent, points);
  ct::FacetApproximationSummary summary;
  ASSERT_EQ(binding.SummarizeApproximation(View(positions), &summary), ct::Status::kOk);
  EXPECT_EQ(summary.parents, fixture.surface.parents().size());
  std::size_t positive_bilinear = 0, positive_roundoff = 0;
  double maximum_bilinear = 0, maximum_roundoff = 0, maximum_total = 0;
  std::size_t bilinear_parent = SIZE_MAX, roundoff_parent = SIZE_MAX, total_parent = SIZE_MAX;
  for (std::size_t p = 0; p < fixture.surface.parents().size(); ++p) {
    ct::FacetApproximationBound bound;
    ASSERT_EQ(binding.Approximation(p, View(positions), &bound), ct::Status::kOk);
    positive_bilinear += bound.bilinear_error_upper_m > 0;
    positive_roundoff += bound.vertex_roundoff_upper_m > 0;
    const auto update = [&](double value, double& current, std::size_t& witness) {
      if (value > current) { current = value; witness = p; }
    };
    update(bound.bilinear_error_upper_m, maximum_bilinear, bilinear_parent);
    update(bound.vertex_roundoff_upper_m, maximum_roundoff, roundoff_parent);
    update(bound.total_error_upper_m, maximum_total, total_parent);
  }
  EXPECT_EQ(summary.positive_bilinear_parents, positive_bilinear);
  EXPECT_EQ(summary.positive_vertex_roundoff_parents, positive_roundoff);
  EXPECT_EQ(summary.maximum_bilinear_error_upper_m, maximum_bilinear);
  EXPECT_EQ(summary.maximum_vertex_roundoff_upper_m, maximum_roundoff);
  EXPECT_EQ(summary.maximum_total_error_upper_m, maximum_total);
  EXPECT_EQ(summary.maximum_bilinear_parent, bilinear_parent);
  EXPECT_EQ(summary.maximum_vertex_roundoff_parent, roundoff_parent);
  EXPECT_EQ(summary.maximum_total_parent, total_parent);

  const auto old = qbat_binding_test::Bytes(summary);
  const auto bad_node = 3 * fixture.surface.parents()[parent].q4.nodes[0];
  const auto old_position = positions[bad_node];
  positions[bad_node] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(binding.SummarizeApproximation(View(positions), &summary),
      ct::Status::kInvalidArgument);
  EXPECT_EQ(qbat_binding_test::Bytes(summary), old);
  EXPECT_EQ(binding.SummarizeApproximation(View(positions),
      reinterpret_cast<ct::FacetApproximationSummary*>(positions.data())),
      ct::Status::kInvalidArgument);
  positions[bad_node] = old_position;
  EXPECT_EQ(binding.SummarizeApproximation(View(positions), &summary),
      ct::Status::kOk);
  EXPECT_EQ(qbat_binding_test::Bytes(summary), old);
}
} // namespace facet_test
