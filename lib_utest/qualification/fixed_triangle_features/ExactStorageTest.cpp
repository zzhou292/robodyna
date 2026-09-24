// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ExactStorageOracle.h"
#include "lib_src/collision/fixed_triangle_features/ExactPredicateKernel.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>

namespace {
namespace c = tlfea::contact;
namespace e = c::fixed_triangle_features::exact;
namespace d = e::detail;
namespace oracle = exact_storage_test;
constexpr auto Adaptive = d::Storage::Adaptive;
constexpr auto Wide = d::Storage::Wide;
void Same(e::Sign actual, e::Sign expected) {
  EXPECT_EQ(actual.valid, expected.valid); EXPECT_EQ(actual.value, expected.value);
}
void Compare(const std::array<c::Vec3, 4>& points, bool finite = true) {
  const auto a = points[0], b = points[1], z = points[2], p = points[3];
  for (unsigned axis = 0; axis < 3; ++axis) {
    const auto expected = d::EvaluateOrient2D<Wide>(a, b, z, axis);
    const auto actual = d::EvaluateOrient2D<Adaptive>(a, b, z, axis);
    Same(actual, expected); Same(e::Orient2D(a, b, z, axis), expected);
    if (finite) { EXPECT_TRUE(actual.valid); EXPECT_EQ(actual.value, oracle::Orient2D(a, b, z, axis)); }
  }
  const auto orientation = d::EvaluateOrient3D<Adaptive>(a, b, z, p);
  Same(orientation, d::EvaluateOrient3D<Wide>(a, b, z, p));
  Same(e::Orient3D(a, b, z, p), orientation);
  const auto directed = d::EvaluateDirectedTriangle<Adaptive>(a, b, z, p);
  Same(directed, d::EvaluateDirectedTriangle<Wide>(a, b, z, p));
  Same(e::DirectedTriangle(a, b, z, p), directed);
  if (finite) {
    EXPECT_TRUE(orientation.valid && directed.valid);
    EXPECT_EQ(orientation.value, oracle::Orient3D(a, b, z, p));
    EXPECT_EQ(directed.value, oracle::Directed(a, b, z, p));
  }
  const c::Vec3 triangle[]{a, b, z};
  e::ClosestTriangleStratum actual{e::ClosestStratumKind::Face, 91}, expected = actual, published = actual;
  const bool valid = d::EvaluateClosestStratum<Adaptive>(p, triangle, &actual);
  EXPECT_EQ(valid, d::EvaluateClosestStratum<Wide>(p, triangle, &expected));
  EXPECT_EQ(valid, e::ClosestStratum(p, triangle, &published));
  EXPECT_EQ(actual.kind, expected.kind); EXPECT_EQ(actual.local, expected.local);
  EXPECT_EQ(published.kind, expected.kind); EXPECT_EQ(published.local, expected.local);
  if (finite) {
    EXPECT_TRUE(valid); const auto answer = oracle::Closest(p, triangle);
    EXPECT_EQ(actual.kind, answer.kind); EXPECT_EQ(actual.local, answer.local);
  }
}
TEST(ExactPredicateStorage, ActualNonzeroExponentsBoundNarrowThresholdAndKeepZeroScale) {
  const double zero[]{0., -0.};
  auto domain = d::AnalyzeCoordinates(zero, 2);
  EXPECT_TRUE(domain.narrow()); EXPECT_EQ(domain.exponent, 0); EXPECT_EQ(domain.coordinate_bits, 0u);
  for (int exponent : {-71, -72, -73}) {
    const double coordinates[]{0., 1., -0., std::ldexp(1., exponent)};
    domain = d::AnalyzeCoordinates(coordinates, 4);
    EXPECT_EQ(domain.exponent, exponent - 52);
    EXPECT_EQ(domain.coordinate_bits, 53u + unsigned(-exponent));
    EXPECT_EQ(domain.narrow(), exponent >= -72);
  }
  const double subnormal[]{0., std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::min()};
  domain = d::AnalyzeCoordinates(subnormal, 3);
  EXPECT_TRUE(domain.narrow()); EXPECT_EQ(domain.exponent, -1074); EXPECT_EQ(domain.coordinate_bits, 53u);
  const double extreme[]{std::numeric_limits<double>::max(), std::numeric_limits<double>::denorm_min()};
  domain = d::AnalyzeCoordinates(extreme, 2);
  EXPECT_FALSE(domain.narrow()); EXPECT_EQ(domain.coordinate_bits, 2098u);
  for (double bad : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    const double coordinates[]{0., bad};
    EXPECT_FALSE(d::AnalyzeCoordinates(coordinates, 2).narrow());
    EXPECT_FALSE(d::AnalyzeCoordinates(coordinates, 2).finite);
  }
}
TEST(ExactPredicateStorage, ThresholdCarryCancellationAndAdjacentPlanesMatchUnlimitedRationals) {
  for (int exponent : {-71, -72, -73, -100, -1074}) {
    const auto tiny = std::ldexp(1., exponent);
    const auto high = std::nextafter(1., 2.);
    for (double height : {-tiny, -0., tiny})
      Compare({{{1, -1, tiny}, {-1, high, -tiny}, {high, -high, tiny}, {tiny, -tiny, height}}});
  }
  for (double height : {std::nextafter(0., -1.), 0., std::nextafter(0., 1.)})
    Compare({{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {.25, .25, height}}});
}
TEST(ExactPredicateStorage, UniformScalesSignedZerosAndFullExponentMixturesRemainExact) {
  for (int exponent : {-1074, -1022, -500, 0, 500, 1000}) {
    const auto a = std::ldexp(1., exponent), b = std::ldexp(2., exponent);
    Compare({{{-0., 0., a}, {b, a, 0}, {a, b, -a}, {a, -a, a}}});
  }
  const auto large = std::numeric_limits<double>::max();
  const auto small = std::numeric_limits<double>::denorm_min();
  Compare({{{large, small, 0}, {-large, 0, small}, {small, -large, 0}, {0, small, large}}});
  Compare({{{0, -0., 0}, {-0., 0, 0}, {0, 0, -0.}, {-0., -0., -0.}}});
}
TEST(ExactPredicateStorage, AllVoronoiRegionsAndClosedBoundaryTiesPreserveBranchOrder) {
  for (const auto point : std::array<c::Vec3, 10>{{
      {-1, -1, 0}, {3, 0, 0}, {0, 3, 0}, {1, -1, 0}, {-1, 1, 0}, {2, 2, 0},
      {.5, .5, 1}, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}}})
    Compare({{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, point}});
  for (double offset : {std::nextafter(0., -1.), 0., std::nextafter(0., 1.)})
    Compare({{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {1, offset, .25}}});
}
TEST(ExactPredicateStorage, DeterministicMixedSignsAndPermutationCorpusPreservesEverySignAndStratum) {
  std::uint64_t state = 0x123456789abcdef0ull;
  for (unsigned row = 0; row < 48; ++row) {
    std::array<c::Vec3, 4> points;
    for (auto& point : points) for (double* coordinate : {&point.x, &point.y, &point.z}) {
      state ^= state << 13; state ^= state >> 7; state ^= state << 17;
      const int mantissa = int(state % 33) - 16;
      const int exponent = row % 3 == 0 ? -int((state >> 8) % 74) : int((state >> 8) % 9) - 4;
      *coordinate = std::ldexp(double(mantissa), exponent);
    }
    Compare(points); std::swap(points[1], points[2]); Compare(points);
  }
}
TEST(ExactPredicateStorage, DroppedCoordinateAndLegacyAxisMappingDoNotEnterTheProjection) {
  const auto bad = std::numeric_limits<double>::quiet_NaN();
  const auto large = std::numeric_limits<double>::max();
  const c::Vec3 a{bad, 0, 0}, b{large, 1, 0}, z{-large, 0, 1};
  // X is not consumed by the YZ predicate. Neither a NaN nor a huge dropped
  // component may become a new input rejection or influence its exact sign.
  const auto projected = e::Orient2D(a, b, z, 0);
  Same(projected, d::EvaluateOrient2D<Wide>(a, b, z, 0));
  EXPECT_TRUE(projected.valid); EXPECT_EQ(projected.value, 1);
  const c::Vec3 first{1, 2, 3}, second{-2, 5, 1}, third{3, -1, 4};
  for (int axis : {-1, 3, 99}) {
    Same(e::Orient2D(first, second, third, axis),
         d::EvaluateOrient2D<Wide>(first, second, third, axis));
    Same(e::Orient2D(first, second, third, axis),
         e::Orient2D(first, second, third, 1));
  }
}
TEST(ExactPredicateStorage, NullOutputAndUnsupportedNonfiniteFallbackKeepOriginalBehavior) {
  const c::Vec3 triangle[]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  EXPECT_FALSE(e::ClosestStratum({}, triangle, nullptr));
  EXPECT_FALSE(d::EvaluateClosestStratum<Wide>({}, triangle, nullptr));
  for (double bad : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    Compare({{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {bad, .5, 0}}}, false);
}
}  // namespace
