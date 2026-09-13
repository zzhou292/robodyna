// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <cfloat>
#include <cmath>
#include <limits>

namespace active_use_test {
namespace {
long double Norm(long double x, long double y, long double z) {
  return std::sqrt(x*x+y*y+z*z);
}
long double IndependentArea(const Fixture& f, const c::SelfContactParentUse& parent) {
  const auto point = [&](unsigned i) {
    return f.domain.nodes()[parent.nodes[i]].position;
  };
  const auto a = point(0), b = point(1), d = point(parent.arity == 4 ? 3 : 2);
  if (parent.arity == 3) {
    const long double ux = b.x-a.x, uy = b.y-a.y, uz = b.z-a.z;
    const long double vx = d.x-a.x, vy = d.y-a.y, vz = d.z-a.z;
    return .5L*Norm(uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx);
  }
  const auto c0 = point(2);
  const long double ux = .25L*(a.x-b.x-c0.x+d.x);
  const long double uy = .25L*(a.y-b.y-c0.y+d.y);
  const long double uz = .25L*(a.z-b.z-c0.z+d.z);
  const long double vx = .25L*(a.x+b.x-c0.x-d.x);
  const long double vy = .25L*(a.y+b.y-c0.y-d.y);
  const long double vz = .25L*(a.z+b.z-c0.z-d.z);
  return 4*Norm(uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx);
}
long double DirectedSum(const c::SelfContactActiveUseBinding& uses, std::size_t parent) {
  long double sum = 0;
  for (const auto& use : uses.vertex_uses())
    if (use.parent == parent) sum += use.directed_vf_area_m2.value;
  return sum;
}
void CheckAreas(unsigned level, bool warped) {
  Fixture fixture(level, warped);
  c::SelfContactActiveUseBinding uses;
  ASSERT_EQ(uses.Initialize(fixture.facets).status, Code::Ok);
  long double mixed_area = 0, mixed_directed = 0;
  for (std::size_t p = 0; p < uses.parents().size(); ++p) {
    const auto& parent = uses.parents()[p];
    const long double oracle = IndependentArea(fixture, parent);
    EXPECT_LE(parent.reference_area_m2.lower, oracle);
    EXPECT_GE(parent.reference_area_m2.upper, oracle);
    const long double directed = DirectedSum(uses, p);
    const long double scale = std::max(oracle, 1e-30L);
    const long double bound = 48*std::numeric_limits<double>::epsilon()*scale;
    EXPECT_LE(std::fabs(directed-.5L*oracle), bound)
        << parent.source.source_parent_id;
    long double dual = 0;
    std::size_t valence = 0;
    for (const auto& use : uses.vertex_uses()) if (use.parent == p) {
      dual += use.dual_area_m2.value;
      valence += use.facet_valence;
      EXPECT_EQ(use.directed_vf_area_m2.value, use.dual_area_m2.value/2);
      EXPECT_LE(use.directed_vf_area_m2.lower, use.directed_vf_area_m2.value);
      EXPECT_GE(use.directed_vf_area_m2.upper, use.directed_vf_area_m2.value);
    }
    EXPECT_EQ(valence, 3*parent.facet_count);
    EXPECT_LE(std::fabs(dual-oracle), 2*bound);
    EXPECT_EQ(parent.area_model, parent.arity == 4 ?
        c::SelfContactReferenceAreaModel::Q4CenterAreaContactModel :
        c::SelfContactReferenceAreaModel::T3CertifiedNativeArea);
    mixed_area += oracle;
    mixed_directed += directed;
  }
  EXPECT_LE(std::fabs(mixed_directed-.5L*mixed_area),
      64*std::numeric_limits<double>::epsilon()*mixed_area);
}
}

TEST(SelfContactActiveUses, CertifiedNativeAndCenterAreasGiveExactParentDualSums) {
  for (unsigned level = 0; level <= 2; ++level) {
    CheckAreas(level, false);
    CheckAreas(level, true);
  }
  EXPECT_STREQ(c::Q4CenterAreaContactModel, "center-area-uniform-natural-v1");
  EXPECT_STREQ(c::SymmetricDirectedVertexDualReferenceV1,
      "SymmetricDirectedVertexDualReferenceV1");
}

TEST(SelfContactActiveUses, NineLevelCombinationsRemainBidirectionallyHalfArea) {
  constexpr long double pressure = 1234567.890123456789L;
  for (unsigned first_level = 0; first_level <= 2; ++first_level) {
    Fixture first(first_level, true);
    c::SelfContactActiveUseBinding a;
    ASSERT_EQ(a.Initialize(first.facets).status, Code::Ok);
    const auto pa = first.Parent(100, a);
    for (unsigned second_level = 0; second_level <= 2; ++second_level) {
      Fixture second(second_level, false);
      c::SelfContactActiveUseBinding b;
      ASSERT_EQ(b.Initialize(second.facets).status, Code::Ok);
      const auto pb = second.Parent(200, b);
      const long double area_a = IndependentArea(first, a.parents()[pa]);
      const long double area_b = IndependentArea(second, b.parents()[pb]);
      const long double represented = DirectedSum(a, pa)+DirectedSum(b, pb);
      const long double expected = .5L*(area_a+area_b);
      const long double tolerance =
          64*std::numeric_limits<double>::epsilon()*(area_a+area_b);
      EXPECT_LE(std::fabs(represented-expected), tolerance);
      // Uniform pressure and summed directed event resultants are the same
      // identity; no implementation force or penalty call is used here.
      EXPECT_LE(std::fabs(pressure*represented-pressure*expected),
          pressure*tolerance);
    }
  }
}
} // namespace active_use_test
