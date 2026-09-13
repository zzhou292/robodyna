// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <cstring>

namespace active_use_test {
TEST(SelfContactActiveUses, RemovedThickUseContributesNoFutureRadiusOrAreaAndNeverRedistributes) {
  Fixture fixture(2);
  c::SelfContactActiveUseBinding uses;
  ASSERT_EQ(uses.Initialize(fixture.facets).status, Code::Ok);
  auto base = fixture.Active(uses), current = base;
  std::size_t thick = SIZE_MAX, thin = SIZE_MAX;
  for (std::size_t i = 0; i < uses.vertices().size(); ++i) {
    const auto& feature = uses.vertices()[i];
    for (std::size_t a = feature.use_offset;
         a < std::size_t(feature.use_offset)+feature.use_count; ++a)
      for (std::size_t b = a+1;
           b < std::size_t(feature.use_offset)+feature.use_count; ++b) {
        const auto ha = uses.parents()[uses.vertex_uses()[a].parent].
            reference_half_thickness_m;
        const auto hb = uses.parents()[uses.vertex_uses()[b].parent].
            reference_half_thickness_m;
        if (ha != hb) {
          thick = ha > hb ? a : b;
          thin = ha > hb ? b : a;
        }
      }
    if (thick != SIZE_MAX) break;
  }
  ASSERT_NE(thick, SIZE_MAX);
  c::SelfContactResolvedVertexUse before_thick, before_thin;
  const c::SelfContactActivityView accepted{base.data(),current.data(),base.size()};
  ASSERT_EQ(uses.ResolveVertexUse(thick, accepted, &before_thick).status, Code::Ok);
  ASSERT_EQ(uses.ResolveVertexUse(thin, accepted, &before_thin).status, Code::Ok);
  ASSERT_TRUE(before_thick.active);
  current[uses.vertex_uses()[thick].parent] = 0;
  const c::SelfContactActivityView removed{base.data(),current.data(),base.size()};
  c::SelfContactResolvedVertexUse after_thick, after_thin;
  ASSERT_EQ(uses.ResolveVertexUse(thick, removed, &after_thick).status, Code::Ok);
  ASSERT_EQ(uses.ResolveVertexUse(thin, removed, &after_thin).status, Code::Ok);
  EXPECT_FALSE(after_thick.active);
  EXPECT_EQ(after_thick.reference_half_thickness_m, 0);
  EXPECT_EQ(after_thick.reference_area_m2.value, 0);
  EXPECT_EQ(after_thick.dual_area_m2.value, 0);
  EXPECT_EQ(after_thick.directed_vf_area_m2.value, 0);
  EXPECT_TRUE(after_thin.active);
  EXPECT_EQ(after_thin.reference_half_thickness_m,
      before_thin.reference_half_thickness_m);
  EXPECT_EQ(after_thin.reference_area_m2.value, before_thin.reference_area_m2.value);
  EXPECT_EQ(after_thin.directed_vf_area_m2.value,
      before_thin.directed_vf_area_m2.value);

  auto removed_base = current, illegal = removed_base;
  illegal[uses.vertex_uses()[thick].parent] = 1;
  c::SelfContactResolvedVertexUse held = after_thin;
  held.reference_half_thickness_m = 987;
  const auto bytes = held;
  EXPECT_EQ(uses.ResolveVertexUse(thin,
      {removed_base.data(),illegal.data(),removed_base.size()}, &held).status,
      Code::InvalidInput);
  EXPECT_EQ(std::memcmp(&held,&bytes,sizeof(held)), 0);
  ASSERT_EQ(uses.ResolveVertexUse(thin,
      {removed_base.data(),removed_base.data(),removed_base.size()}, &held).status,
      Code::Ok);
  EXPECT_TRUE(held.active);

  auto invalid = base;
  invalid[uses.vertex_uses()[thin].parent] = 2;
  held.reference_half_thickness_m = 654;
  const auto invalid_before = held;
  EXPECT_EQ(uses.ResolveVertexUse(thin,
      {base.data(),invalid.data(),base.size()},&held).status,Code::InvalidInput);
  EXPECT_EQ(std::memcmp(&held,&invalid_before,sizeof(held)),0);
  ASSERT_EQ(uses.ResolveVertexUse(thin,
      {base.data(),base.data(),base.size()},&held).status,Code::Ok);
}

TEST(SelfContactActiveUses, CoincidentIndependentSourceLayersRemainDistinctAndAdmitted) {
  Fixture fixture(2, false, true);
  c::SelfContactActiveUseBinding uses;
  ASSERT_EQ(uses.Initialize(fixture.facets).status, Code::Ok);
  const auto first = fixture.VertexUse(100, uses, 10);
  const auto second = fixture.VertexUse(101, uses, 20);
  ASSERT_NE(first, SIZE_MAX);
  ASSERT_NE(second, SIZE_MAX);
  EXPECT_FALSE(c::SameFacetVertexKey(uses.vertex_uses()[first].key,
      uses.vertex_uses()[second].key));
  const auto facet = fixture.RemoteFacet(101, uses.vertex_uses()[first].feature, uses);
  ASSERT_NE(facet, SIZE_MAX);
  const auto point = fixture.FacePoint(facet, uses);
  auto active = fixture.Active(uses);
  c::SelfContactPairClassification pair;
  ASSERT_EQ(uses.ClassifyVertexFace(first, facet, point,
      {active.data(),active.data(),active.size()}, &pair).status, Code::Ok);
  EXPECT_EQ(pair.status, c::SelfContactPairStatus::AdmittedVertexFace);
}
} // namespace active_use_test
