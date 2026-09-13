// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace active_use_test {
namespace {
c::SelfContactActivityView View(std::vector<std::uint8_t>& activity) {
  return {activity.data(),activity.data(),activity.size()};
}
c::WeightedSurfacePoint Midpoint(const c::SelfContactFacetEdgeUse& edge) {
  c::WeightedSurfacePoint point;
  point.count = edge.endpoints[0].count;
  for (unsigned i = 0; i < point.count; ++i) {
    point.nodes[i] = edge.endpoints[0].nodes[i];
    point.weights[i] = .5*edge.endpoints[0].weights[i] +
        .5*edge.endpoints[1].weights[i];
  }
  return point;
}
std::size_t EdgeUse(std::uint64_t eid, const Fixture& fixture,
    const c::SelfContactActiveUseBinding& uses) {
  const auto parent = fixture.Parent(eid, uses);
  for (std::size_t i = 0; i < uses.edge_uses().size(); ++i)
    if (uses.edge_uses()[i].parent == parent) return i;
  return SIZE_MAX;
}
}

TEST(SelfContactActiveUses, OnlyLocalVfIncidenceExcludesSamePidSharedCornerTopology) {
  Fixture fixture(2);
  c::SelfContactActiveUseBinding uses;
  ASSERT_EQ(uses.Initialize(fixture.facets).status, Code::Ok);
  const auto vertex = fixture.VertexUse(100, uses, 11);
  ASSERT_NE(vertex, SIZE_MAX);
  std::size_t local = SIZE_MAX;
  for (std::size_t i = 0; i < uses.facet_uses().size(); ++i)
    for (const auto feature : uses.facet_uses()[i].vertex_features)
      if (feature == uses.vertex_uses()[vertex].feature) { local = i; break; }
  ASSERT_NE(local, SIZE_MAX);
  auto active = fixture.Active(uses);
  c::SelfContactPairClassification pair;
  ASSERT_EQ(uses.ClassifyVertexFace(vertex, local, fixture.FacePoint(local, uses),
      View(active), &pair).status, Code::Ok);
  EXPECT_EQ(pair.status, c::SelfContactPairStatus::ExcludedLocalIncidence);
  EXPECT_TRUE(pair.excluded);

  const auto remote = fixture.RemoteFacet(200,
      uses.vertex_uses()[vertex].feature, uses);
  ASSERT_NE(remote, SIZE_MAX);
  ASSERT_EQ(uses.parents()[uses.vertex_uses()[vertex].parent].source.source_part_id,
      uses.parents()[uses.facet_uses()[remote].parent].source.source_part_id);
  ASSERT_EQ(uses.ClassifyVertexFace(vertex, remote,
      fixture.FacePoint(remote, uses), View(active), &pair).status, Code::Ok);
  EXPECT_EQ(pair.status, c::SelfContactPairStatus::AdmittedVertexFace);
  EXPECT_FALSE(pair.local_incidence);
  EXPECT_FALSE(pair.excluded);
}

TEST(SelfContactActiveUses, CompletePartAndPlainBodiesExcludeButDifferentPartialAndMixedAdmit) {
  {
    Fixture fixture(2);
    Rigid rigid(fixture, {{10,11,12,13,14}});
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&rigid.binding,{}}).status, Code::Ok);
    const auto vertex = fixture.VertexUse(100, uses, 10);
    const auto facet = fixture.RemoteFacet(200, uses.vertex_uses()[vertex].feature, uses);
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::ExcludedSameRigidGroup);
  }
  {
    Fixture fixture(2,false,true);
    Rigid rigid(fixture, {{20,21}}, {10,11,12,13,14});
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&rigid.binding,{}}).status, Code::Ok);
    const auto vertex = fixture.VertexUse(100, uses, 10);
    const auto facet = fixture.RemoteFacet(200, uses.vertex_uses()[vertex].feature, uses);
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::ExcludedSameRigidGroup);
    EXPECT_EQ(pair.endpoint_support[0].complete_rigid_group,1u);
  }
  {
    Fixture fixture(2,false,true);
    Rigid rigid(fixture, {{10,11,12,13,14},{20,21,22,23}});
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&rigid.binding,{}}).status, Code::Ok);
    const auto vertex = fixture.VertexUse(100, uses, 10);
    const auto facet = fixture.RemoteFacet(101, uses.vertex_uses()[vertex].feature, uses);
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);
    EXPECT_NE(pair.endpoint_support[0].complete_rigid_group,
        pair.endpoint_support[1].complete_rigid_group);
  }
  {
    Fixture fixture(2);
    Rigid rigid(fixture, {{10,11}});
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&rigid.binding,{}}).status, Code::Ok);
    const auto vertex = fixture.VertexUse(100, uses, 10);
    const auto facet = fixture.RemoteFacet(200, uses.vertex_uses()[vertex].feature, uses);
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);
    EXPECT_EQ(pair.endpoint_support[1].status,
        c::SelfContactSupportStatus::AdmittedPartialOrMixedRigid);
  }
  {
    Fixture fixture(2);
    Rigid rigid(fixture, {{10,11},{12,13}});
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&rigid.binding,{}}).status, Code::Ok);
    const auto vertex = fixture.VertexUse(200, uses, 14);
    const auto facet = fixture.RemoteFacet(100, uses.vertex_uses()[vertex].feature, uses);
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);
    EXPECT_EQ(pair.endpoint_support[1].status,
        c::SelfContactSupportStatus::AdmittedPartialOrMixedRigid);
  }
}

TEST(SelfContactActiveUses, EveryStandaloneEeCaseHasNoInventedForceArea) {
  Fixture fixture(2,false,true);
  c::SelfContactActiveUseBinding uses;
  ASSERT_EQ(uses.Initialize(fixture.facets).status, Code::Ok);
  const auto first = EdgeUse(100,fixture,uses), second = EdgeUse(101,fixture,uses);
  ASSERT_NE(first,SIZE_MAX); ASSERT_NE(second,SIZE_MAX);
  const auto a = Midpoint(uses.edge_uses()[first]);
  const auto b = Midpoint(uses.edge_uses()[second]);
  auto active = fixture.Active(uses);
  constexpr c::SelfContactEdgeEdgeCase cases[]{
      c::SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum,
      c::SelfContactEdgeEdgeCase::EdgeEdgeOnlyPenetrationOrCrossing,
      c::SelfContactEdgeEdgeCase::UnresolvedGeometricTie,
      c::SelfContactEdgeEdgeCase::CoplanarOverlap,
      c::SelfContactEdgeEdgeCase::ZeroDistance};
  c::SelfContactPairClassification pair;
  for (const auto edge_case : cases) {
    ASSERT_EQ(uses.ClassifyEdgeEdge(first,a,second,b,edge_case,View(active),
        nullptr,&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::UnadmittedEdgeEdgeForceArea);
    EXPECT_EQ(pair.admitted_force_area_m2.value,0);
  }
  const auto vertex = fixture.VertexUse(100,uses,10);
  const auto facet = fixture.RemoteFacet(101,uses.vertex_uses()[vertex].feature,uses);
  c::SelfContactPairClassification vf;
  ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
      View(active),&vf).status,Code::Ok);
  ASSERT_EQ(vf.status,c::SelfContactPairStatus::AdmittedVertexFace);
  std::size_t covered_first = SIZE_MAX;
  const auto first_parent = fixture.Parent(100,uses);
  for (std::size_t i = 0; i < uses.edge_uses().size(); ++i) {
    const auto& edge = uses.edge_uses()[i];
    const auto& feature = uses.edges()[edge.feature];
    if (edge.parent == first_parent &&
        (feature.vertices[0] == uses.vertex_uses()[vertex].feature ||
         feature.vertices[1] == uses.vertex_uses()[vertex].feature)) {
      covered_first = i;
      break;
    }
  }
  ASSERT_NE(covered_first,SIZE_MAX);
  const auto covered_second = uses.facet_uses()[facet].edge_uses[0];
  const auto covered_a = Midpoint(uses.edge_uses()[covered_first]);
  const auto covered_b = Midpoint(uses.edge_uses()[covered_second]);
  auto unauthenticated = vf;
  unauthenticated.binding_identity = nullptr;
  ASSERT_EQ(uses.ClassifyEdgeEdge(covered_first,covered_a,covered_second,covered_b,
      cases[0],View(active),&unauthenticated,&pair).status,Code::Ok);
  EXPECT_EQ(pair.status,c::SelfContactPairStatus::UnadmittedEdgeEdgeForceArea);
  ASSERT_EQ(uses.ClassifyEdgeEdge(covered_first,covered_a,covered_second,covered_b,
      cases[0],View(active),
      &vf,&pair).status,Code::Ok);
  EXPECT_EQ(pair.status,
      c::SelfContactPairStatus::CoveredByIndependentAdmittedVertexFace);
  EXPECT_EQ(pair.admitted_force_area_m2.value,0);
}
} // namespace active_use_test
