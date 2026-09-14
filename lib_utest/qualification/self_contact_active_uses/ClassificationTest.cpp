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

TEST(SelfContactActiveUses, SameParentRemoteVfNeedsAuthenticatedCurrentRegularity) {
  for (const unsigned level : {0u,2u}) {
    Fixture fixture(level);
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets).status,Code::Ok);
    const auto vertex = fixture.VertexUse(100,uses,11);
    ASSERT_NE(vertex,SIZE_MAX);
    const auto remote = fixture.RemoteFacet(
        100,uses.vertex_uses()[vertex].feature,uses);
    ASSERT_NE(remote,SIZE_MAX) << "level " << level;
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,remote,
        fixture.FacePoint(remote,uses),View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,
        c::SelfContactPairStatus::SameParentNeedsCurrentRegularity);
    EXPECT_FALSE(pair.local_incidence);
    EXPECT_FALSE(pair.excluded);
    EXPECT_EQ(pair.admitted_force_area_m2.value,0);
  }
}

TEST(SelfContactActiveUses, ActualPartAndPlainBodiesUseExactExecutionAuthority) {
  {
    ExecutionFixture fixture(2,ExecutionRigidMode::MergedParts);
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&fixture.rigid,{}}).status,Code::Ok);
    const auto vertex = fixture.VertexUse(100, uses, 10);
    const auto facet = fixture.RemoteFacet(300,uses.vertex_uses()[vertex].feature,uses);
    ASSERT_NE(facet,SIZE_MAX);
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::ExcludedSameRigidGroup);
  }
  {
    ExecutionFixture fixture(2,ExecutionRigidMode::PartAndPlain);
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&fixture.rigid,{}}).status,Code::Ok);
    const auto vertex = fixture.VertexUse(300,uses,20);
    const auto facet = fixture.RemoteFacet(fixture.second_plain_parent,
        uses.vertex_uses()[vertex].feature,uses);
    ASSERT_NE(facet,SIZE_MAX);
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::ExcludedSameRigidGroup);
    EXPECT_EQ(pair.endpoint_support[0].complete_rigid_group,1u);
  }
  {
    ExecutionFixture fixture(2,ExecutionRigidMode::SeparateParts);
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&fixture.rigid,{}}).status,Code::Ok);
    const auto vertex = fixture.VertexUse(100, uses, 10);
    const auto facet = fixture.RemoteFacet(300,uses.vertex_uses()[vertex].feature,uses);
    ASSERT_NE(facet,SIZE_MAX);
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);
    EXPECT_NE(pair.endpoint_support[0].complete_rigid_group,
        pair.endpoint_support[1].complete_rigid_group);
  }
  {
    ExecutionFixture fixture(2,ExecutionRigidMode::PartAndPlain);
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&fixture.rigid,{}}).status,Code::Ok);
    const auto vertex = fixture.VertexUse(100, uses, 10);
    const auto facet = fixture.RemoteFacet(300,uses.vertex_uses()[vertex].feature,uses);
    ASSERT_NE(facet,SIZE_MAX);
    auto active = fixture.Active(uses);
    c::SelfContactPairClassification pair;
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,fixture.FacePoint(facet,uses),
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);
    ASSERT_EQ(fixture.rigid.groups().size(),2u);
    EXPECT_EQ(fixture.rigid.groups()[0].source_id,
        fixture.rigid.groups()[1].source_id);
    EXPECT_NE(fixture.rigid.groups()[0].source_kind,
        fixture.rigid.groups()[1].source_kind);
  }
  {
    ExecutionFixture fixture(2,ExecutionRigidMode::SeparateParts);
    c::SelfContactActiveUseBinding uses;
    ASSERT_EQ(uses.Initialize(fixture.facets,{&fixture.rigid,{}}).status,Code::Ok);
    const auto vertex=fixture.VertexUse(300,uses,21);
    const auto facet=fixture.RemoteFacet(
        fixture.mixed_parent,uses.vertex_uses()[vertex].feature,uses);
    ASSERT_NE(vertex,SIZE_MAX);
    ASSERT_NE(facet,SIZE_MAX);
    auto active=fixture.Active(uses);
    c::SelfContactPairClassification pair;
    auto mixed=fixture.FacePoint(facet,uses);
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,mixed,
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);
    EXPECT_EQ(pair.endpoint_support[1].status,
        c::SelfContactSupportStatus::AdmittedPartialOrMixedRigid);
    auto partial=mixed;
    for (unsigned slot=0;slot<partial.count;++slot) {
      const auto source_id=fixture.domain.nodes()[partial.nodes[slot]].source_id;
      partial.weights[slot]=source_id == 20 ? 0 : .5;
    }
    ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,partial,
        View(active),&pair).status,Code::Ok);
    EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);
    EXPECT_EQ(pair.endpoint_support[1].status,
        c::SelfContactSupportStatus::AdmittedPartialOrMixedRigid);
  }
}

TEST(SelfContactActiveUses,
     StandaloneEeAreaRequiresAuthenticatedClosestPointCase) {
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
      c::SelfContactEdgeEdgeCase::BoundaryVertexEdgeMinimum,
      c::SelfContactEdgeEdgeCase::EdgeEdgeOnlyPenetrationOrCrossing,
      c::SelfContactEdgeEdgeCase::UnresolvedGeometricTie,
      c::SelfContactEdgeEdgeCase::CoplanarOverlap,
      c::SelfContactEdgeEdgeCase::ZeroDistance};
  c::SelfContactPairClassification pair;
  for (const auto edge_case : cases) {
    ASSERT_EQ(uses.ClassifyEdgeEdge(first,a,second,b,edge_case,View(active),
        &pair).status,Code::Ok);
    const bool admitted =
        edge_case == c::SelfContactEdgeEdgeCase::
                         StrictInteriorInteriorMinimum ||
        edge_case == c::SelfContactEdgeEdgeCase::
                         BoundaryVertexEdgeMinimum ||
        edge_case == c::SelfContactEdgeEdgeCase::ZeroDistance;
    EXPECT_EQ(pair.status, admitted
        ? c::SelfContactPairStatus::AdmittedEdgeEdge
        : c::SelfContactPairStatus::UnadmittedEdgeEdgeForceArea);
    if (admitted) {
      EXPECT_GT(pair.admitted_force_area_m2.lower,0);
      EXPECT_LE(pair.admitted_force_area_m2.lower,
                pair.admitted_force_area_m2.value);
      EXPECT_GE(pair.admitted_force_area_m2.upper,
                pair.admitted_force_area_m2.value);
    } else {
      EXPECT_EQ(pair.admitted_force_area_m2.value,0);
    }
  }

  const auto invented = fixture.FacePoint(
      uses.parents()[uses.edge_uses()[first].parent].facet_offset,uses);
  pair.status = c::SelfContactPairStatus::AdmittedVertexFace;
  EXPECT_EQ(uses.ClassifyEdgeEdge(
      first,invented,second,b,
      c::SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum,
      View(active),&pair).status,Code::InvalidInput);
  EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);
}

TEST(SelfContactActiveUses, SameParentRemoteEeAlsoNeedsCurrentRegularity) {
  Fixture fixture(2);
  c::SelfContactActiveUseBinding uses;
  ASSERT_EQ(uses.Initialize(fixture.facets).status,Code::Ok);
  const auto parent = fixture.Parent(100,uses);
  std::size_t first = SIZE_MAX, second = SIZE_MAX;
  for (std::size_t i = 0; i < uses.edge_uses().size(); ++i) {
    if (uses.edge_uses()[i].parent != parent) continue;
    for (std::size_t j = i+1; j < uses.edge_uses().size(); ++j) {
      if (uses.edge_uses()[j].parent != parent) continue;
      const auto& a = uses.edges()[uses.edge_uses()[i].feature];
      const auto& b = uses.edges()[uses.edge_uses()[j].feature];
      bool incident = false;
      for (const auto av : a.vertices) for (const auto bv : b.vertices)
        incident = incident || av == bv;
      if (!incident) { first = i; second = j; break; }
    }
    if (first != SIZE_MAX) break;
  }
  ASSERT_NE(first,SIZE_MAX);
  auto active = fixture.Active(uses);
  c::SelfContactPairClassification pair;
  ASSERT_EQ(uses.ClassifyEdgeEdge(first,Midpoint(uses.edge_uses()[first]),
      second,Midpoint(uses.edge_uses()[second]),
      c::SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum,
      View(active),&pair).status,Code::Ok);
  EXPECT_EQ(pair.status,
      c::SelfContactPairStatus::SameParentNeedsCurrentRegularity);
  EXPECT_EQ(pair.admitted_force_area_m2.value,0);
}
} // namespace active_use_test
