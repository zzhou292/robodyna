// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <cmath>
#include <cstring>

namespace current_regularity_test {
namespace {

c::WeightedSurfacePoint FacePoint(
    Fixture& fixture,std::size_t facet) {
  return fixture.source.FacePoint(facet,fixture.uses);
}

std::size_t RemoteOwnFacet(
    Fixture& fixture,std::size_t parent,std::size_t vertex_use) {
  return fixture.source.RemoteFacet(
      fixture.uses.parents()[parent].source.source_parent_id,
      fixture.uses.vertex_uses()[vertex_use].feature,fixture.uses);
}

} // namespace

TEST(SelfContactCurrentRegularity,
    BaseActiveRemovalIsCertifiedAndLongInactiveGeometryIsExplicitlySkipped) {
  {
    Fixture fixture(2);
    const auto parent=fixture.FirstParent(4);
    fixture.Only(parent,true);
    fixture.SetParent(parent,SaddleQ4);
    fixture.InitializeRegularity();
    c::SelfContactCurrentRegularityReceipt receipt;
    ASSERT_EQ(fixture.regularity.Certify(
        fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
    const auto& result=fixture.Result(parent);
    EXPECT_EQ(result.state,c::SelfContactCurrentParentState::Removing);
    EXPECT_TRUE(result.geometry_evaluated);
    EXPECT_EQ(result.facets_evaluated,result.facet_count);
    const auto summary=fixture.regularity.results().summary;
    EXPECT_EQ(summary.removing_parents,1u);
    EXPECT_EQ(summary.active_parents,0u);
  }
  {
    Fixture fixture(2);
    std::fill(fixture.base.begin(),fixture.base.end(),0);
    fixture.current=fixture.base;
    std::fill(fixture.positions.begin(),fixture.positions.end(),
        std::numeric_limits<double>::quiet_NaN());
    fixture.InitializeRegularity();
    c::SelfContactCurrentRegularityReceipt receipt;
    ASSERT_EQ(fixture.regularity.Certify(
        fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
    const auto view=fixture.regularity.results();
    ASSERT_TRUE(view.complete);
    EXPECT_EQ(view.summary.skipped_parents,view.count);
    EXPECT_EQ(view.summary.certified_parents,0u);
    EXPECT_EQ(view.summary.facets_evaluated,0u);
    for(std::size_t p=0;p<view.count;++p) {
      EXPECT_EQ(view.data[p].state,
          c::SelfContactCurrentParentState::LongInactiveSkipped);
      EXPECT_EQ(view.data[p].chart,
          c::SelfContactCurrentChartStatus::SkippedLongInactive);
      EXPECT_FALSE(view.data[p].geometry_evaluated);
      EXPECT_EQ(view.data[p].facets_evaluated,0u);
      EXPECT_EQ(view.data[p].approximation.total_error_upper_m,0);
    }
    const auto parent=fixture.FirstParent(4);
    const auto eid=fixture.uses.parents()[parent].source.source_parent_id;
    const auto vertex=fixture.source.VertexUse(eid,fixture.uses);
    const auto facet=RemoteOwnFacet(fixture,parent,vertex);
    c::SelfContactPairClassification inactive;
    ASSERT_EQ(fixture.uses.ClassifyVertexFace(
        vertex,facet,FacePoint(fixture,facet),fixture.Activity(),
        &inactive).status,c::SelfContactActiveUseStatus::Ok);
    ASSERT_EQ(inactive.status,c::SelfContactPairStatus::InactiveParent);
    inactive.status=
        c::SelfContactPairStatus::SameParentNeedsCurrentRegularity;
    inactive.active[0]=inactive.active[1]=true;
    c::SelfContactPairClassification output;
    EXPECT_EQ(fixture.regularity.ExcludeCertifiedOwnParent(
        inactive,receipt,&output).status,Status::ReceiptMismatch);
  }
}

TEST(SelfContactCurrentRegularity,
    InvalidActivityAndReactivationPreserveCompleteQueryAndReceipt) {
  Fixture fixture(1);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  const auto before=fixture.regularity.results();
  const auto generation=receipt.generation();

  fixture.current[1]=2;
  EXPECT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,
      Status::InvalidInput);
  EXPECT_EQ(receipt.generation(),generation);
  EXPECT_EQ(fixture.regularity.results().data,before.data);
  fixture.current[1]=1;
  fixture.base[1]=0;
  EXPECT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,
      Status::InvalidInput);
  EXPECT_EQ(receipt.generation(),generation);
  EXPECT_EQ(fixture.regularity.results().data,before.data);
}

TEST(SelfContactCurrentRegularity,
    ExactFreshReceiptAloneExcludesCertifiedOwnParentWithZeroArea) {
  Fixture fixture(2);
  fixture.InitializeRegularity();
  const auto parent=fixture.FirstParent(4);
  const auto eid=fixture.uses.parents()[parent].source.source_parent_id;
  const auto vertex=fixture.source.VertexUse(eid,fixture.uses);
  ASSERT_NE(vertex,SIZE_MAX);
  const auto own_facet=RemoteOwnFacet(fixture,parent,vertex);
  ASSERT_NE(own_facet,SIZE_MAX);
  c::SelfContactPairClassification own;
  ASSERT_EQ(fixture.uses.ClassifyVertexFace(
      vertex,own_facet,FacePoint(fixture,own_facet),
      fixture.Activity(),&own).status,
      c::SelfContactActiveUseStatus::Ok);
  ASSERT_EQ(own.status,
      c::SelfContactPairStatus::SameParentNeedsCurrentRegularity);

  c::SelfContactCurrentRegularityReceipt first;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&first).status,Status::Ok);
  EXPECT_TRUE(first.MatchesInputs(
      fixture.Positions(),fixture.Activity()));
  c::SelfContactPairClassification excluded;
  ASSERT_EQ(fixture.regularity.ExcludeCertifiedOwnParent(
      own,first,&excluded).status,Status::Ok);
  EXPECT_EQ(excluded.status,
      c::SelfContactPairStatus::ExcludedRegularOwnParent);
  EXPECT_TRUE(excluded.excluded);
  EXPECT_EQ(excluded.candidate_directed_area_m2.value,0);
  EXPECT_EQ(excluded.admitted_force_area_m2.value,0);

  c::SelfContactCurrentRegularityReceipt second;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&second).status,Status::Ok);
  const auto held=excluded;
  EXPECT_EQ(fixture.regularity.ExcludeCertifiedOwnParent(
      own,first,&excluded).status,Status::ReceiptMismatch);
  EXPECT_EQ(std::memcmp(&excluded,&held,sizeof(held)),0);

  c::SelfContactCurrentRegularity foreign;
  ASSERT_EQ(foreign.Initialize(fixture.uses).status,Status::Ok);
  c::SelfContactCurrentRegularityReceipt foreign_receipt;
  ASSERT_EQ(foreign.Certify(
      fixture.Positions(),fixture.Activity(),&foreign_receipt).status,
      Status::Ok);
  EXPECT_EQ(fixture.regularity.ExcludeCertifiedOwnParent(
      own,foreign_receipt,&excluded).status,Status::ReceiptMismatch);
  EXPECT_EQ(std::memcmp(&excluded,&held,sizeof(held)),0);

  auto different_activity=fixture.current;
  c::SelfContactPairClassification wrong_activity;
  ASSERT_EQ(fixture.uses.ClassifyVertexFace(
      vertex,own_facet,FacePoint(fixture,own_facet),
      {different_activity.data(),different_activity.data(),
       different_activity.size()},&wrong_activity).status,
      c::SelfContactActiveUseStatus::Ok);
  EXPECT_EQ(fixture.regularity.ExcludeCertifiedOwnParent(
      wrong_activity,second,&excluded).status,Status::ReceiptMismatch);
  EXPECT_EQ(std::memcmp(&excluded,&held,sizeof(held)),0);

  auto wrong_parent=own;
  wrong_parent.parent[1]=(wrong_parent.parent[0]+1)%fixture.uses.parents().size();
  EXPECT_EQ(fixture.regularity.ExcludeCertifiedOwnParent(
      wrong_parent,second,&excluded).status,Status::InvalidPair);
  EXPECT_EQ(std::memcmp(&excluded,&held,sizeof(held)),0);
}

TEST(SelfContactCurrentRegularity,
    DifferentParentAndForeignBindingPairsAreNeverChangedByOwnParentReceipt) {
  Fixture fixture(2);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  const auto parent=fixture.FirstParent(4);
  const auto eid=fixture.uses.parents()[parent].source.source_parent_id;
  const auto vertex=fixture.source.VertexUse(eid,fixture.uses);
  std::size_t other=parent==0?1:0;
  const auto remote=fixture.source.RemoteFacet(
      fixture.uses.parents()[other].source.source_parent_id,
      fixture.uses.vertex_uses()[vertex].feature,fixture.uses);
  ASSERT_NE(remote,SIZE_MAX);
  c::SelfContactPairClassification different;
  ASSERT_EQ(fixture.uses.ClassifyVertexFace(
      vertex,remote,FacePoint(fixture,remote),fixture.Activity(),
      &different).status,c::SelfContactActiveUseStatus::Ok);
  ASSERT_NE(different.parent[0],different.parent[1]);
  const auto different_before=different;
  c::SelfContactPairClassification output;
  output.feature[0]=987;
  const auto held=output;
  EXPECT_EQ(fixture.regularity.ExcludeCertifiedOwnParent(
      different,receipt,&output).status,Status::InvalidPair);
  EXPECT_EQ(std::memcmp(&output,&held,sizeof(held)),0);
  EXPECT_EQ(std::memcmp(&different,&different_before,sizeof(different)),0);

  Fixture other_fixture(2);
  const auto other_parent=other_fixture.FirstParent(4);
  const auto other_eid=
      other_fixture.uses.parents()[other_parent].source.source_parent_id;
  const auto other_vertex=
      other_fixture.source.VertexUse(other_eid,other_fixture.uses);
  const auto other_facet=
      RemoteOwnFacet(other_fixture,other_parent,other_vertex);
  c::SelfContactPairClassification foreign_pair;
  ASSERT_EQ(other_fixture.uses.ClassifyVertexFace(
      other_vertex,other_facet,FacePoint(other_fixture,other_facet),
      other_fixture.Activity(),&foreign_pair).status,
      c::SelfContactActiveUseStatus::Ok);
  EXPECT_EQ(fixture.regularity.ExcludeCertifiedOwnParent(
      foreign_pair,receipt,&output).status,Status::ReceiptMismatch);
  EXPECT_EQ(std::memcmp(&output,&held,sizeof(held)),0);
}

} // namespace current_regularity_test
