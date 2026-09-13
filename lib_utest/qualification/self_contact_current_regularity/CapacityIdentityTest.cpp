// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <cstring>

namespace current_regularity_test {

TEST(SelfContactCurrentRegularity,
    ExactParentFacetAndByteCapsRejectOneShortThenRetry) {
  Fixture fixture(2);
  const auto plan=c::SelfContactCurrentRegularity::Preflight(fixture.uses);
  ASSERT_EQ(plan.report.status,Status::Ok);
  EXPECT_EQ(plan.forecast.parents,fixture.uses.parents().size());
  EXPECT_EQ(plan.forecast.facets,fixture.uses.facet_uses().size());
  EXPECT_EQ(plan.forecast.publication_records,2*plan.forecast.parents);
  EXPECT_EQ(plan.forecast.facet_staging_records,plan.forecast.facets);

  c::SelfContactCurrentRegularityLimits parent_cap;
  parent_cap.max_parents=plan.forecast.parents-1;
  c::SelfContactCurrentRegularity rejected_parent;
  EXPECT_EQ(rejected_parent.Initialize(
      fixture.uses,parent_cap).status,Status::ResourceLimit);
  EXPECT_FALSE(rejected_parent.initialized());

  c::SelfContactCurrentRegularityLimits facet_cap;
  facet_cap.max_facets=plan.forecast.facets-1;
  c::SelfContactCurrentRegularity rejected_facet;
  EXPECT_EQ(rejected_facet.Initialize(
      fixture.uses,facet_cap).status,Status::ResourceLimit);
  EXPECT_FALSE(rejected_facet.initialized());

  c::SelfContactCurrentRegularityLimits bytes;
  bytes.max_host_bytes=plan.forecast.startup_payload_bytes-1;
  c::SelfContactCurrentRegularity exact;
  EXPECT_EQ(exact.Initialize(fixture.uses,bytes).status,
      Status::ResourceLimit);
  EXPECT_FALSE(exact.initialized());
  ++bytes.max_host_bytes;
  ASSERT_EQ(exact.Initialize(fixture.uses,bytes).status,Status::Ok);
  EXPECT_EQ(exact.forecast().arena_bytes,plan.forecast.arena_bytes);
  EXPECT_EQ(exact.Initialize(fixture.uses).status,
      Status::AlreadyInitialized);
}

TEST(SelfContactCurrentRegularity,
    QueryAliasesAndLateParentFailurePreserveCompletePublicationThenRetry) {
  Fixture fixture(2,true);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  const auto before=fixture.regularity.results();
  std::vector<c::SelfContactCurrentParentResult> bytes(
      before.data,before.data+before.count);
  const auto generation=receipt.generation();

  auto* receipt_alias=
      reinterpret_cast<c::SelfContactCurrentRegularityReceipt*>(
          fixture.current.data());
  EXPECT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),receipt_alias).status,
      Status::InvalidInput);
  EXPECT_EQ(fixture.regularity.results().data,before.data);

  const c::VectorView position_alias{
      reinterpret_cast<const double*>(before.data),
      static_cast<std::uint32_t>(fixture.source.domain.node_count()),3,1};
  EXPECT_EQ(fixture.regularity.Certify(
      position_alias,fixture.Activity(),&receipt).status,
      Status::InvalidInput);
  EXPECT_EQ(receipt.generation(),generation);
  EXPECT_EQ(fixture.regularity.results().data,before.data);

  const auto last=fixture.uses.parents().size()-1;
  fixture.Only(last);
  if(fixture.uses.parents()[last].arity==4) {
    const std::array<c::Vec3,4> bad{{
        {2,0,0},{0,0,0},{-2,0,0},{0,0,0}}};
    fixture.SetParent(last,bad);
  } else {
    const std::array<c::Vec3,3> bad{{
        {0,0,0},{1,0,0},{2,0,0}}};
    fixture.SetParent(last,bad);
  }
  const auto failed=fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt);
  EXPECT_NE(failed.status,Status::Ok);
  EXPECT_EQ(failed.parent,last);
  EXPECT_EQ(receipt.generation(),generation);
  const auto preserved=fixture.regularity.results();
  EXPECT_EQ(preserved.data,before.data);
  EXPECT_EQ(std::memcmp(
      preserved.data,bytes.data(),
      bytes.size()*sizeof(bytes[0])),0);

  if(fixture.uses.parents()[last].arity==4)
    fixture.SetParent(last,SkewQ4);
  else
    fixture.SetParent(last,FlatT3);
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  EXPECT_EQ(receipt.generation(),generation+1);
  EXPECT_NE(fixture.regularity.results().data,before.data);
}

TEST(SelfContactCurrentRegularity,
    PermutedSelectionPublishesIdenticalSourceEidOrderedSummary) {
  Fixture first(2,false,false);
  Fixture second(2,false,true);
  first.InitializeRegularity();
  second.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt a_receipt,b_receipt;
  ASSERT_EQ(first.regularity.Certify(
      first.Positions(),first.Activity(),&a_receipt).status,Status::Ok);
  ASSERT_EQ(second.regularity.Certify(
      second.Positions(),second.Activity(),&b_receipt).status,Status::Ok);
  const auto a=first.regularity.results();
  const auto b=second.regularity.results();
  ASSERT_TRUE(a.complete);
  ASSERT_TRUE(b.complete);
  ASSERT_EQ(a.count,b.count);
  for(std::size_t p=0;p<a.count;++p) {
    EXPECT_EQ(a.data[p].source_eid,b.data[p].source_eid);
    if(p) EXPECT_LT(a.data[p-1].source_eid,a.data[p].source_eid);
    EXPECT_EQ(a.data[p].arity,b.data[p].arity);
    EXPECT_EQ(a.data[p].facet_count,b.data[p].facet_count);
    EXPECT_EQ(a.data[p].facets_evaluated,b.data[p].facets_evaluated);
    EXPECT_EQ(a.data[p].current_area_enclosure_m2.lower,
        b.data[p].current_area_enclosure_m2.lower);
    EXPECT_EQ(a.data[p].current_area_enclosure_m2.upper,
        b.data[p].current_area_enclosure_m2.upper);
    EXPECT_EQ(a.data[p].minimum_scaled_jacobian_quality,
        b.data[p].minimum_scaled_jacobian_quality);
    EXPECT_EQ(a.data[p].approximation.total_error_upper_m,
        b.data[p].approximation.total_error_upper_m);
  }
  EXPECT_EQ(a.summary.parents,b.summary.parents);
  EXPECT_EQ(a.summary.facets,b.summary.facets);
  EXPECT_EQ(a.summary.certified_parents,b.summary.certified_parents);
  EXPECT_EQ(a.summary.facets_evaluated,b.summary.facets_evaluated);
}

TEST(SelfContactCurrentRegularity,
    UnpreparedAndWrongSizedInputsRejectWithoutPublication) {
  c::SelfContactActiveUseBinding absent;
  c::SelfContactCurrentRegularity regularity;
  EXPECT_EQ(regularity.Initialize(absent).status,Status::InvalidInput);
  EXPECT_FALSE(regularity.initialized());

  Fixture fixture(0);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  auto short_activity=fixture.Activity();
  --short_activity.parent_count;
  EXPECT_EQ(fixture.regularity.Certify(
      fixture.Positions(),short_activity,&receipt).status,
      Status::InvalidInput);
  EXPECT_FALSE(fixture.regularity.results().complete);
  auto wrong_positions=fixture.Positions();
  --wrong_positions.node_count;
  EXPECT_EQ(fixture.regularity.Certify(
      wrong_positions,fixture.Activity(),&receipt).status,
      Status::InvalidInput);
  EXPECT_FALSE(fixture.regularity.results().complete);
}

} // namespace current_regularity_test
