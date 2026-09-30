// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <cfloat>
#include <cstring>

namespace current_regularity_test {
namespace {

void ExpectPreserved(
    Fixture& fixture, const std::array<c::Vec3,4>& invalid,
    const c::SelfContactCurrentParentResult* published,
    const std::vector<c::SelfContactCurrentParentResult>& before,
    c::SelfContactCurrentRegularityReceipt& receipt) {
  const auto parent=fixture.FirstParent(4);
  fixture.SetParent(parent,invalid);
  const auto generation=receipt.generation();
  const auto report=fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt);
  EXPECT_NE(report.status,Status::Ok);
  EXPECT_EQ(receipt.generation(),generation);
  const auto view=fixture.regularity.results();
  ASSERT_TRUE(view.complete);
  EXPECT_EQ(view.data,published);
  ASSERT_EQ(view.count,before.size());
  EXPECT_EQ(std::memcmp(
      view.data,before.data(),view.count*sizeof(before[0])),0);
}

} // namespace

TEST(SelfContactCurrentRegularity,
    InvertedFoldedAndNearDegenerateQ4RejectWithoutReplacingPublication) {
  Fixture fixture(2);
  const auto parent=fixture.FirstParent(4);
  fixture.Only(parent);
  fixture.SetParent(parent,FlatQ4);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  const auto initial=fixture.regularity.results();
  std::vector<c::SelfContactCurrentParentResult> before(
      initial.data,initial.data+initial.count);

  const std::array<c::Vec3,4> inverted{{
      {1,1,0},{-1,-1,0},{-1,1,0},{1,-1,0}}};
  ExpectPreserved(fixture,inverted,initial.data,before,receipt);

  const std::array<c::Vec3,4> folded{{
      {1,1,0},{-1,1,0},{.75,0,0},{1,-1,0}}};
  ExpectPreserved(fixture,folded,initial.data,before,receipt);

  const double epsilon=DBL_EPSILON/4;
  const std::array<c::Vec3,4> near_degenerate{{
      {1,epsilon,0},{-1,epsilon,0},
      {-1,-epsilon,0},{1,-epsilon,0}}};
  ExpectPreserved(fixture,near_degenerate,initial.data,before,receipt);

  fixture.SetParent(parent,SkewQ4);
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  EXPECT_EQ(receipt.generation(),2u);
  EXPECT_NE(fixture.regularity.results().data,initial.data);
}

TEST(SelfContactCurrentRegularity,
    ZeroApproximationDoesNotAdmitDegenerateCurrentParent) {
  Fixture fixture(2);
  const auto parent=fixture.FirstParent(4);
  fixture.Only(parent);
  const std::array<c::Vec3,4> collinear{{
      {2,0,0},{0,0,0},{-2,0,0},{0,0,0}}};
  fixture.SetParent(parent,collinear);
  c::FacetApproximationBound approximation;
  ASSERT_EQ(fixture.source.facets.Approximation(
      fixture.uses.parents()[parent].surface_parent,
      fixture.Positions(),&approximation),c::Status::kOk);
  EXPECT_EQ(approximation.bilinear_error_upper_m,0);
  EXPECT_EQ(approximation.total_error_upper_m,0);
  fixture.InitializeRegularity();
  c::SelfContactCurrentRegularityReceipt receipt;
  const auto report=fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt);
  EXPECT_EQ(report.status,Status::ParentGeometryUnresolved);
  EXPECT_FALSE(receipt.prepared());
  EXPECT_FALSE(fixture.regularity.results().complete);

  fixture.SetParent(parent,FlatQ4);
  ASSERT_EQ(fixture.regularity.Certify(
      fixture.Positions(),fixture.Activity(),&receipt).status,Status::Ok);
  EXPECT_TRUE(receipt.prepared());
}

} // namespace current_regularity_test
