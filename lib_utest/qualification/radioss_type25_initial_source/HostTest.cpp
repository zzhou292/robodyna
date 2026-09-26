// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/collision/radioss_type25/search_startup/Internal.h"
#include <gtest/gtest.h>
#include <cstring>
namespace initial_source_test {
namespace {
std::uint64_t Bits(double x){std::uint64_t out;std::memcpy(&out,&x,sizeof(out));return out;}
struct SearchOutput {
  tl::util::HostArena output,scratch;search::Snapshot snapshot;
  explicit SearchOutput(const search::Input& in) {
    const auto f=search::PreflightComposed(in);
    if(f.status!=search::Status::Ok||!output.Initialize(f.output_bytes)||!scratch.Initialize(f.scratch_bytes))
      throw std::runtime_error("Owning composed search allocation rejected");
  }
};
InventoryInput OneFace() {
  InventoryInput p;p.positions={{{0,0,0}},{{1,0,0}},{{1,1,0}},{{0,1,0}},{{.4,.4,.02}},{{3,3,3}}};
  p.mains={{{1,2,3,4}}};p.types={1};p.secondary={1,2,3,4,5,6};p.main_nodes={1,2,3,4};
  p.codes.assign(6,0);p.skews.assign(6,0);p.coefficients={100};p.secondary_coefficients.assign(6,100);
  p.main_gap={.05};p.secondary_gap.assign(6,.1);p.corner_gaps={{{.05,.05,.05,.05}}};
  p.support={{{1,0}}};p.removed_nodes.resize(1);p.multiplier=double(.20f);p.global_gap=.15;return p;
}
}
TEST(InitialSourceHost, WholeNativeInventoryRetainsNegativeMainAndAppliesLiteralRowEligibility) {
  auto input=OneFace();const auto baseline=NativeInventory(input);ASSERT_EQ(baseline.pairs.size(),1u);
  EXPECT_EQ(baseline.pairs.front(),(std::array<int,2>{5,1}));
  input.coefficients[0]=-100.;const auto negative=NativeInventory(input);EXPECT_EQ(negative.pairs,baseline.pairs);
  input.removed_nodes[0]={5};EXPECT_TRUE(NativeInventory(input).pairs.empty());
  input.removed_nodes[0].clear();input.secondary_coefficients[4]=0.;EXPECT_TRUE(NativeInventory(input).pairs.empty());
}
TEST(InitialSourceHost, WholeNativeBucMutatesOnlyConsumedSolidCornerAndRetainsPartExclusion) {
  auto input=OneFace();input.types[0]=0;input.corner_gaps[0]={5.,2.,3.,4.};
  auto result=NativeInventory(input);EXPECT_EQ(Bits(result.corner_gaps[0][0]),Bits(1.));
  for(unsigned k=1;k<4;++k)EXPECT_EQ(Bits(result.corner_gaps[0][k]),Bits(input.corner_gaps[0][k]));
  // Literal raw eight-slot incidence, including repeated positions. No S6Z
  // orientation/scatter or physical reference enters this native packet.
  input.solids.push_back({400,9,{1,2,3,4,5,5,6,6}});input.support[0]={1,0};
  EXPECT_TRUE(NativeInventory(input).pairs.empty());
}
TEST(InitialSourceHost, WholeInventoryCrossesActual128LaneDefaultCohortBoundary) {
  auto input=OneFace();
  for(unsigned i=0;i<160;++i) {
    input.positions.push_back({.2+.003*double(i),.4,.02});input.secondary.push_back(int(input.positions.size()));
    input.codes.push_back(0);input.skews.push_back(0);input.secondary_coefficients.push_back(100.);input.secondary_gap.push_back(.1);
  }
  const auto result=NativeInventory(input);ASSERT_GT(result.pairs.size(),128u);
  EXPECT_EQ(result.pairs.size(),161u);
  for(std::size_t i=1;i<result.pairs.size();++i)EXPECT_LT(result.pairs[i-1][0],result.pairs[i][0]);
}
TEST(InitialSourceHost, NativePreparedMappingHasDefinedWarmAndColdRows) {
  std::vector<std::array<int,4>> rows{{7,2,7,0},{0,0,0,0},{2,1,2,0}};
  NativePreparedMain(8,rows);EXPECT_EQ(rows[0],(std::array<int,4>{7,2,7,1}));
  EXPECT_EQ(rows[1],(std::array<int,4>{0,0,0,0}));EXPECT_EQ(rows[2],(std::array<int,4>{2,1,2,1}));
}
TEST(InitialSourceHost, ExplicitCompletePopulationTiersMatchNativeAndRejectCrossings) {
  Fixture fixture;auto in=fixture.Search();in.native_population.policy=search::NativePopulationPolicy::CompleteModelMultiplierTier;
  in.contributors.native_auxiliary_nodes=SIZE_MAX;
  for(const auto limits:std::vector<std::array<std::size_t,2>>{{100,1500000},{1500001,2500000},{2500001,3000000}}) {
    SCOPED_TRACE(limits[0]);
    in.native_population.lower=limits[0];in.native_population.upper=limits[1];
    std::size_t operand=0;ASSERT_EQ(search::detail::ResolveContext(in,{},search::detail::Context::ComposedNoTied,operand).status,search::Status::Ok);
    double value=0;ASSERT_EQ(search::ResolveMultiplier(operand,&value),search::Status::Ok);
    EXPECT_EQ(Bits(value),Bits(type25_search_startup_test::OracleMultiplier(int(limits[0]))));
    EXPECT_EQ(Bits(value),Bits(type25_search_startup_test::OracleMultiplier(int(limits[1]))));
    EXPECT_EQ(search::Preflight(in).status,search::Status::UnsupportedProfile);
  }
  for(const auto bounds:std::vector<std::array<std::size_t,2>>{{1500000,1500001},{2500000,2500001},{0,100},{100,99}}) {
    in.native_population.lower=bounds[0];in.native_population.upper=bounds[1];std::size_t sentinel=987;
    EXPECT_NE(search::detail::ResolveContext(in,{},search::detail::Context::ComposedNoTied,sentinel).status,search::Status::Ok);
    EXPECT_EQ(sentinel,987u);
  }
}
TEST(InitialSourceHost, ComposedRangePreservesExactLegacyGeometryAndReportsUnavailableCount) {
  Fixture f;auto exact=f.Search();SearchOutput a(exact);
  ASSERT_EQ(search::Build(exact,{},a.output,a.scratch,&a.snapshot).status,search::Status::Ok);
  auto interval=exact;interval.native_population={search::NativePopulationPolicy::CompleteModelMultiplierTier,f.nodes.size(),1500000};
  interval.contributors.native_auxiliary_nodes=SIZE_MAX;SearchOutput b(interval);
  ASSERT_EQ(search::BuildComposedNoTied(interval,{},b.output,b.scratch,&b.snapshot).status,search::Status::Ok);
  EXPECT_FALSE(b.snapshot.native_model_nodes_exact);EXPECT_EQ(b.snapshot.native_model_nodes,0u);
  EXPECT_EQ(b.snapshot.native_population.lower,f.nodes.size());EXPECT_EQ(b.snapshot.native_population.upper,1500000u);
  EXPECT_EQ(Bits(a.snapshot.multiplier),Bits(b.snapshot.multiplier));EXPECT_EQ(Bits(a.snapshot.margin),Bits(b.snapshot.margin));
  ASSERT_EQ(a.snapshot.removal_count,b.snapshot.removal_count);
  for(std::size_t i=0;i<=f.mains.size();++i)EXPECT_EQ(a.snapshot.main_offsets[i],b.snapshot.main_offsets[i]);
  for(std::size_t i=0;i<a.snapshot.removal_count;++i){EXPECT_EQ(a.snapshot.removed_nodes[i],b.snapshot.removed_nodes[i]);EXPECT_EQ(a.snapshot.removed_mains[i],b.snapshot.removed_mains[i]);}
}
TEST(InitialSourceHost, UnknownSourcePhaseUnitsAndCensusRejectBeforeCudaOrOutputPublication) {
  Fixture f;auto in=f.Input();src::PreparedSource output;
  in.phase=src::Phase::Unspecified;EXPECT_EQ(src::PrepareSource(in,f.Limits(),output).status,src::Status::WrongPhase);
  in=f.Input();in.units.length_m=0;EXPECT_EQ(src::PrepareSource(in,f.Limits(),output).status,src::Status::InvalidInput);
  in=f.Input();in.contributors.other_interfaces=1;EXPECT_EQ(src::PrepareSource(in,f.Limits(),output).status,src::Status::InvalidInput);
  in=f.Input();in.controls.starter_workers=2;EXPECT_EQ(src::PrepareSource(in,f.Limits(),output).status,src::Status::UnsupportedProfile);
  EXPECT_FALSE(output.prepared());
}
}
