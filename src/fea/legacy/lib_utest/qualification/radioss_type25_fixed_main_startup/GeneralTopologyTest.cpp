// SPDX-License-Identifier: AGPL-3.0-or-later
#include "GeneralCases.h"
#include "Assertions.h"
#include "SelectorOracle.h"
#include "lib_src/collision/radioss_type25/startup/NeighborGeometry.h"
#include <limits>
namespace type25_startup_test {
namespace {
std::uint64_t DoubleBits(double x){std::uint64_t bits;std::memcpy(&bits,&x,sizeof(bits));return bits;}
std::vector<unsigned char> Bytes(const tl::util::HostArena& a) {
  const auto* p=static_cast<const unsigned char*>(a.data());return {p,p+a.bytes()};
}
void SameWarnings(const GeneralBuilt& built,const NativeResult& native,const Case& source) {
  EXPECT_EQ(built.report.neighbor_warnings.count,std::size_t(native.warning_count));
  if(native.warning_count) {
    const auto& w=built.report.neighbor_warnings;ASSERT_LT(w.first_main,built.startup.main_count);ASSERT_LT(w.first_edge,4u);
    const auto& main=built.startup.mains[w.first_main];
    EXPECT_EQ(source.ids[main.nodes[w.first_edge]],std::uint64_t(native.warning_node_ids[0]));
    EXPECT_EQ(source.ids[main.nodes[(w.first_edge+1)%4]],std::uint64_t(native.warning_node_ids[1]));
  }
}
void Compare(const Case& source) {
  const auto input=GeneralInput(source);const GeneralBuilt built(source);
  const auto expected=Oracle(input,source.coefficients.data(),source.coefficients.size());
  SameStarter(built.startup,expected);SameWarnings(built,expected,source);
  EXPECT_EQ(built.startup.profile,input.profile);EXPECT_EQ(built.startup.topology,input.topology);
  EXPECT_EQ(built.startup.main_count,2*source.primary.size());
  for(std::size_t m=0;m<built.startup.main_count;++m)for(unsigned k=0;k<4;++k) {
    const auto& a=built.startup.mains[m];if(!a.neighbors[k])continue;
    const auto& b=built.startup.mains[a.neighbors[k]-1];const auto e=unsigned(a.neighbor_edges[k]-1);
    ASSERT_LT(e,4u);EXPECT_EQ(b.neighbors[e],int(m+1));EXPECT_EQ(b.neighbor_edges[e],int(k+1));
  }
}
NativeSelection CompareSelector(const Case& source,const std::vector<int>& ids) {
  namespace ng=s::detail::neighbor_geometry;
  const auto expected=SelectorOracle(source,1,0,ids);
  std::vector<s::Main> mains(source.primary.size());std::vector<n::Vector> points(source.ids.size());
  for(std::size_t i=0;i<mains.size();++i)for(unsigned j=0;j<4;++j)mains[i].nodes[j]=source.primary[i].nodes[j];
  const double length=source.units==s::Coordinates::Si?source.scale.length_m:1.;
  for(std::size_t i=0;i<points.size();++i)points[i]={source.positions[3*i]/length,source.positions[3*i+1]/length,source.positions[3*i+2]/length};
  std::vector<double> angles(ids.size()),sides(ids.size());
  EXPECT_TRUE(ng::Scores(mains[0],mains.data(),points.data(),mains[0].nodes[0],mains[0].nodes[1],ids.data(),ids.size(),angles.data(),sides.data()));
  EXPECT_EQ(expected.angles.size(),angles.size());
  for(std::size_t i=0;i<ids.size();++i){EXPECT_EQ(DoubleBits(angles[i]),DoubleBits(expected.angles[i]));EXPECT_EQ(DoubleBits(sides[i]),DoubleBits(expected.sides[i]));}
  const auto winner=ng::Winner(angles.data(),sides.data(),ids.size());
  EXPECT_EQ(winner==SIZE_MAX?0:ids[winner],expected.winner);EXPECT_EQ(expected.calls,1);
  EXPECT_EQ(expected.warning,winner==SIZE_MAX?11:0);EXPECT_EQ(DoubleBits(ng::Em20),DoubleBits(expected.em20));return expected;
}
}
TEST(Type25GeneralStartup, ValenceThreeFourAndWindingPreserveCompleteNativeStages) {
  for(unsigned valence:{3u,4u})for(unsigned mode=0;mode<3;++mode)for(bool winding:{false,true}) {
    SCOPED_TRACE(valence);
    SCOPED_TRACE(mode);
    SCOPED_TRACE(winding);
    auto source=EdgeStar(valence,mode,winding);Compare(source);
    const auto native=Oracle(GeneralInput(source),source.coefficients.data(),source.coefficients.size());
    EXPECT_GT(native.selector_calls,0);
    std::reverse(source.primary.begin(),source.primary.end());Compare(source);
  }
}
TEST(Type25GeneralStartup, OrderedExtraVertexAndMultipleEdgeRulesMatchNative) {
  for(bool tri:{false,true}) {
    auto source=SharedExtraVertex(tri);Compare(source);
    std::reverse(source.primary.begin(),source.primary.end());Compare(source);
  }
  // Both primary shared edges have the same input direction; generated opposite
  // sides supply actual native reversed candidates, without rewinding a source face.
  Compare(EdgeStar(2));
}
TEST(Type25GeneralStartup, DisconnectedReferencesStayBoundToOriginalNodeWithoutWelding) {
  auto source=DisconnectedFan();GeneralBuilt built(source);
  const auto expected=Oracle(GeneralInput(source),source.coefficients.data(),source.coefficients.size());SameStarter(built.startup,expected);
  std::vector<int> refs;
  for(std::size_t m=0;m<built.startup.main_count;++m)for(unsigned k=0;k<3;++k)
    if(built.startup.mains[m].nodes[k]==0)refs.push_back(built.startup.mains[m].normal_reference[k]);
  std::sort(refs.begin(),refs.end());refs.erase(std::unique(refs.begin(),refs.end()),refs.end());EXPECT_EQ(refs.size(),4u);
  auto legacy=source.Input();EXPECT_EQ(s::BuildStarter(legacy,{},built.output,built.scratch,nullptr).status,s::Status::InvalidInput);
  tl::util::HostArena out,scratch;const auto f=s::Preflight(legacy);ASSERT_TRUE(out.Initialize(f.output_bytes));ASSERT_TRUE(scratch.Initialize(f.scratch_bytes));
  s::Snapshot rejected;EXPECT_EQ(s::BuildStarter(legacy,{},out,scratch,&rejected).status,s::Status::UnsupportedTopology);
}
TEST(Type25GeneralStartup, DoubleScoresAndFirstTieWinnerAreOriginalNativeValues) {
  auto source=EdgeStar(3);
  for(unsigned i:{1u,2u}){std::swap(source.primary[i].nodes[0],source.primary[i].nodes[1]);std::swap(source.primary[i].nodes[2],source.primary[i].nodes[3]);}
  for(unsigned k:{2u,3u})for(unsigned axis=0;axis<3;++axis)
    source.positions[3*source.primary[2].nodes[k]+axis]=source.positions[3*source.primary[1].nodes[k]+axis];
  const auto first=CompareSelector(source,{2,3});const auto reversed=CompareSelector(source,{3,2});
  ASSERT_EQ(DoubleBits(first.angles[0]),DoubleBits(first.angles[1]));ASSERT_EQ(DoubleBits(first.sides[0]),DoubleBits(first.sides[1]));
  EXPECT_EQ(first.winner,2);EXPECT_EQ(reversed.winner,3);
  const auto slot=3*source.primary[2].nodes[2]+1;const auto original=source.positions[slot];
  for(double next:{std::nextafter(original,-std::numeric_limits<double>::infinity()),std::nextafter(original,std::numeric_limits<double>::infinity())}) {
    source.positions[slot]=next;CompareSelector(source,{2,3});
  }
}
TEST(Type25GeneralStartup, NativeDoubleFloorsSiAndWarpedNormalsRetainBits) {
  for(unsigned change=0;change<5;++change) {
    SCOPED_TRACE(change);auto source=EdgeStar(4,2,true);
    for(std::size_t i=0;i<source.ids.size();++i) {
      auto& x=source.positions[3*i];auto& y=source.positions[3*i+1];auto& z=source.positions[3*i+2];
      if(change==0){const auto old=x;x=.8*x-.6*z;z=.6*old+.8*z;}
      if(change==1)z+=.125*x*y;
      if(change==2){x*=1e-12;y*=1e-12;z*=1e-12;}
      if(change==3){x*=.001;y*=.001;z*=.001;}
      if(change==4){if(x==0)x=-0.;if(y==0)y=-0.;if(z==0)z=-0.;}
    }
    if(change==3)source.units=s::Coordinates::Si;
    Compare(source);
    // Use the original direct selector too: it observes double topology scores,
    // separate from complete startup's REAL4 output comparisons above.
    CompareSelector(source,{2,3,4});
  }
}
TEST(Type25GeneralStartup, LegacyForecastOutputAndUnqualifiedFixedReadyRemainExplicit) {
  auto source=Grid(3,2,2);Built legacy(source);const GeneralBuilt general(source);
  const auto native=Oracle(source.Input(),source.coefficients.data(),source.coefficients.size());Same(legacy,native);SameStarter(general.startup,native);
  const auto a=s::Preflight(source.ids.size(),source.primary.size(),{}),b=s::Preflight(source.Input());
  EXPECT_EQ(a.output_bytes,b.output_bytes);EXPECT_EQ(a.scratch_bytes,b.scratch_bytes);EXPECT_EQ(a.ready_output_bytes,b.ready_output_bytes);EXPECT_EQ(a.ready_scratch_bytes,b.ready_scratch_bytes);
  EXPECT_GT(general.forecast.scratch_bytes,a.scratch_bytes);EXPECT_EQ(general.forecast.output_bytes,a.output_bytes);
  EXPECT_EQ(general.forecast.ready_output_bytes,0u);EXPECT_EQ(general.forecast.ready_scratch_bytes,0u);
  auto input=GeneralInput(source);input.profile=s::Profile::OrdinaryExteriorFixedMain;
  s::FixedMainView preserved=legacy.ready;const auto bytes=Bytes(legacy.ready_output);
  EXPECT_EQ(s::BuildFixedMain(input,general.startup,{source.coefficients.data(),source.coefficients.size()},
      {},legacy.ready_output,legacy.ready_scratch,&preserved).status,s::Status::UnsupportedProfile);
  EXPECT_EQ(std::memcmp(bytes.data(),legacy.ready_output.data(),bytes.size()),0);EXPECT_EQ(preserved.normals.face_normals,legacy.ready.normals.face_normals);
}
TEST(Type25GeneralStartup, CapacityAliasNonfiniteAndRetryPreservePublishedOutput) {
  auto source=EdgeStar(4);GeneralBuilt built(source);const auto before=Bytes(built.output);const auto old=built.startup;
  auto in=GeneralInput(source);auto cap=s::Limits{};cap.max_scratch_bytes=built.forecast.scratch_bytes-1;
  EXPECT_EQ(s::BuildStarter(in,cap,built.output,built.scratch,&built.startup).status,s::Status::ResourceLimit);
  cap={};cap.max_output_bytes=built.forecast.output_bytes-1;
  EXPECT_EQ(s::BuildStarter(in,cap,built.output,built.scratch,&built.startup).status,s::Status::ResourceLimit);
  EXPECT_EQ(s::BuildStarter(in,{},built.scratch,built.scratch,&built.startup).status,s::Status::InvalidInput);
  in.topology=static_cast<s::TopologyPolicy>(99);EXPECT_EQ(s::Preflight(in).status,s::Status::UnsupportedProfile);
  EXPECT_EQ(s::BuildStarter(in,{},built.output,built.scratch,&built.startup).status,s::Status::UnsupportedProfile);
  in=GeneralInput(source);const auto good=source.positions;for(auto& x:source.positions)x*=1e150;
  EXPECT_EQ(s::BuildStarter(in,{},built.output,built.scratch,&built.startup).status,s::Status::NonfiniteResult);
  EXPECT_EQ(std::memcmp(before.data(),built.output.data(),before.size()),0);EXPECT_EQ(built.startup.mains,old.mains);
  source.positions=good;in=GeneralInput(source);cap={};cap.max_output_bytes=built.forecast.output_bytes;cap.max_scratch_bytes=built.forecast.scratch_bytes;
  EXPECT_EQ(s::BuildStarter(in,cap,built.output,built.scratch,&built.startup).status,s::Status::Ok);
  EXPECT_EQ(std::memcmp(before.data(),built.output.data(),before.size()),0);
}
TEST(Type25GeneralStartup, SnapshotOriginMismatchDoesNotPublishReadyValues) {
  auto source=Grid(2,1);Built built(source);auto view=built.startup;const auto input=source.Input();const auto bytes=Bytes(built.ready_output);
  view.profile=s::Profile::OrdinaryExteriorMovingMain;
  EXPECT_EQ(s::BuildFixedMain(input,view,{source.coefficients.data(),source.coefficients.size()},
      {},built.ready_output,built.ready_scratch,&built.ready).status,s::Status::InvalidInput);
  view=built.startup;view.topology=s::TopologyPolicy::NativeOrdinaryShell;
  EXPECT_EQ(s::BuildFixedMain(input,view,{source.coefficients.data(),source.coefficients.size()},
      {},built.ready_output,built.ready_scratch,&built.ready).status,s::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(bytes.data(),built.ready_output.data(),bytes.size()),0);
}
TEST(Type25GeneralStartup, CompleteCensusForecastRequiresNoSourceReadsOrAllocation) {
  s::Input in;in.profile=s::Profile::OrdinaryExteriorMovingMain;
  in.topology=s::TopologyPolicy::NativeOrdinaryShell;in.node_count=393165;in.primary_count=337092;
  // Null source pointers are deliberate: this is count/profile-only preflight,
  // not full model admission or an allocation of the forecast arenas.
  const auto plan=s::Preflight(in);ASSERT_EQ(plan.status,s::Status::Ok);
  EXPECT_EQ(plan.expanded_mains,674184u);EXPECT_EQ(plan.maximum_references,2696736u);
  EXPECT_LT(plan.output_bytes,std::size_t{1}<<30);EXPECT_LT(plan.scratch_bytes,std::size_t{1}<<30);
  RecordProperty("full_census_output_bytes",std::to_string(plan.output_bytes));
  RecordProperty("full_census_scratch_bytes",std::to_string(plan.scratch_bytes));
  auto cap=s::Limits{};cap.max_scratch_bytes=plan.scratch_bytes-1;
  EXPECT_EQ(s::Preflight(in,cap).status,s::Status::ResourceLimit);
  cap.max_scratch_bytes=plan.scratch_bytes;cap.max_output_bytes=plan.output_bytes;
  EXPECT_EQ(s::Preflight(in,cap).status,s::Status::Ok);
}
} // namespace type25_startup_test
