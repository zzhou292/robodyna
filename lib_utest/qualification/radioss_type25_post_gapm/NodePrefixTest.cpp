// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>
namespace type25_post_gapm_test {
namespace {
struct Extended {
  Fixture original;
  std::vector<std::uint64_t> ids;
  std::vector<n::Vector> points;
  s::Input input;
  s::NodePrefixExtension prefix;
  s::Forecast forecast;
  tl::util::HostArena output,scratch;
  s::Snapshot snapshot;
  explicit Extended(bool si=false):original(Blocks(1,true)),ids(original.mesh.ids),points(original.mesh.points),input(original.input) {
    if(original.report.status!=s::Status::Ok)throw std::runtime_error("Original mixed fixture failed");
    if(si) {
      original.mesh.coordinates=s::Coordinates::Si;
      original.mesh.units.length_m=.001;original.mesh.units.mass_kg=1000;original.mesh.units.time_s=1;
      for(auto& x:original.mesh.points){x.x*=.001;x.y*=.001;x.z*=.001;}
      original.input.coordinates=original.mesh.coordinates;original.input.units=original.mesh.units;
      points=original.mesh.points;input=original.input;
    }
    prefix.node_source_ids=original.input.node_source_ids;prefix.positions=original.input.positions;
    prefix.node_count=original.input.node_count;prefix.coordinates=original.input.coordinates;prefix.units=original.input.units;
    for(unsigned i=0;i<4;++i){ids.push_back(9000+i);points.push_back({10.+double(i%2),-2.,double(i/2)});}
    input.node_count=ids.size();input.node_source_ids=ids.data();
    input.positions={reinterpret_cast<const double*>(points.data()),std::uint32_t(points.size()),3,1};
    forecast=s::PreflightMixedStarter(input,original.sides.result,original.post,prefix);
    if(forecast.status!=s::Status::Ok || !output.Initialize(forecast.output_bytes) || !scratch.Initialize(forecast.scratch_bytes))
      throw std::runtime_error("Combined-domain fixture forecast failed");
  }
  s::Report Build() {return s::BuildStarter(input,original.sides.result,original.post,prefix,{},output,scratch,&snapshot);}
};
}
TEST(MixedNodePrefix, GenuineSidesRemainOriginalWhileWholeNativeUsesCompleteDomain) {
  for(bool si:{false,true}) {
    Extended f(si);const auto original_count=f.original.sides.result.node_count;
    ASSERT_EQ(f.Build().status,s::Status::Ok);
    EXPECT_EQ(f.original.sides.result.node_count,original_count);
    EXPECT_EQ(f.snapshot.node_count,original_count+4);
    const auto expected=Oracle(f.input,f.original.post,f.original.coefficients.data(),f.original.coefficients.size());
    type25_startup_test::SameStarter(f.snapshot,expected);
    EXPECT_EQ(f.snapshot.primary_count,f.original.startup.primary_count);
    EXPECT_EQ(f.snapshot.main_count,f.original.startup.main_count);
    EXPECT_EQ(s::PreflightMixedStarter(f.input,f.original.sides.result,f.original.post).status,s::Status::InvalidInput);
    EXPECT_EQ(s::BuildStarter(f.input,f.original.sides.result,f.original.post,{},f.output,f.scratch,&f.snapshot).status,s::Status::InvalidInput);
  }
}
TEST(MixedNodePrefix, PrefixBitsUnitsAndFaceExtentRejectBeforeAnyPublication) {
  Extended f(true);ASSERT_EQ(f.Build().status,s::Status::Ok);
  const auto saved=upstream::Bytes(f.output);const auto good_input=f.input;
  for(unsigned fault=0;fault<6;++fault) {
    auto prefix=f.prefix;auto faces=std::vector<s::PrimaryFace>(f.input.primary,f.input.primary+f.input.primary_count);
    if(fault==0)f.ids[0]++;
    if(fault==1)f.points[0].x=-0.;
    if(fault==2)prefix.units.length_m=.01;
    if(fault==3)prefix.node_count=f.input.node_count;
    if(fault==4){faces.back().nodes[0]=std::uint32_t(prefix.node_count);f.input.primary=faces.data();}
    if(fault==5)prefix.positions.data=nullptr;
    EXPECT_NE(s::PreflightMixedStarter(f.input,f.original.sides.result,f.original.post,prefix).status,s::Status::Ok);
    EXPECT_NE(s::BuildStarter(f.input,f.original.sides.result,f.original.post,prefix,{},f.output,f.scratch,&f.snapshot).status,s::Status::Ok);
    EXPECT_EQ(upstream::Bytes(f.output),saved);
    f.ids[0]=f.original.mesh.ids[0];f.points[0]=f.original.mesh.points[0];f.input=good_input;
  }
  EXPECT_EQ(f.Build().status,s::Status::Ok);EXPECT_EQ(upstream::Bytes(f.output),saved);
}
TEST(MixedNodePrefix, AddedDuplicateOrNonfiniteNodeCannotRepairOrChangeOriginalSource) {
  Extended f;ASSERT_EQ(f.Build().status,s::Status::Ok);
  const auto saved=upstream::Bytes(f.output);const auto old=f.snapshot;
  const auto id=f.ids.back();const auto point=f.points.back();
  for(unsigned fault=0;fault<3;++fault) {
    if(fault==0)f.ids.back()=f.ids.front();
    if(fault==1)f.ids.back()=0;
    if(fault==2)f.points.back().z=std::numeric_limits<double>::infinity();
    EXPECT_EQ(f.Build().status,s::Status::InvalidInput);
    EXPECT_EQ(upstream::Bytes(f.output),saved);EXPECT_EQ(f.snapshot.mains,old.mains);
    f.ids.back()=id;f.points.back()=point;
  }
  EXPECT_EQ(f.Build().status,s::Status::Ok);
}
TEST(MixedNodePrefix, BorrowedPrefixAliasAndExactArenaCapPreserveOutput) {
  Extended f;ASSERT_EQ(f.Build().status,s::Status::Ok);
  const auto saved=upstream::Bytes(f.output);const auto scratch=upstream::Bytes(f.scratch);
  auto* alias=reinterpret_cast<s::Snapshot*>(f.original.mesh.points.data());
  EXPECT_EQ(s::BuildStarter(f.input,f.original.sides.result,f.original.post,f.prefix,{},f.output,f.scratch,alias).status,s::Status::InvalidInput);
  EXPECT_EQ(upstream::Bytes(f.output),saved);EXPECT_EQ(upstream::Bytes(f.scratch),scratch);
  auto limits=s::Limits{};limits.max_scratch_bytes=f.forecast.scratch_bytes-1;
  EXPECT_EQ(s::PreflightMixedStarter(f.input,f.original.sides.result,f.original.post,f.prefix,limits).status,s::Status::ResourceLimit);
  limits.max_scratch_bytes++;
  EXPECT_EQ(s::PreflightMixedStarter(f.input,f.original.sides.result,f.original.post,f.prefix,limits).status,s::Status::Ok);
  EXPECT_EQ(s::BuildStarter(f.input,f.original.sides.result,f.original.post,f.prefix,limits,f.output,f.scratch,&f.snapshot).status,s::Status::Ok);
}
}
