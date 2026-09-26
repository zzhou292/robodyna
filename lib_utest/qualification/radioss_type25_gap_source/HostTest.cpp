// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/collision/RadiossType25ShellSource.h"
namespace gap_source_test {
TEST(GapSource, CompleteOrderedMaskLineAndSpringCompositionMatchesOriginalNative) {
  Fixture f;const auto in=f.Input();Attempt a(in);
  ASSERT_EQ(a.Run(in).status,g::Status::Ok);Same(a.result,Oracle(in));
  Exact(a.result.secondary[4],3.); // Masked noncontact shell node restored by beam.
  Exact(a.result.secondary[9],3.); // No shell: still an authentic beam endpoint.
  Exact(a.result.secondary[5],0.); // No area/stiffness fallback for zero-thickness spring.
  Exact(a.result.main_nodes[3],4.); // Whole physical shell thickness survives at shared main node.
  for(int type:{13,25,45})for(double thickness:{-2.,-0.,0.,6.}) {
    SCOPED_TRACE(type);
    SCOPED_TRACE(thickness);
    f.springs[0].property_type=type;f.springs[0].part_contact_thickness=thickness;
    ASSERT_NO_FATAL_FAILURE(Compare(f));
  }
}
TEST(GapSource, PropertyPrecedenceModeScalesCapsAndRosterOrdersMatchAllBits) {
  for(int mode:{0,1})for(double part:{-2.,-0.,0.,4.})for(double element:{0.,6.})
    for(double scale:{0.,.5,2.}) {
      SCOPED_TRACE(mode);
      SCOPED_TRACE(part);
      SCOPED_TRACE(element);
      SCOPED_TRACE(scale);
      Fixture f;f.profile.input_thickness_mode=mode;f.profile.scale=scale;
      f.profile.maximum_main=1.75;f.profile.maximum_secondary=2.25;
      for(auto& s:f.shells){s.part_contact_thickness=part;s.element_thickness=element;}
      std::reverse(f.main_nodes.begin(),f.main_nodes.end());std::reverse(f.secondary.begin(),f.secondary.end());
      ASSERT_NO_FATAL_FAILURE(Compare(f));
    }
}
TEST(GapSource, SourceDefinedSignedZeroAndUnusedStructuralFieldsRemainExact) {
  for(unsigned mask=0;mask<32;++mask) {
    SCOPED_TRACE(mask);
    Fixture f;f.beams.clear();f.springs.clear();
    for(unsigned i=0;i<f.shells.size();++i) {
      auto& s=f.shells[i];s.property_thickness=mask&(1u<<i)?-0.:0.;
      s.young=s.structural_thickness=std::numeric_limits<double>::quiet_NaN();
    }
    f.profile.maximum_main=mask&8?-0.:0.;f.profile.maximum_secondary=mask&16?-0.:0.;
    ASSERT_NO_FATAL_FAILURE(Compare(f));
  }
  Fixture f;f.beams[0].part_contact_thickness=4.;f.beams[0].native_area=std::numeric_limits<double>::quiet_NaN();
  ASSERT_NO_FATAL_FAILURE(Compare(f));
}
TEST(GapSource, SolidRoleClearsSharedMainNodesWithoutAddingLineTermsToMain) {
  Fixture f;f.mains[1].segment_type=0;
  const auto in=f.Input();Attempt a(in);
  ASSERT_EQ(a.Run(in).status,g::Status::Ok);Same(a.result,Oracle(in));
  for(auto x:a.result.main_nodes)Exact(x,0.);
  Exact(a.result.secondary[0],1.);Exact(a.result.secondary[9],3.);
  f.trusses=f.beams;f.trusses[0].source_element_id=15;f.trusses[0].native_area=100.;
  ASSERT_NO_FATAL_FAILURE(Compare(f));
}
TEST(GapSource, EmptyNativeRostersPreserveExactExtremaAndNoInventedWrites) {
  Fixture f;f.secondary.clear();f.main_nodes.clear();f.mains.clear();f.primary=0;
  ASSERT_NO_FATAL_FAILURE(Compare(f));
  f.shells.clear();f.beams.clear();f.springs.clear();
  ASSERT_NO_FATAL_FAILURE(Compare(f));
}
TEST(GapSource, CapsAliasesMalformedRowsAndLateOverflowPreserveCompleteOutputThenRetry) {
  Fixture f;const auto good=f.Input();Attempt a(good);const auto sentinel=a.result;
  auto caps=g::Limits{};caps.scratch_bytes=a.forecast.scratch_bytes-1;
  EXPECT_EQ(a.Run(good,caps).status,g::Status::ResourceLimit);Same(a.result,sentinel);
  caps={};caps.output_bytes=a.forecast.output_bytes-1;
  EXPECT_EQ(a.Run(good,caps).status,g::Status::ResourceLimit);Same(a.result,sentinel);
  auto output=a.Output();output.secondary=&f.shells[0].property_thickness;output.secondary_count=1;
  auto one=good;one.secondary_count=1;
  const double old=f.shells[0].property_thickness;
  EXPECT_EQ(g::Build(one,{},a.scratch.data(),a.scratch.bytes(),output).status,g::Status::InvalidInput);
  Exact(f.shells[0].property_thickness,old);Same(a.result,sentinel);
  for(unsigned fault=0;fault<6;++fault) {
    SCOPED_TRACE(fault);
    Fixture invalid;
    if(fault==0)invalid.main_nodes[3]=2;
    if(fault==1)invalid.secondary[1]=0;
    if(fault==2)invalid.shells.back().nodes[3]=7;
    if(fault==3)invalid.springs[0].property_type=12;
    if(fault==4)invalid.beams[0].native_area=-1.;
    if(fault==5)invalid.mains[0].nodes[3]=99;
    const auto in=invalid.Input();Attempt trial(in);const auto retained=trial.result;
    EXPECT_NE(trial.Run(in).status,g::Status::Ok);Same(trial.result,retained);
  }
  Fixture overflow;overflow.secondary={0};overflow.profile.scale=4.;
  for(unsigned k=0;k<4;++k)overflow.shells[1].nodes[k]=4+k;
  overflow.shells[1].property_thickness=std::numeric_limits<double>::max();
  auto in=overflow.Input();Attempt late(in);const auto retained=late.result;
  const auto failed=late.Run(in);
  EXPECT_EQ(failed.status,g::Status::NonfiniteResult);EXPECT_EQ(failed.field,g::Field::Node);
  EXPECT_EQ(failed.input_row,4u);Same(late.result,retained);
  ASSERT_EQ(a.Run(good).status,g::Status::Ok);Same(a.result,Oracle(good));
}
TEST(GapSource, UnsupportedControlAndOversizedDescriptorsRejectBeforeBorrowedReads) {
  Fixture f;auto in=f.Input();g::Forecast retained{71,91};
  for(unsigned fault=0;fault<5;++fault) {
    SCOPED_TRACE(fault);
    auto bad=in;
    if(fault==0)bad.profile.gap_mode=5;
    if(fault==1)bad.profile.property_type=17;
    if(fault==2)bad.profile.free_edge_gap=1;
    if(fault==3)bad.profile.contact_thickness_update=1;
    if(fault==4)bad.profile.input_thickness_mode=2;
    EXPECT_EQ(g::Preflight(bad,{},retained).status,g::Status::UnsupportedProfile);
    EXPECT_EQ(retained.scratch_bytes,71u);EXPECT_EQ(retained.output_bytes,91u);
  }
  in.shells=reinterpret_cast<const g::PhysicalShell*>(std::uintptr_t{1});
  in.shell_count=SIZE_MAX;
  EXPECT_EQ(g::Preflight(in,{},retained).status,g::Status::ResourceLimit);
  EXPECT_EQ(retained.scratch_bytes,71u);EXPECT_EQ(retained.output_bytes,91u);
}
TEST(GapSource, LegacyOrdinaryShellBuilderRetainsItsIndependentGapResults) {
  Fixture f;f.beams.clear();f.springs.clear();
  namespace old=n::source_shells;
  old::Input in;in.node_count=f.nodes;in.shells=Data(f.shells);in.shell_count=f.shells.size();
  in.profile={old::Population::OrdinaryShellsOnly,1,0,1,1,0,0,1.,1.,1.e30,1.e30};
  const std::uint32_t primary[]{0};in.primary_shells=primary;in.primary_count=1;
  std::vector<old::Secondary> source;
  for(auto node:f.secondary)source.push_back({node,1.});
  in.secondary=source.data();in.secondary_count=source.size();
  old::Forecast forecast;
  ASSERT_EQ(old::Preflight(in,{},forecast).status,old::Status::Ok);
  tl::util::HostArena arena;ASSERT_TRUE(arena.Initialize(forecast.scratch_bytes));
  std::vector<old::NodeFields> nodes(f.nodes);std::vector<old::SecondaryFields> sec(source.size());double stiffness;
  old::Output out{nodes.data(),nodes.size(),&stiffness,1,sec.data(),sec.size()};
  ASSERT_EQ(old::Build(in,{},arena.data(),arena.bytes(),out).status,old::Status::Ok);
  Attempt actual(f.Input());ASSERT_EQ(actual.Run(f.Input()).status,g::Status::Ok);
  for(unsigned i=0;i<sec.size();++i)Exact(actual.result.secondary[i],sec[i].gap);
  for(unsigned i=0;i<f.main_nodes.size();++i)Exact(actual.result.main_nodes[i],nodes[f.main_nodes[i]].main_gap);
  Same(actual.result,Oracle(f.Input()));
}
} // namespace gap_source_test
