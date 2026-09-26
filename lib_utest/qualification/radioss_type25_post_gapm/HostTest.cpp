// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>
namespace type25_post_gapm_test {
TEST(PostGapmStartup, WholeNativeTopologyAndBothNormalPhasesMatchGenuineMixedSides) {
  for(unsigned mode=0;mode<5;++mode)for(bool shells:{false,true}) {
    if(mode==4 && shells)continue;
    SCOPED_TRACE(mode);
    SCOPED_TRACE(shells);
    Fixture f(Blocks(mode,shells));
    ASSERT_EQ(f.report.status,s::Status::Ok);
    type25_startup_test::SameStarter(f.startup,f.native);
    EXPECT_EQ(f.report.neighbor_warnings.count,std::size_t(f.native.warning_count));
    EXPECT_EQ(f.startup.main_count,f.startup.primary_count+f.startup.shell_primary_count);
    for(std::size_t i=0;i<f.startup.primary_count;++i)
      EXPECT_EQ(f.startup.primary_to_partner[i]==0,f.input.primary_identities[i].kind==s::PrimaryFaceKind::Solid);
    if(mode==1 || mode==3 || mode==4) {
      EXPECT_GT(f.post.pre_shell_internal_count,0u);
      EXPECT_EQ(f.startup.post_gapm->final_solid_erosion,s::SolidErosion::Enabled);
    }
    if(mode==4)EXPECT_GT(f.native.selector_calls,0u);
    const auto ready=Evaluate(f,f.Current());
    ASSERT_EQ(ready.report.status,c::Status::Ok);
    current::SameNormals(ready.normals,f.native.ready_normals);
    current::SameReferences(ready.references,f.native.ready_references);
  }
}
TEST(PostGapmStartup, ExplicitSiSourceLengthRetainsNativeTopologyAndFloatStages) {
  auto source=Blocks(1,true);source.coordinates=s::Coordinates::Si;source.units={1000,.001,1};
  for(auto& x:source.points){x.x*=.001;x.y*=.001;x.z*=.001;}
  Fixture f(std::move(source));ASSERT_EQ(f.report.status,s::Status::Ok);
  type25_startup_test::SameStarter(f.startup,f.native);
  current::Same(Evaluate(f,f.Current()),current::OracleMixed(f.Current(true)));
}
TEST(PostGapmStartup, PrimaryOnlyPermutationRetainsAuthenticUnchangedShellPartner) {
  for(bool triangle:{false,true}) {
    Fixture f(triangle?Penta():Blocks(0,true),true);
    ASSERT_EQ(f.report.status,s::Status::Ok);
    type25_startup_test::SameStarter(f.startup,f.native);
    bool coating=false;
    for(std::size_t i=0;i<f.startup.primary_count;++i)if(f.before[i].first_solid_source_id && f.startup.primary_to_partner[i]) {
      coating=true;const auto partner=f.startup.primary_to_partner[i]-1;
      for(unsigned k=0;k<4;++k)EXPECT_EQ(f.startup.mains[partner].nodes[k],f.sides.result.mains[partner].nodes[k]);
    }
    EXPECT_TRUE(coating);
    EXPECT_EQ(c::ValidateMixedSource(f.Current().topology,f.startup).status,c::Status::Ok);
    current::Same(Evaluate(f,f.Current()),current::OracleMixed(f.Current(true)));
  }
}
TEST(PostGapmStartup, ErosionBranchUsesRealPreShellCountAndSourceCopiesAreOwned) {
  Fixture enabled(Blocks(3)),disabled(Blocks(3),false,false);
  ASSERT_EQ(enabled.report.status,s::Status::Ok);
  ASSERT_EQ(disabled.report.status,s::Status::Ok);
  type25_startup_test::SameStarter(disabled.startup,disabled.native);
  EXPECT_NE(enabled.startup.post_gapm, &enabled.post);
  EXPECT_NE(enabled.startup.post_gapm->primary_corners,enabled.corners.data());
  EXPECT_NE(enabled.startup.post_gapm->final_support,enabled.supports.data());
  const auto before=enabled.startup.post_gapm->before_shell[0].first_solid_source_id;
  enabled.before[0].first_solid_source_id=99999;
  EXPECT_EQ(enabled.startup.post_gapm->before_shell[0].first_solid_source_id,before);
  EXPECT_NE(enabled.native.offsets,disabled.native.offsets); // Extra unions are actually consumed.
}
TEST(PostGapmStartup, InvalidLateMetadataAndCapacityPreservePublishedBytesThenRetry) {
  Fixture f(Blocks(1,true));ASSERT_EQ(f.report.status,s::Status::Ok);
  const auto saved=upstream::Bytes(f.output);const auto snapshot=f.startup;
  for(unsigned fault=0;fault<6;++fault) {
    auto post=f.post;auto support=f.supports;auto corners=f.corners;auto limit=s::Limits{};
    if(fault==0)post.pre_shell_internal_count=0;
    if(fault==1)post.final_solid_erosion=s::SolidErosion::Disabled;
    if(fault==2){support.back().first.source_element_id=0;post.final_support=support.data();}
    if(fault==3){corners.back().source_corner[0]=4;post.primary_corners=corners.data();}
    if(fault==4)post.source_generation++;
    if(fault==5)limit.max_output_bytes=f.forecast.output_bytes-1;
    EXPECT_NE(s::BuildStarter(f.input,f.sides.result,post,limit,f.output,f.scratch,&f.startup).status,s::Status::Ok);
    EXPECT_EQ(upstream::Bytes(f.output),saved);
    EXPECT_EQ(f.startup.mains,snapshot.mains);EXPECT_EQ(f.startup.post_gapm,snapshot.post_gapm);
  }
  EXPECT_EQ(s::BuildStarter(f.input,f.sides.result,f.post,{},f.output,f.scratch,&f.startup).status,s::Status::Ok);
  EXPECT_EQ(upstream::Bytes(f.output),saved);
}
TEST(PostGapmStartup, RichMetadataAndArenaHandleAliasesRejectWithoutPublication) {
  Fixture f(Blocks(1));ASSERT_EQ(f.report.status,s::Status::Ok);
  const auto saved=upstream::Bytes(f.output);const auto scratch=upstream::Bytes(f.scratch);
  auto* alias=const_cast<s::Snapshot*>(reinterpret_cast<const s::Snapshot*>(f.post.final_support));
  EXPECT_EQ(s::BuildStarter(f.input,f.sides.result,f.post,{},f.output,f.scratch,alias).status,s::Status::InvalidInput);
  EXPECT_EQ(upstream::Bytes(f.output),saved);EXPECT_EQ(upstream::Bytes(f.scratch),scratch);
  EXPECT_NE(s::BuildStarter(f.input,{},f.output,f.scratch,&f.startup).status,s::Status::Ok);
}
TEST(PostGapmNormals, IndependentNativeRecurrenceIncludesInactiveNegativeAndChangedGeometry) {
  for(unsigned mode:{1u,3u,4u}) {
    Fixture f(Blocks(mode,mode!=4));ASSERT_EQ(f.report.status,s::Status::Ok);
    for(unsigned step=0;step<5;++step) {
      SCOPED_TRACE(mode);
      SCOPED_TRACE(step);
      f.Deform(step);
      if(step==1)for(std::size_t i=0;i<f.active.size();++i)f.active[i]=i%2;
      if(step==2){std::fill(f.active.begin(),f.active.end(),0);std::fill(f.tags.begin(),f.tags.end(),0);}
      if(step==3){std::fill(f.coefficients.begin(),f.coefficients.end(),-0.);f.RefreshFree();}
      if(step==4){std::fill(f.coefficients.begin(),f.coefficients.end(),200.);std::fill(f.active.begin(),f.active.end(),1);std::fill(f.tags.begin(),f.tags.end(),1);f.RefreshFree();}
      const auto expected=current::OracleMixed(f.Current(true));
      const auto actual=Evaluate(f,f.Current());current::Same(actual,expected);
      f.prior=actual.normals;f.native_prior=expected.normals;
    }
  }
}
TEST(PostGapmNormals, RichIdentityMapsHostCapAndLateFailureRemainClosedAndAtomic) {
  Fixture f(Blocks(1,true));ASSERT_EQ(f.report.status,s::Status::Ok);
  auto in=f.Current();c::Forecast forecast;
  ASSERT_EQ(c::Preflight(in,f.startup,Fixture::Limits(),forecast).status,c::Status::Ok);
  EXPECT_EQ(forecast.source_validation_bytes,c::MixedSourceValidationBytes(f.startup.primary_count));
  auto cap=Fixture::Limits();cap.source_validation_bytes=forecast.source_validation_bytes-1;
  EXPECT_EQ(c::ValidateMixedSource(in.topology,f.startup,cap.source_validation_bytes).status,c::Status::ResourceLimit);
  cap.source_validation_bytes++;
  EXPECT_EQ(c::ValidateMixedSource(in.topology,f.startup,cap.source_validation_bytes).status,c::Status::Ok);
  EXPECT_EQ(c::Preflight(in,Fixture::Limits(),forecast).status,c::Status::UnsupportedProfile);
  tl::util::HostArena scratch;ASSERT_TRUE(scratch.Initialize(forecast.scratch_bytes));
  std::vector<n::StoredNormal> normals(in.prior_count,{7.f,-0.f,11.f});
  std::vector<s::NormalReference> refs(in.topology.references);for(auto& ref:refs)ref.boundary=71;
  const auto old_normals=normals;const auto old_refs=refs;
  f.positions.back()=std::numeric_limits<double>::infinity();
  EXPECT_EQ(c::Evaluate(in,f.startup,cap,scratch.data(),scratch.bytes(),{normals.data(),normals.size(),refs.data(),refs.size()}).status,c::Status::InvalidInput);
  current::SameNormals(normals,old_normals);current::SameReferences(refs,old_refs);
  f.Deform(0);
  auto forged=f.startup;auto origins=std::vector<s::PrimaryFaceIdentity>(forged.raw_origins,forged.raw_origins+forged.raw_origin_count);
  origins.back().physical_parent_id=0;forged.raw_origins=origins.data();
  EXPECT_NE(c::ValidateMixedSource(in.topology,forged).status,c::Status::Ok);
  std::vector<std::uint32_t> partners(f.startup.primary_to_partner,f.startup.primary_to_partner+f.startup.primary_count);
  partners[0]=std::uint32_t(f.startup.main_count+1);in.topology.mixed_maps.primary_to_partner=partners.data();
  EXPECT_NE(c::ValidateMixedSource(in.topology,f.startup).status,c::Status::Ok);
  current::Same(Evaluate(f,f.Current()),current::OracleMixed(f.Current(true)));
}
}
