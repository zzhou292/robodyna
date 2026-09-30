// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NormalViewFixture.h"
#include <array>
namespace type25_lifecycle_test::normal_view {
namespace {
struct Scratch {
  std::array<l::Occurrence,128> occurrences{};
  std::array<n::NativeRawGeometryResult,128> geometry{};
  std::array<int,512> sliding{};
  l::RowScratch View(){return {occurrences.data(),geometry.data(),occurrences.size(),sliding.data(),sliding.size(),0};}
};
}
TEST(Type25NormalView, LegacyAndCompleteViewMatchFullNativeRowsAcrossAllClassificationPaths) {
  for(unsigned scenario=0;scenario<7;++scenario) {
    SCOPED_TRACE(scenario);auto f=Scenario(scenario);Fields fields(f);
    l::HostResult legacy,current;
    const auto before=l::EvaluateNativeLifecycleHost(f.Input(),Fixture::Limits(),&legacy);
    ASSERT_EQ(before.status,n::selection::Status::Ok);Same(legacy,OracleLifecycle(f.Input()));
    SameReport(Evaluate(f,fields,current),before);Same(current,legacy,true);
    PoisonLegacy(f);auto input=fields.Bind(f);input.source.normals=nullptr;
    SameReport(l::EvaluateNativeLifecycleHost(input,Fixture::Limits(),&current),before);Same(current,legacy,true);
    // A cleared descriptor is genuinely the old path, never a remembered lease.
    input.current_normals={};
    EXPECT_EQ(l::EvaluateNativeLifecycleHost(input,Fixture::Limits(),&current).status,n::selection::Status::InvalidInput);
    Same(current,legacy,true);
  }
}
TEST(Type25NormalView, FactoriesReadOneViewForPrimaryOppositeContinuationAndSelectedGeometry) {
  auto f=Scenario(5);Fields fields(f);
  for(std::size_t i=0;i<fields.faces.size();++i)fields.faces[i]={float(i)+.25f,-0.f,-float(i)-.5f};
  for(std::size_t i=0;i<fields.references.size();++i) {
    auto& r=fields.references[i];r.boundary=i%2?2:0;
    r.bisector[0]={-0.f,float(i)+.125f,-1.f};r.bisector[1]={1.f,-float(i)-.5f,0.f};
    if(!r.boundary)r.bisector[0].x=std::numeric_limits<float>::quiet_NaN();
  }
  PoisonLegacy(f);auto input=fields.Bind(f);input.source.normals=nullptr;
  ASSERT_EQ(l::detail::Validate(input),n::selection::Status::Ok);
  n::units_detail::Factors units;ASSERT_TRUE(l::detail::Factors(input.current,units));
  int sliding[4]{};
  for(std::size_t m=0;m<f.mains.size();++m) {
    const auto pair=l::detail::Pair(input,0,int(m+1),17,units);
    const auto continuation=l::detail::Continuation(input,0,int(m+1),17,sliding,units);
    for(unsigned j=0;j<4;++j) {
      SameFloat(pair.normal_slot[j],fields.faces[4*m+j]);SameFloat(continuation.pair.normal_slot[j],fields.faces[4*m+j]);
      const auto ref=std::size_t(f.mains[m].normal_reference[j]-1);const auto& r=fields.references[ref];
      EXPECT_EQ(pair.boundary_ids[j],r.boundary?ref+1:0);
      for(unsigned k=0;k<2;++k)SameFloat(pair.vertex_bisector[j][k],r.boundary?r.bisector[k]:n::StoredNormal{});
    }
    l::Occurrence selected;selected.selected={true,pair.key,int(m+1),1,1,.25,.25,2};selected.cache.occurrence=17;
    const auto geometry=l::detail::Geometry(input,selected,units);
    for(unsigned j=0;j<4;++j)SameFloat(geometry.corner_normal[j],fields.faces[4*m+j]);
  }
  const auto impact=l::detail::NewImpact(input,0,1,19,units);ASSERT_EQ(impact.opposite.local_main,2);
  for(unsigned j=0;j<4;++j) {
    SameFloat(impact.opposite.normal_slot[j],fields.faces[4+j]);
    const auto ref=std::size_t(f.mains[1].normal_reference[j]-1);const auto& r=fields.references[ref];
    EXPECT_EQ(impact.opposite.boundary_ids[j],r.boundary?ref+1:0);
    for(unsigned k=0;k<2;++k)SameFloat(impact.opposite.vertex_bisector[j][k],r.boundary?r.bisector[k]:n::StoredNormal{});
  }
}
TEST(Type25NormalView, SameAddressesUpdatedBetweenCallsRemainIndependentOfStaleEmbeddedPayloads) {
  auto actual_scene=Scenario(1),native_scene=actual_scene;Fields fields(actual_scene);PoisonLegacy(actual_scene);
  const auto* face_address=fields.faces.data();const auto* ref_address=fields.references.data();
  for(unsigned step=0;step<4;++step) {
    SCOPED_TRACE(step);fields.Scale(1.03125f);
    const auto control=fields.LegacyControl(native_scene);const auto native=OracleLifecycle(control.Input());
    l::HostResult expected,actual;const auto reference=l::EvaluateNativeLifecycleHost(control.Input(),Fixture::Limits(),&expected);
    ASSERT_EQ(reference.status,n::selection::Status::Ok);Same(expected,native);
    SameReport(Evaluate(actual_scene,fields,actual),reference);Same(actual,expected,true);
    EXPECT_EQ(fields.faces.data(),face_address);EXPECT_EQ(fields.references.data(),ref_address);
    const auto actual_finish=OracleFinish(actual.rows),native_finish=OracleFinish(native.rows);
    for(std::size_t row=0;row<actual_scene.secondary.size();++row) {
      actual_scene.accepted[row]=actual_finish[row].history;actual_scene.secondary[row].initial_contact_flag=actual_finish[row].initial_contact_flag;
      native_scene.accepted[row]=native_finish[row].history;native_scene.secondary[row].initial_contact_flag=native_finish[row].initial_contact_flag;
    }
    actual_scene.step.time=native_scene.step.time=(step+1)*.001;
  }
}
TEST(Type25NormalView, PartialMalformedAndAliasedViewsNeverFallBackOrReplacePriorOutput) {
  auto f=Scenario(0);Fields clean(f);l::HostResult prior;ASSERT_EQ(Evaluate(f,clean,prior).status,n::selection::Status::Ok);
  for(unsigned fault=0;fault<10;++fault) {
    SCOPED_TRACE(fault);auto fields=clean;auto in=fields.Bind(f);auto out=prior;
    switch(fault) {
      case 0:in.current_normals.normal_count=0;break;
      case 1:--in.current_normals.normal_count;break;
      case 2:--in.current_normals.reference_count;break;
      case 3:in.current_normals.face_normals=nullptr;break;
      case 4:in.current_normals.references=nullptr;break;
      case 5:in.current_normals.normal_count=SIZE_MAX;break;
      case 6:in.current_normals.face_normals=reinterpret_cast<const n::StoredNormal*>(reinterpret_cast<const unsigned char*>(fields.faces.data())+1);break;
      case 7:in.current_normals.face_normals=fields.references[0].bisector;break; // A live typed subobject, rejected before any aliased float read.
      case 8:fields.faces.back().z=std::numeric_limits<float>::quiet_NaN();break;
      case 9:fields.references.back().boundary=2;fields.references.back().bisector[1].y=std::numeric_limits<float>::quiet_NaN();break;
    }
    EXPECT_EQ(l::EvaluateNativeLifecycleHost(in,Fixture::Limits(),&out).status,n::selection::Status::InvalidInput);Same(out,prior,true);
  }
  l::HostResult retry;ASSERT_EQ(Evaluate(f,clean,retry).status,n::selection::Status::Ok);Same(retry,prior,true);
}
TEST(Type25NormalView, UnboundScratchAndSignedZeroSurviveWithoutInventedBoundaryValues) {
  auto f=Scenario(5);Fields fields(f);
  for(auto& r:fields.references){r.boundary=0;for(auto& v:r.bisector)v={std::numeric_limits<float>::quiet_NaN(),0,-0.f};}
  fields.faces[2]={-0.f,0.f,-0.f};
  const auto control=fields.LegacyControl(f);l::HostResult expected,actual;
  ASSERT_EQ(l::EvaluateNativeLifecycleHost(control.Input(),Fixture::Limits(),&expected).status,n::selection::Status::Ok);
  Same(expected,OracleLifecycle(control.Input()));PoisonLegacy(f);
  ASSERT_EQ(Evaluate(f,fields,actual).status,n::selection::Status::Ok);Same(actual,expected,true);
  const auto input=fields.Bind(f);n::units_detail::Factors units;ASSERT_TRUE(l::detail::Factors(input.current,units));
  const auto pair=l::detail::Pair(input,0,1,0,units);SameFloat(pair.normal_slot[2],fields.faces[2]);
  for(const auto& edge:pair.vertex_bisector)for(const auto& v:edge)SameFloat(v,{});
}
TEST(Type25NormalView, BothWritingPhasesRejectLiveReferenceAliasBeforeMutation) {
  auto f=Scenario(1);Fields fields(f);const auto in=fields.Bind(f);
  n::units_detail::Factors units;ASSERT_TRUE(l::detail::Factors(in.current,units));
  const auto optimized=l::detail::PrepareRowBeforeNormals(in,0,units);ASSERT_EQ(optimized.stage.report.status,n::selection::Status::Ok);
  Scratch storage;auto scratch=storage.View();const auto before=fields.references;
  // Actual int subobject inside a live reference record, not a cast of an
  // unrelated struct. Rejection must precede even the first sliding write.
  scratch.sliding_mains=&fields.references[0].boundary;scratch.sliding_capacity=1;
  const auto failed=l::detail::PrepareRowAfterNormals(in,0,scratch,units,optimized);
  EXPECT_EQ(failed.stage.report.status,n::selection::Status::InvalidInput);EXPECT_EQ(failed.stage.report.stage,l::Stage::Admission);
  EXPECT_EQ(std::memcmp(before.data(),fields.references.data(),before.size()*sizeof(before[0])),0);
  const auto prepared=l::detail::PrepareRowAfterNormals(in,0,storage.View(),units,optimized);
  ASSERT_EQ(prepared.stage.report.status,n::selection::Status::Ok);
  const auto stopped=l::detail::CompleteRow(in,0,scratch,units,prepared);
  EXPECT_EQ(stopped.report.status,n::selection::Status::InvalidInput);EXPECT_EQ(stopped.report.stage,l::Stage::Admission);
  EXPECT_EQ(std::memcmp(before.data(),fields.references.data(),before.size()*sizeof(before[0])),0);
  ASSERT_EQ(l::detail::CompleteRow(in,0,storage.View(),units,prepared).report.status,n::selection::Status::Ok);
}
TEST(Type25NormalView, BeforeNormalsDoesNotReadAnUnreadyViewAndAfterRequiresTheCompleteBinding) {
  auto f=Scenario(1);Fields fields(f);PoisonLegacy(f);auto in=f.Input();in.source.normals=nullptr;
  in.current_normals.normal_count=1; // Deliberately incomplete before the producer barrier.
  n::units_detail::Factors units;ASSERT_TRUE(l::detail::Factors(in.current,units));
  const auto optimized=l::detail::PrepareRowBeforeNormals(in,0,units);ASSERT_EQ(optimized.stage.report.status,n::selection::Status::Ok);
  Scratch storage;
  EXPECT_EQ(l::detail::PrepareRowAfterNormals(in,0,storage.View(),units,optimized).stage.report.status,n::selection::Status::InvalidInput);
  in.current_normals=fields.View();ASSERT_EQ(l::detail::Validate(in),n::selection::Status::Ok);
  const auto prepared=l::detail::PrepareRowAfterNormals(in,0,storage.View(),units,optimized);
  ASSERT_EQ(prepared.stage.report.status,n::selection::Status::Ok);
  const auto complete=l::detail::CompleteRow(in,0,storage.View(),units,prepared);ASSERT_EQ(complete.report.status,n::selection::Status::Ok);
  l::HostResult actual;actual.rows.push_back(complete.value);
  actual.occurrences.assign(storage.occurrences.begin(),storage.occurrences.begin()+complete.occurrence_count);
  actual.geometry.assign(storage.geometry.begin(),storage.geometry.begin()+complete.occurrence_count);
  const auto control=fields.LegacyControl(f);Same(actual,OracleLifecycle(control.Input()));
}
} // namespace type25_lifecycle_test::normal_view
