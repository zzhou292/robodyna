// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
#include "NormalViewFixture.h"
namespace type25_lifecycle_test {
namespace nv=normal_view;
TEST(Type25NormalViewCuda,CompleteRowsMatchCopiedLegacyAndIndependentNativeAtBothLaunchShapes) {
  device::Device gpu;
  for(unsigned scenario=0;scenario<7;++scenario) {
    SCOPED_TRACE(scenario);
    auto f=nv::Scenario(scenario);nv::Fields fields(f);fields.Scale(.5f);
    auto control=fields.LegacyControl(f);const auto cpu=RunLifecycleFixture(control);
    const auto native=OracleLifecycle(control.Input());Same(cpu,native);
    const auto view=fields.View();
    for(unsigned threads:{1u,32u})for(bool reverse:{false,true}) {
      l::HostResult actual;
      ASSERT_EQ(gpu.Evaluate(f,actual,reverse,512,threads,true,&view).status,n::selection::Status::Ok);
      Same(actual,cpu,true);Same(actual,native);
    }
  }
}
TEST(Type25NormalViewCuda,UnusedEmbeddedFloatsAndNullLegacyReferencesDoNotOverrideTheView) {
  device::Device gpu;
  for(unsigned scenario:{0u,1u,3u,5u,6u}) {
    SCOPED_TRACE(scenario);
    auto f=nv::Scenario(scenario);nv::Fields fields(f);
    auto control=fields.LegacyControl(f);const auto expected=OracleLifecycle(control.Input());
    nv::PoisonLegacy(f);const auto view=fields.View();l::HostResult actual;
    ASSERT_EQ(gpu.Evaluate(f,actual,false,512,32,false,&view,true).status,n::selection::Status::Ok);
    Same(actual,expected);l::HostResult cpu;
    ASSERT_EQ(nv::Evaluate(f,fields,cpu).status,n::selection::Status::Ok);Same(actual,cpu,true);
    auto stale=actual;
    EXPECT_NE(gpu.Evaluate(f,stale).status,n::selection::Status::Ok);Same(stale,actual,true);
  }
}
TEST(Type25NormalViewCuda,RowsKeepGlobalOccurrenceOrderAcrossCurrentNormalBoundary) {
  auto f=nv::Scenario(1);f.AddSecondary();f.spatial={{2,1},{1,3},{1,1},{2,3}};f.Rebuild();
  nv::Fields fields(f);auto control=fields.LegacyControl(f);const auto native=OracleLifecycle(control.Input());
  ASSERT_EQ(native.occurrences.size(),3u);const auto view=fields.View();device::Device gpu;
  for(bool reverse:{false,true})for(bool split:{false,true}) {
    l::HostResult actual;ASSERT_EQ(gpu.Evaluate(f,actual,reverse,512,32,split,&view).status,n::selection::Status::Ok);
    Same(actual,native);EXPECT_EQ(actual.occurrences[0].origin,l::Origin::Retained);
    EXPECT_EQ(actual.occurrences[1].origin,l::Origin::Spatial);EXPECT_EQ(actual.occurrences[1].source_ordinal,0u);
    EXPECT_EQ(actual.occurrences[2].origin,l::Origin::Sliding);
  }
}
TEST(Type25NormalViewCuda,PartialWidthsAndSelectedNonfiniteFieldsPreserveResultAndRetry) {
  Fixture f;nv::Fields fields(f);const auto view=fields.View();device::Device gpu;l::HostResult original;
  ASSERT_EQ(gpu.Evaluate(f,original,false,512,32,true,&view).status,n::selection::Status::Ok);
  for(unsigned fault=0;fault<6;++fault) {
    SCOPED_TRACE(fault);
    auto bad=view;
    if(fault==0)--bad.normal_count;
    if(fault==1)++bad.reference_count;
    if(fault==2)bad.face_normals=nullptr;
    if(fault==3)bad.references=nullptr;
    if(fault==4)bad.normal_count=SIZE_MAX;
    if(fault==5)bad.reference_count=0;
    auto result=original;
    EXPECT_EQ(gpu.Evaluate(f,result,false,512,32,true,&bad).status,n::selection::Status::InvalidInput);
    Same(result,original,true);
  }
  const auto old=fields.faces[0];fields.faces[0].x=std::numeric_limits<float>::quiet_NaN();
  auto result=original;const auto bad=fields.View();
  EXPECT_EQ(gpu.Evaluate(f,result,false,512,32,true,&bad).status,n::selection::Status::InvalidInput);
  Same(result,original,true);fields.faces[0]=old;
  ASSERT_EQ(gpu.Evaluate(f,result,false,512,32,true,&view).status,n::selection::Status::Ok);
  Same(result,original,true);
}
TEST(Type25NormalViewCuda,SameBufferAddressesSupplyFreshFieldsAcrossRepeatedCalls) {
  auto f=nv::Scenario(1);nv::Fields fields(f);nv::PoisonLegacy(f);
  const auto* face_address=fields.faces.data();const auto* reference_address=fields.references.data();
  device::Device gpu;l::HostResult accepted;
  for(float scale:{1.f,.5f,2.f}) {
    fields.Scale(scale);EXPECT_EQ(fields.faces.data(),face_address);EXPECT_EQ(fields.references.data(),reference_address);
    const auto view=fields.View();auto control=fields.LegacyControl(f);
    ASSERT_EQ(gpu.Evaluate(f,accepted,false,512,32,true,&view,true).status,n::selection::Status::Ok);
    Same(accepted,OracleLifecycle(control.Input()));Same(accepted,RunLifecycleFixture(control),true);
    const auto saved=fields.faces.back();fields.faces.back().z=std::numeric_limits<float>::quiet_NaN();
    auto rejected=accepted;const auto invalid=fields.View();
    EXPECT_EQ(gpu.Evaluate(f,rejected,false,512,32,true,&invalid,true).status,n::selection::Status::InvalidInput);
    Same(rejected,accepted,true);fields.faces.back()=saved;
  }
}
TEST(Type25NormalViewCuda,WritableSlidingAliasUsesLiveReferenceIntegerAndIsRejectedBeforeMutation) {
  auto f=nv::Scenario(1);nv::Fields fields(f);const auto view=fields.View();device::Device gpu;l::HostResult original;
  ASSERT_EQ(gpu.Evaluate(f,original,false,512,32,true,&view).status,n::selection::Status::Ok);
  auto result=original;
  const auto report=gpu.Evaluate(f,result,false,512,32,true,&view,false,true);
  EXPECT_EQ(report.status,n::selection::Status::InvalidInput);Same(result,original,true);
  ASSERT_EQ(gpu.Evaluate(f,result,false,512,32,true,&view).status,n::selection::Status::Ok);
  Same(result,original,true);
}
} // namespace type25_lifecycle_test
