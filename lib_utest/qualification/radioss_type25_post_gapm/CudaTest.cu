// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../radioss_type25_current_normals/CudaFixture.h"
namespace type25_post_gapm_test {
using MixedNormalsCuda=current::device::CurrentNormalsCuda;
namespace {
void SameStages(const current::device::Observation& actual,const current::NativeResult& expected) {
  ASSERT_EQ(actual.report.status,c::Status::Ok);
  ASSERT_TRUE(expected.finite);
  current::SameNormals(actual.values.flag1_normals,expected.flag1_normals);
  current::SameNormals(actual.values.normals,expected.normals);
  current::SameReferences(actual.values.references,expected.references);
  EXPECT_EQ(actual.values.primary_skip,expected.primary_skip);
}
}
TEST_F(MixedNormalsCuda, GenuineMixedTopologyMatchesWholeNativeStagesAndRecurrence) {
  for(unsigned mode:{1u,3u,4u})for(unsigned threads:{1u,7u,32u,64u})for(bool reverse:{false,true}) {
    SCOPED_TRACE(mode);
    SCOPED_TRACE(threads);
    SCOPED_TRACE(reverse);
    Fixture f(Blocks(mode,mode!=4));ASSERT_EQ(f.report.status,s::Status::Ok);
    for(unsigned step=0;step<4;++step) {
      SCOPED_TRACE(step);
      f.Deform(step);
      if(step==1)for(std::size_t i=0;i<f.active.size();++i)f.active[i]=i%2;
      if(step==2){std::fill(f.active.begin(),f.active.end(),0);std::fill(f.tags.begin(),f.tags.end(),0);}
      if(step==3){std::fill(f.coefficients.begin(),f.coefficients.end(),-0.);f.RefreshFree();}
      const auto expected=current::OracleMixed(f.Current(true));
      const auto actual=EvaluateDevice(f.Current(),threads,reverse,Fixture::Limits(),&f.startup);
      SameStages(actual,expected);f.prior=actual.values.normals;f.native_prior=expected.normals;
    }
  }
}
TEST_F(MixedNormalsCuda, PrimaryReversalAndTriangleUnusedCacheAreIndependentOfLaunchOrder) {
  for(bool triangle:{false,true})for(unsigned threads:{1u,64u}) {
    Fixture f(triangle?Penta():Blocks(0,true),true);ASSERT_EQ(f.report.status,s::Status::Ok);
    for(std::size_t i=0;i<f.startup.main_count;++i)if(f.startup.mains[i].nodes[2]==f.startup.mains[i].nodes[3])
      f.prior[4*i+2]=f.native_prior[4*i+2]={float(i+1),-0.f,.125f};
    f.Deform(2);
    const auto expected=current::OracleMixed(f.Current(true));
    SameStages(EvaluateDevice(f.Current(),threads,true,Fixture::Limits(),&f.startup),expected);
  }
}
TEST_F(MixedNormalsCuda, FailedSourceAndLateFloatArithmeticPreservePublicationAndRetry) {
  Fixture f(Blocks(1,true));ASSERT_EQ(f.report.status,s::Status::Ok);
  const auto expected=current::OracleMixed(f.Current(true));
  const auto valid=EvaluateDevice(f.Current(),32,false,Fixture::Limits(),&f.startup);SameStages(valid,expected);
  for(unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault);
    f.Deform(0);auto cap=Fixture::Limits();auto source=f.startup;
    if(fault==0)cap.source_validation_bytes=c::MixedSourceValidationBytes(source.primary_count)-1;
    if(fault==1)source.raw_origin_count--;
    if(fault==2)f.tags.back()=2;
    if(fault==3)for(auto& x:f.positions)x*=1e25;
    const auto result=EvaluateDevice(f.Current(),64,true,cap,&source);
    EXPECT_NE(result.report.status,c::Status::Ok);
    if(fault==3)EXPECT_EQ(result.report.status,c::Status::NonfiniteResult);
    EXPECT_TRUE(result.publication_unchanged);
    current::SameNormals(result.values.normals,valid.values.normals);
    current::SameReferences(result.values.references,valid.values.references);
    f.tags.back()=1;
  }
  f.Deform(0);SameStages(EvaluateDevice(f.Current(),7,false,Fixture::Limits(),&f.startup),expected);
}
}
