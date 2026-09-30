// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Support.h"
#include "ReferenceFinalize.cuh"
#include "lib_src/elements/type13/resident/measurement/Finalize.cuh"
namespace type13_read_tile_test {
__global__ void Current(b::Storage* s,t::BatchDiagnostics d){__shared__ tile::Tile staging;tile::Finalize(*s,d,staging);}
struct Device {
  b::Storage* state=nullptr;b::Measurement* values=nullptr;t::Status* status=nullptr;std::size_t count=0;
  bool Initialize(std::size_t n){count=n;
    if(cudaMallocManaged(&state,sizeof(*state))!=cudaSuccess||cudaMallocManaged(&values,n*sizeof(*values))!=cudaSuccess||cudaMallocManaged(&status,n*sizeof(*status))!=cudaSuccess)return false;
    *state={};state->model.element_count=n;state->measurement=values;state->candidate_status=status;Reset();return true;}
  void Reset(){for(std::size_t i=0;i<count;++i){values[i]=Row(i);status[i]=t::Status::Success;}state->measurement=values;}
  ~Device(){cudaFree(status);cudaFree(values);cudaFree(state);}
};
void Drain(){ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);}
b::Control Compare(Device& d,t::BatchDiagnostics seed){
  d.state->control=Poison();b::read_tile_reference::Finalize<<<1,1>>>(d.state,0,1,{},seed);Drain();const auto expected=d.state->control;
  d.state->control=Poison();Current<<<1,tile::Threads>>>(d.state,seed);Drain();Same(d.state->control,expected);return d.state->control;
}
TEST(Type13ReadTileCuda, EveryNamedControlFieldMatchesAcrossPartialTilesAndInactiveHistory){
  for(std::size_t n:{1u,2u,31u,32u,33u,63u,64u,65u,127u,128u,129u,193u,4442u}){
    SCOPED_TRACE(n);Device d;ASSERT_TRUE(d.Initialize(n));for(bool valid:{false,true}){Compare(d,Seed(valid,-0.));ASSERT_FALSE(HasFailure());}}
}
TEST(Type13ReadTileCuda, CompleteStatusPassBeatsEarlyOverflowAndNeverReadsFailedPayload){
  Device d;ASSERT_TRUE(d.Initialize(129));d.values[0].work[0]=std::numeric_limits<double>::max();d.values[1].work[0]=std::numeric_limits<double>::max();
  d.status[128]=t::Status::NonfiniteResult;auto c=Compare(d,Seed());ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,t::BatchStatus::ElementFailure);EXPECT_EQ(c.element,128u);EXPECT_EQ(c.diagnostics.element_count,101u);
  d.status[64]=t::Status::InvalidInput;d.state->measurement=nullptr;c=Compare(d,Seed(true,std::nan("19")));ASSERT_FALSE(HasFailure());EXPECT_EQ(c.element,64u);EXPECT_EQ(c.element_status,t::Status::InvalidInput);
  d.Reset();c=Compare(d,Seed());ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,t::BatchStatus::Success);
}
TEST(Type13ReadTileCuda, CancellationPreservesEveryParentChannelAndEndpointAddition){
  Device d;ASSERT_TRUE(d.Initialize(129));
  for(unsigned first:{0u,62u,63u,64u,126u})for(double seed:{0.,-0.,0x1p54,-0x1p54,std::numeric_limits<double>::denorm_min()}){
    d.Reset();for(std::size_t p=0;p<d.count;++p){auto&v=d.values[p];for(unsigned c=0;c<t::ChannelCount;++c)v.work[c]=v.increment[c]=0.;for(unsigned n=0;n<2;++n)v.kick[n]=v.drift[n]=0.;}
    const double terms[]{0x1p54,1,-0x1p54};for(unsigned i=0;i<3;++i){auto&v=d.values[first+i];for(unsigned c=0;c<t::ChannelCount;++c)v.work[c]=v.increment[c]=terms[i];v.kick[0]=terms[i];v.drift[1]=terms[i];}
    Compare(d,Seed(true,seed));ASSERT_FALSE(HasFailure());
  }
  d.Reset();for(std::size_t p=0;p<d.count;++p){auto&v=d.values[p];for(unsigned c=0;c<t::ChannelCount;++c)v.work[c]=v.increment[c]=0.;for(unsigned n=0;n<2;++n)v.kick[n]=v.drift[n]=0.;}
  d.values[63].kick[0]=0x1p54;d.values[63].kick[1]=1;d.values[64].kick[0]=-0x1p54;
  d.values[63].drift[0]=0x1p54;d.values[63].drift[1]=1;d.values[64].drift[0]=-0x1p54;
  const auto c=Compare(d,Seed(false,0.));ASSERT_FALSE(HasFailure());EXPECT_EQ(c.diagnostics.internal_kick_work_J,0.);EXPECT_EQ(c.diagnostics.internal_drift_work_J,0.);
}
TEST(Type13ReadTileCuda, SignedZeroAndRawFlagEncodingAreNotNormalized){
  Device d;ASSERT_TRUE(d.Initialize(65));for(std::size_t p=0;p<d.count;++p){auto&v=d.values[p];v.active=255;v.newly_failed=2;for(unsigned c=0;c<t::ChannelCount;++c)v.work[c]=v.increment[c]=-0.;for(unsigned n=0;n<2;++n)v.kick[n]=v.drift[n]=-0.;}
  const auto c=Compare(d,Seed(false,-0.));ASSERT_FALSE(HasFailure());EXPECT_EQ(Bits(c.diagnostics.internal_kick_work_J),Bits(-0.));EXPECT_EQ(c.diagnostics.active_count,103u+255u*65u);
}
TEST(Type13ReadTileCuda, FinalFiniteCheckRetainsCompleteFailedDiagnosticsAndRepairsSameArena){
  Device d;ASSERT_TRUE(d.Initialize(129));
  for(unsigned field=0;field<5;++field)for(double value:{std::numeric_limits<double>::max(),double(INFINITY),std::nan("23")}){
    d.Reset();for(unsigned p:{63u,64u}) {auto&v=d.values[p];if(field==0)v.work[3]=value;if(field==1)v.increment[5]=value;if(field==2)v.kick[1]=value;if(field==3)v.drift[0]=value;if(field==4)v.native_dt=-1;}
    const auto c=Compare(d,Seed());ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,t::BatchStatus::NonfiniteResult);EXPECT_EQ(c.diagnostics.element_count,129u);
    EXPECT_EQ(c.diagnostics.active_count,103u+86u);EXPECT_EQ(c.element,SIZE_MAX);
  }
  d.Reset();const auto c=Compare(d,Seed());ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,t::BatchStatus::Success);
}
}
