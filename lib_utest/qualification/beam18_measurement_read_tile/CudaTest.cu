// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DeviceFixture.cuh"
#include "ReferenceFinalize.cuh"
#include "lib_src/elements/beam18/resident/measurement/Finalize.cuh"
namespace beam18_read_tile_test {
__global__ void Current(d::Storage*s,fe::NodalPreparedView v,b::BatchDiagnostics seed,bool initial){__shared__ tile::Tile stage;tile::Finalize(*s,0,initial?0:1,v,seed,initial,stage);}
void Drain(){ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);}
d::Control Compare(Device&x,b::BatchDiagnostics seed=Seed(),bool initial=false){
  x.state->control=Poison();d::read_tile_reference::Finalize<<<1,1>>>(x.state,0,initial?0:1,x.view,seed,initial);Drain();const auto expected=x.state->control;
  x.state->control=Poison();Current<<<1,tile::Threads>>>(x.state,x.view,seed,initial);Drain();Same(x.state->control,expected);return x.state->control;
}
TEST(Beam18ReadTileCuda, EveryControlFieldMatchesInitialCandidateAndTileTails){
  for(std::size_t n:{0u,1u,2u,31u,32u,63u,64u,65u,127u,128u,129u,142u,193u}){SCOPED_TRACE(n);Device x;ASSERT_TRUE(x.Initialize(n));
    for(bool initial:{false,true})for(bool valid:{false,true}){const auto c=Compare(x,Seed(valid,-0.),initial);ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,b::BatchStatus::Success);}}
}
TEST(Beam18ReadTileCuda, PerParentPriorityRejectsBeforeLaterStatusAndPreservesExactPrefix){
  Device x;ASSERT_TRUE(x.Initialize(129));x.now[0].geometry.length_m=-1;x.status[128]=int(b::Status::InvalidInput);
  auto c=Compare(x);ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,b::BatchStatus::NonfiniteResult);EXPECT_EQ(c.parent,0u);
  ASSERT_TRUE(x.Reset());x.now[63].diagnostics.internal_work_increment_j[0]=std::numeric_limits<double>::max();x.now[64].diagnostics.internal_work_increment_j[0]=std::numeric_limits<double>::max();x.status[128]=int(b::Status::InvalidInput);
  c=Compare(x);ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,b::BatchStatus::NonfiniteResult);EXPECT_EQ(c.parent,64u);
  for(unsigned p:{0u,63u,64u,65u,128u}){ASSERT_TRUE(x.Reset());x.status[p]=int(b::Status::InvalidInput);x.now[p]={};c=Compare(x);ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,b::BatchStatus::ElementFailure);EXPECT_EQ(c.parent,p);}
  ASSERT_TRUE(x.Reset());c=Compare(x);ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,b::BatchStatus::Success);
}
TEST(Beam18ReadTileCuda, OrderedNativeAndEndpointCancellationUsesNoSubtotals){
  Device x;ASSERT_TRUE(x.Initialize(129));for(unsigned first:{0u,62u,63u,64u,126u})for(double seed:{0.,-0.,0x1p54,-0x1p54}){
    ASSERT_TRUE(x.Reset());for(std::size_t p=0;p<x.count;++p)for(auto&v:x.now[p].diagnostics.internal_work_increment_j)v=0.;
    const double terms[]{0x1p54,1,-0x1p54};for(unsigned i=0;i<3;++i)for(auto&v:x.now[first+i].diagnostics.internal_work_increment_j)v=terms[i];
    Compare(x,Seed(true,seed));ASSERT_FALSE(HasFailure());}
  ASSERT_TRUE(x.Reset());x.old[63].rhs_force_n[0].x=0x1p54;x.old[63].rhs_force_n[1].x=1.;x.old[64].rhs_force_n[0].x=-0x1p54;
  auto c=Compare(x);ASSERT_FALSE(HasFailure());EXPECT_EQ(c.diagnostics.internal_kick_work_j,0.);
}
TEST(Beam18ReadTileCuda, EndpointFailureKeepsBothWorkAccumulatorsUnpublished){
  Device x;ASSERT_TRUE(x.Initialize(65));const auto seed=Seed(false,7.);
  for(unsigned fault=0;fault<3;++fault){ASSERT_TRUE(x.Reset());x.old[0].rhs_force_n[0].x=1.;x.old[0].rhs_force_n[1].x=std::numeric_limits<double>::max();
    const_cast<double*>(x.view.base_kinematics.velocity_xyz)[3]=const_cast<double*>(x.view.kinematics.velocity_xyz)[3]=2.;
    if(fault==1)const_cast<double*>(x.view.kinematics.velocity_xyz)[3]=std::nan("7");
    if(fault==2)const_cast<double*>(x.view.base_kinematics.orientation_wxyz)[4]=2.;
    const auto c=Compare(x,seed);ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,b::BatchStatus::NonfiniteResult);EXPECT_EQ(c.parent,0u);
    EXPECT_EQ(Bits(c.diagnostics.internal_kick_work_j),Bits(seed.internal_kick_work_j));EXPECT_EQ(Bits(c.diagnostics.internal_drift_work_j),Bits(seed.internal_drift_work_j));}
}
TEST(Beam18ReadTileCuda, InvalidResultAndRawSignedZeroKeepOriginalFiniteChecks){
  Device x;ASSERT_TRUE(x.Initialize(129));
  for(unsigned p:{0u,63u,64u,65u,128u})for(unsigned fault=0;fault<4;++fault){ASSERT_TRUE(x.Reset());
    if(fault==0)x.now[p].diagnostics.minimum_unscaled_dt_s=0.;if(fault==1)x.now[p].diagnostics.plastic_work_increment_j=-1.;
    if(fault==2)x.now[p].rhs_force_n[1].y=std::nan("17");if(fault==3)x.now[p].point[3].tangent_factor=2.;
    const auto c=Compare(x,Seed(true,-0.));ASSERT_FALSE(HasFailure());EXPECT_EQ(c.status,b::BatchStatus::NonfiniteResult);EXPECT_EQ(c.parent,p);}
  ASSERT_TRUE(x.Reset());for(std::size_t p=0;p<x.count;++p)for(auto&v:x.now[p].diagnostics.internal_work_increment_j)v=-0.;
  const auto c=Compare(x,Seed(false,-0.));ASSERT_FALSE(HasFailure());EXPECT_EQ(Bits(c.diagnostics.native_internal_work_increment_j[0]),Bits(-0.));
}
}
