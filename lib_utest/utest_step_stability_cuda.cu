#include "lib_src/collision/SurfaceContactMass.h"
#include "lib_src/solvers/ExplicitStepStability.h"
#include "lib_src/solvers/FENodalStateView.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <cmath>

namespace {
namespace sc=tlfea::contact;
namespace st=tl::fea::stability;
using Status=sc::Status;
template<class T> struct Device {
  Device():status(cudaMalloc(&pointer,sizeof(T))) {}
  ~Device() { if(pointer) cudaFree(pointer); }
  Device(const Device&)=delete;Device& operator=(const Device&)=delete;
  T* pointer=nullptr;cudaError_t status;
};
struct Pipeline {
  Status status[5]{};
  sc::NormalJacobian jacobian;
  st::StepLimit limit;
  double stiffness[4]{},damping[4]{};
  double force[3][4]{},couple[3][4]{};
  bool current=false;
};

// One serialized work item: arithmetic parity and real TL physical force-view
// consumption. This does not qualify concurrent atomic scatter or a timestepper.
__global__ void Assemble(Pipeline* out) {
  *out={};
  const double inverse[4]={1,.5,.25,.125};const std::uint8_t fixed[4]={};
  const sc::LumpedTranslationMassView mass{inverse,fixed,4,7,sc::TranslationMassModel::kIsotropicLumped};
  const sc::SurfaceTriangle a{{0,1,2},1,1,0,0,sc::SurfaceInterpolation::kLinearTriangle};
  const sc::SurfaceTriangle b{{1,2,3},2,2,0,0,sc::SurfaceInterpolation::kLinearTriangle};
  const double weights[3]={.25,.5,.25};
  out->status[0]=sc::BuildLinearTriangleNormalJacobian(mass,a,weights,&b,weights,{.6,0,.8},1,&out->jacobian);
  st::RowBounds rows{out->stiffness,out->damping,4,4};
  out->status[1]=st::ResetRows(&rows,7,1);
  const tl::fea::DeviceNodalForceView force{
    out->force[0],out->force[1],out->force[2],out->couple[0],out->couple[1],out->couple[2],4,7};
  // The coordinator clears once, before both structural and contact batches.
  for(int i=0;i<4;++i) {
    force.force_x[i]=force.force_y[i]=force.force_z[i]=0;
    force.couple_x[i]=force.couple_y[i]=force.couple_z[i]=0;
  }
  for(int batch=0;batch<2;++batch) {
    st::RowContribution contribution;
    auto status=st::MakeRankOneContribution(out->jacobian,batch?200:100,batch?2:1,&contribution);
    if(status==Status::kOk) status=st::AccumulateRows(&rows,contribution);
    out->status[2+batch]=status;
    if(status!=Status::kOk) { st::InvalidateRows(&rows);continue; }
    const double magnitude=batch?5:3;
    for(unsigned j=0;j<out->jacobian.count;++j) {
      const auto node=out->jacobian.nodes[j];const auto value=sc::Scale(out->jacobian.values[j],magnitude);
      force.force_x[node]+=value.x;force.force_y[node]+=value.y;force.force_z[node]+=value.z;
    }
  }
  out->status[4]=st::FinalizeRows(&rows,.8,1e-8,1,&out->limit);
  out->current=st::IsCurrentLimit(rows,out->limit);
}

struct Failures {
  Status cancellation{},mass_overflow{},row_overflow{},failed_finalize{},retry{},minimum{};
  bool preserved=false,invalid=false,current=false;
  double old_dt=0,retry_dt=0;
};
__global__ void RejectAndRetry(Failures* out) {
  *out={};
  const double inverse[2]={1,1};const std::uint8_t fixed[2]={};
  const sc::LumpedTranslationMassView mass{inverse,fixed,2,7,sc::TranslationMassModel::kIsotropicLumped};
  const sc::SignedNodeWeight cancelled[2]={{0,1},{0,-1}},huge[1]={{0,1e200}},good[1]={{0,1}};
  sc::NormalJacobian j;
  out->cancellation=sc::BuildNormalJacobian(mass,cancelled,2,{1,0,0},1,&j);
  out->mass_overflow=sc::BuildNormalJacobian(mass,huge,1,{1,0,0},1,&j);
  double k[2],c[2];st::RowBounds rows{k,c,2,2};st::ResetRows(&rows,7,1);
  k[0]=1;k[1]=DBL_MAX/2;
  st::RowContribution bad;bad.valid=true;bad.count=2;bad.nodes[0]=0;bad.nodes[1]=1;
  bad.base_epoch=7;bad.attempt=1;bad.stiffness[0]=2;bad.stiffness[1]=DBL_MAX/2;
  out->row_overflow=st::AccumulateRows(&rows,bad);
  out->preserved=k[0]==1 && k[1]==DBL_MAX/2 && c[0]==0 && c[1]==0;
  out->invalid=!rows.valid;
  st::StepLimit limit;out->failed_finalize=st::FinalizeRows(&rows,.8,1e-8,1,&limit);
  st::ResetRows(&rows,7,2);
  sc::BuildNormalJacobian(mass,good,1,{1,0,0},2,&j);
  st::RowContribution valid;st::MakeRankOneContribution(j,100,0,&valid);
  st::AccumulateRows(&rows,valid);out->retry=st::FinalizeRows(&rows,.8,1e-8,1,&limit);
  out->retry_dt=limit.dt;out->old_dt=limit.dt;
  st::ResetRows(&rows,7,3);out->current=st::IsCurrentLimit(rows,limit);
  k[0]=1e12;out->minimum=st::FinalizeRows(&rows,.8,1e-5,1,&limit);
}

class StepStabilityCuda:public ::testing::Test {
  void SetUp() override { int count=0;ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0); }
};
TEST_F(StepStabilityCuda, SharedMassAndCombinedBoundsMatchHostAndSingleGlobalForceAssembly) {
  Device<Pipeline> device;ASSERT_EQ(device.status,cudaSuccess);
  Assemble<<<1,1>>>(device.pointer);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Pipeline result;ASSERT_EQ(cudaMemcpy(&result,device.pointer,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
  for(auto status:result.status) ASSERT_EQ(status,Status::kOk);
  EXPECT_TRUE(result.current);EXPECT_NEAR(result.jacobian.inverse_effective_mass,.1171875,2e-15);
  const double inverse[4]={1,.5,.25,.125};const std::uint8_t fixed[4]={};
  const sc::LumpedTranslationMassView mass{inverse,fixed,4,7,sc::TranslationMassModel::kIsotropicLumped};
  const sc::SignedNodeWeight weights[4]={{0,.25},{1,.25},{2,-.25},{3,-.25}};
  sc::NormalJacobian j;ASSERT_EQ(sc::BuildNormalJacobian(mass,weights,4,{.6,0,.8},1,&j),Status::kOk);
  double k[4],c[4];st::RowBounds rows{k,c,4,4};ASSERT_EQ(st::ResetRows(&rows,7,1),Status::kOk);
  for(int batch=0;batch<2;++batch) {
    st::RowContribution contribution;ASSERT_EQ(st::MakeRankOneContribution(j,batch?200:100,batch?2:1,&contribution),Status::kOk);
    ASSERT_EQ(st::AccumulateRows(&rows,contribution),Status::kOk);
  }
  st::StepLimit expected;ASSERT_EQ(st::FinalizeRows(&rows,.8,1e-8,1,&expected),Status::kOk);
  EXPECT_NEAR(result.limit.dt,expected.dt,2e-14);
  for(int i=0;i<4;++i) {
    EXPECT_NEAR(result.stiffness[i],k[i],2e-12);EXPECT_NEAR(result.damping[i],c[i],2e-14);
    const double weight=i<2?.25:-.25;
    EXPECT_NEAR(result.force[0][i],8*.6*weight,2e-14);EXPECT_DOUBLE_EQ(result.force[1][i],0);
    EXPECT_NEAR(result.force[2][i],8*.8*weight,2e-14);
    for(int axis=0;axis<3;++axis) EXPECT_DOUBLE_EQ(result.couple[axis][i],0);
  }
}
TEST_F(StepStabilityCuda, ActualOverflowInvalidatesScratchAndCleanRetryIsBounded) {
  Device<Failures> device;ASSERT_EQ(device.status,cudaSuccess);
  RejectAndRetry<<<1,1>>>(device.pointer);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  Failures result;ASSERT_EQ(cudaMemcpy(&result,device.pointer,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(result.cancellation,Status::kNoDynamicDofs);EXPECT_EQ(result.mass_overflow,Status::kNonFiniteResult);
  EXPECT_EQ(result.row_overflow,Status::kNonFiniteResult);EXPECT_EQ(result.failed_finalize,Status::kNoTrial);
  EXPECT_TRUE(result.preserved);EXPECT_TRUE(result.invalid);EXPECT_EQ(result.retry,Status::kOk);
  EXPECT_NEAR(result.retry_dt,.16,2e-14);EXPECT_GT(result.old_dt,0);EXPECT_FALSE(result.current);
  EXPECT_EQ(result.minimum,Status::kOutOfRange);
}
}  // namespace
