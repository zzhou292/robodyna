// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Profile.h"
#include "NativeReplay.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
#include <vector>
#include <limits>
#include <cmath>
namespace qeph_projection_test {
namespace {
struct Device {ProfileInput* in=nullptr;ProfileResult* out=nullptr;~Device(){cudaDeviceSynchronize();cudaFree(in);cudaFree(out);}};
__global__ void ProjectPacketsKernel(const ProfileInput* in,ProfileResult* out,unsigned count){const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i<count)out[i]=EvaluateProfile(in[i]);}
void Near(double a,double b){ASSERT_TRUE(std::isfinite(a));ASSERT_TRUE(std::isfinite(b));EXPECT_NEAR(a,b,256*std::numeric_limits<double>::epsilon()*std::max(1.,std::abs(b)));}
template<class T,std::size_t N>void Near(const std::array<T,N>& a,const std::array<T,N>& b){for(unsigned i=0;i<N;++i)Near(a[i],b[i]);}
void Same(const ProfileResult& a,const ProfileResult& b){ASSERT_EQ(a.rate_status,b.rate_status);ASSERT_EQ(a.force_status,b.force_status);EXPECT_EQ(a.metric_length,b.metric_length);
  if(a.rate_status!=q::Status::kSuccess)return;
  EXPECT_EQ(a.rate.projection.planar,b.rate.projection.planar);EXPECT_EQ(a.rate.projection.warped_defined,b.rate.projection.warped_defined);
  Near(a.rate.projection.z1,b.rate.projection.z1);Near(a.rate.projection.di,b.rate.projection.di);Near(a.rate.projection.db,b.rate.projection.db);Near(a.rate.projection.vqn,b.rate.projection.vqn);
  Near(a.rate.v13,b.rate.v13);Near(a.rate.v24,b.rate.v24);Near(a.rate.vhi,b.rate.vhi);Near(a.rate.rlxyz,b.rate.rlxyz);
  if(a.force_status!=q::Status::kSuccess)return;Near(a.force.force,b.force.force);Near(a.force.couple,b.force.couple);
}
}
TEST(QephProjectionCuda,ActualCapturedProfilesAcrossLaunchSizesMatchHostAndNative) {
  std::vector<ProfileInput> input;std::vector<ProfileResult> expected;
  for(const auto& row:CapturedRows())for(double length:{.001,.01,1.}){input.push_back(Profile(row,length));
    auto rate=NativeRates(Scale(row.rate_entry,.001/length));auto forces=NativeForces(Scale(row.force_entry,.001/length,rate.projection));
    rate.projection.z1*=length;for(auto* v:{&rate.v13,&rate.v24,&rate.vhi})for(auto& x:*v)x*=length;
    for(auto& moment:forces.couple)for(auto& x:moment)x*=length;
    expected.push_back({q::Status::kSuccess,q::Status::kSuccess,rate,forces,length});}
  std::vector<ProfileResult> actual(input.size());Device d;
  ASSERT_EQ(cudaMalloc(&d.in,input.size()*sizeof(ProfileInput)),cudaSuccess);ASSERT_EQ(cudaMalloc(&d.out,input.size()*sizeof(ProfileResult)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(d.in,input.data(),input.size()*sizeof(ProfileInput),cudaMemcpyHostToDevice),cudaSuccess);
  for(unsigned threads:{1u,7u,32u,64u}) {ProjectPacketsKernel<<<(input.size()+threads-1)/threads,threads>>>(d.in,d.out,input.size());ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(actual.data(),d.out,actual.size()*sizeof(ProfileResult),cudaMemcpyDeviceToHost),cudaSuccess);
    for(unsigned i=0;i<input.size();++i){SCOPED_TRACE(i);Same(actual[i],expected[i]);Same(actual[i],EvaluateProfile(input[i]));}}
}
TEST(QephProjectionCuda,MalformedAndUnrepresentableMetricPacketsRejectOnDevice) {
  std::vector<ProfileInput> input;for(double length:{0.,-1.,1e-300,1e300,1e-150,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})input.push_back(Profile(CapturedRows()[8],length));
  std::vector<ProfileResult> actual(input.size());Device d;ASSERT_EQ(cudaMalloc(&d.in,input.size()*sizeof(ProfileInput)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&d.out,input.size()*sizeof(ProfileResult)),cudaSuccess);ASSERT_EQ(cudaMemcpy(d.in,input.data(),input.size()*sizeof(ProfileInput),cudaMemcpyHostToDevice),cudaSuccess);
  ProjectPacketsKernel<<<1,32>>>(d.in,d.out,input.size());ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaMemcpy(actual.data(),d.out,actual.size()*sizeof(ProfileResult),cudaMemcpyDeviceToHost),cudaSuccess);
  for(unsigned i=0;i<input.size();++i){EXPECT_NE(actual[i].rate_status,q::Status::kSuccess);Same(actual[i],EvaluateProfile(input[i]));}
}
} // namespace qeph_projection_test
