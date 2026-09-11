// SPDX-License-Identifier: AGPL-3.0-or-later
#include <cfloat>
#include "DeviceFixture.cuh"
#include "NativeOracle.h"
#include "PrescribedPath.h"

namespace heph_test {
TEST(HephForceCuda, DeviceOwnedMovingHistoriesMatchIndependentNativeFor32Intervals) {
  auto reversed=solid24_test::Distorted();
  for(unsigned n=0;n<4;++n)std::swap(reversed.position_m[n],reversed.position_m[n+4]);
  DeviceArray<s::ReferenceInput> device_input;
  DeviceArray<s::PrescribedInterval> device_interval;
  DeviceArray<DeviceState> device;
  for(auto input:{solid24_test::Brick(),solid24_test::Distorted(),reversed}) {
    input.profile.reference_strain=s::ReferenceStrain::TotalLagrangian10;
    const auto reference=Reference(input);
    const auto material=Material(input.density_kg_m3);
    auto native=InitializeNative(input);
    ASSERT_EQ(cudaMemcpy(device_input.data,&input,sizeof(input),cudaMemcpyHostToDevice),cudaSuccess);
    InitializeDevice<<<1,1>>>(device_input.data,device.data,1);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    DeviceState actual;
    ASSERT_EQ(cudaMemcpy(&actual,device.data,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(actual.status,s::ForceStatus::Success);
    for(unsigned step=1;step<=32;++step) {
      SCOPED_TRACE(step);
      auto interval=Path(reference,step);
      interval.base_time_s=actual.history.stamp().time_s;
      ASSERT_EQ(cudaMemcpy(device_interval.data,&interval,sizeof(interval),cudaMemcpyHostToDevice),cudaSuccess);
      AdvanceDevice<<<1,1>>>(device.data,device_interval.data,1);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaMemcpy(&actual,device.data,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
      ASSERT_EQ(actual.status,s::ForceStatus::Success);
      const auto expected=NativeStep(native,interval,material);
      Compare(actual.trial,expected);
      EXPECT_EQ(actual.history.stamp().sample_index,step);
      EXPECT_TRUE(s::force_detail::SameReference(actual.history.reference(),reference));
      AcceptNative(expected,native);
    }
  }
}
TEST(HephForceCuda, LateOverflowIdentityCutoffAndRetryPreserveAllNamedFields) {
  auto input=solid24_test::Distorted();
  input.profile.reference_strain=s::ReferenceStrain::TotalLagrangian10;
  const auto reference=Reference(input);
  const auto interval=Path(reference,1);
  DeviceArray<s::ReferenceInput> device_input;
  DeviceArray<s::PrescribedInterval> device_interval;
  DeviceArray<DeviceState> device;
  DeviceArray<DeviceRejections> rejected;
  ASSERT_EQ(cudaMemcpy(device_input.data,&input,sizeof(input),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device_interval.data,&interval,sizeof(interval),cudaMemcpyHostToDevice),cudaSuccess);
  InitializeDevice<<<1,1>>>(device_input.data,device.data,1);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  AdvanceDevice<<<1,1>>>(device.data,device_interval.data,1);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  DeviceState before,after;
  ASSERT_EQ(cudaMemcpy(&before,device.data,sizeof(before),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(before.status,s::ForceStatus::Success);
  const auto native=InitializeNative(input);
  Compare(before.trial,NativeStep(native,interval,Material(input.density_kg_m3)));
  auto cutoff=Material(input.density_kg_m3);cutoff.tension_cutoff_pa=1;
  ASSERT_EQ(NativeStep(native,interval,cutoff).status,1);
  RejectAndRetry<<<1,1>>>(device.data,interval,rejected.data);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  DeviceRejections result;
  ASSERT_EQ(cudaMemcpy(&result,rejected.data,sizeof(result),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&after,device.data,sizeof(after),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_NE(result.overflow_status,s::ForceStatus::Success);
  EXPECT_EQ(result.identity_status,s::ForceStatus::ReferenceMismatch);
  EXPECT_EQ(result.cutoff_status,s::ForceStatus::MaterialFailure);
  EXPECT_EQ(result.retry_status,s::ForceStatus::Success);
  for(const auto* trial:{&result.overflow,&result.identity,&result.cutoff,&result.retry})
    EXPECT_TRUE(Same(*trial,before.trial));
  EXPECT_TRUE(Same(after.trial,before.trial));
  EXPECT_EQ(after.history.stamp().sample_index,before.history.stamp().sample_index);
}
} // namespace heph_test
