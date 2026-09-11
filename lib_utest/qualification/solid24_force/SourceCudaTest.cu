// SPDX-License-Identifier: AGPL-3.0-or-later
#include <cfloat>
#include "DeviceFixture.cuh"
#include "NativeOracle.h"
#include "PrescribedPath.h"
#include "../solid24_reference/SourceFixture.h"

namespace heph_test {
TEST(HephForceSourceCuda, All1309DeviceOwnedHistoriesMatchThreeNativeIntervals) {
  std::vector<s::ReferenceInput> input;
  std::vector<s::Reference> reference;
  std::vector<NativeHistory> native;
  for(unsigned index=0;index<solid24_test::SourceCount;++index) {
    const auto source=solid24_test::Source(index);
    if(!solid24_test::IsBrick(source))continue;
    input.push_back(solid24_test::TotalReference(source,s::WorkingLengthUnit::Millimetre));
    reference.push_back(Reference(input.back()));
    native.push_back(InitializeNative(input.back()));
  }
  ASSERT_EQ(input.size(),1309u);
  const unsigned count=input.size();
  DeviceArray<s::ReferenceInput> device_input(count);
  DeviceArray<s::PrescribedInterval> device_interval(count);
  DeviceArray<DeviceState> device(count);
  ASSERT_EQ(cudaMemcpy(device_input.data,input.data(),count*sizeof(input[0]),cudaMemcpyHostToDevice),cudaSuccess);
  InitializeDevice<<<(count+31)/32,32>>>(device_input.data,device.data,count);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  std::vector<DeviceState> actual(count);
  ASSERT_EQ(cudaMemcpy(actual.data(),device.data,count*sizeof(actual[0]),cudaMemcpyDeviceToHost),cudaSuccess);
  for(const auto& state:actual)ASSERT_EQ(state.status,s::ForceStatus::Success);
  std::vector<s::PrescribedInterval> interval(count);
  for(unsigned step=1;step<=3;++step) {
    for(unsigned n=0;n<count;++n) {
      interval[n]=Path(reference[n],step,1e-8);
      interval[n].base_time_s=actual[n].history.stamp().time_s;
    }
    ASSERT_EQ(cudaMemcpy(device_interval.data,interval.data(),count*sizeof(interval[0]),cudaMemcpyHostToDevice),cudaSuccess);
    AdvanceDevice<<<(count+31)/32,32>>>(device.data,device_interval.data,count);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(actual.data(),device.data,count*sizeof(actual[0]),cudaMemcpyDeviceToHost),cudaSuccess);
    for(unsigned n=0;n<count;++n) {
      SCOPED_TRACE(input[n].source_element_id);
      SCOPED_TRACE(step);
      ASSERT_EQ(actual[n].status,s::ForceStatus::Success);
      const auto expected=NativeStep(native[n],interval[n],Material(input[n].density_kg_m3));
      Compare(actual[n].trial,expected);
      EXPECT_EQ(actual[n].history.stamp().sample_index,step);
      EXPECT_TRUE(s::force_detail::SameReference(actual[n].history.reference(),reference[n]));
      for(unsigned slot=0;slot<8;++slot)
        EXPECT_EQ(actual[n].reference.source_slot(slot),unsigned(native[n].reference.permutation[slot]));
      AcceptNative(expected,native[n]);
    }
  }
  RecordProperty("mapped_original_heph_device_parents",count);
}
} // namespace heph_test
