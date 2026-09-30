// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "Observation.h"
#include <cuda_runtime.h>
#include <cfloat>
#include <memory>

namespace type45_test {
constexpr unsigned Steps=32, Profiles=6;
struct DeviceInput {
  Property property;
  GeometryInput geometry;
  DampingEndpoint damping[2];
  AutomaticStiffnessContext context;
};
struct DeviceOutput {
  Reference reference;
  Interval interval[Steps];
  Evaluation value[Steps];
  Status status=Status::InvalidHistory;
  bool rollback=false, retry=false;
};
__global__ void Recurrence(const DeviceInput* input,DeviceOutput* output,bool reject) {
  const unsigned profile=threadIdx.x;
  if(profile>=Profiles) return;
  const auto& f=input[profile];
  auto& out=output[profile];
  Reference reference;
  out.status=Reference::Prepare(f.property,f.geometry,f.damping,f.context,reference);
  if(out.status!=Status::Success) return;
  out.reference=reference;
  History accepted;
  out.status=History::Initialize(reference,accepted);
  if(out.status!=Status::Success) return;
  for(unsigned i=0;i<Steps;++i) {
    Interval step{{f.geometry.position_m[0],f.geometry.position_m[1]},
      {{.13,-.07,.03},{.23,.19,-.11}},accepted.stamp().time_s,1e-4,accepted.stamp().sample_index+1};
    step.position_m[1].x+=2e-5*::sin(.2*(i+1));
    step.position_m[1].y-=3e-5*::cos(.17*(i+1));
    Evaluation value;
    out.status=Evaluate(reference,accepted,step,value);
    if(out.status!=Status::Success) return;
    if(reject && i==17) {
      const auto saved=value;
      auto bad=step;
      bad.position_m[1].z=DBL_MAX/4;
      const auto failed=Evaluate(reference,accepted,bad,value);
      out.rollback=failed==Status::NonfiniteResult && EqualObservation(saved,value) &&
        value.history.Matches(reference) && accepted.stamp().sample_index==17;
      out.status=Evaluate(reference,accepted,step,value);
      out.retry=out.status==Status::Success && EqualObservation(saved,value);
      if(!out.rollback || !out.retry) return;
    }
    out.interval[i]=step;
    out.value[i]=value;
    accepted=value.history;
  }
}
void RunDevice(bool reject) {
  int devices=0;
  ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);
  ASSERT_GT(devices,0);
  auto inputs=std::make_unique<DeviceInput[]>(Profiles);
  auto outputs=std::make_unique<DeviceOutput[]>(Profiles);
  Fixture fixture[Profiles];
  for(unsigned i=0;i<Profiles;++i) {
    fixture[i]=Fixture(static_cast<Kind>(i%3+1));
    fixture[i].property.working_units=i<3 ? WorkingUnits::SI : WorkingUnits::MillimetreTonneSecond;
    if(i==4) fixture[i].context.main[1].inertia_kg_m2=0;
    auto& in=inputs[i];
    in.property=fixture[i].property;
    in.geometry=fixture[i].geometry;
    in.context=fixture[i].context;
    for(unsigned j=0;j<2;++j) in.damping[j]=fixture[i].damping[j];
  }
  DeviceInput* device_input=nullptr;
  DeviceOutput* device_output=nullptr;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device_input),Profiles*sizeof(DeviceInput)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device_output),Profiles*sizeof(DeviceOutput)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device_input,inputs.get(),Profiles*sizeof(DeviceInput),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device_output,outputs.get(),Profiles*sizeof(DeviceOutput),cudaMemcpyHostToDevice),cudaSuccess);
  Recurrence<<<1,Profiles>>>(device_input,device_output,reject);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(outputs.get(),device_output,Profiles*sizeof(DeviceOutput),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaFree(device_input),cudaSuccess);
  ASSERT_EQ(cudaFree(device_output),cudaSuccess);
  for(unsigned i=0;i<Profiles;++i) {
    SCOPED_TRACE(i);
    const auto& output=outputs[i];
    ASSERT_EQ(output.status,Status::Success);
    if(reject) {
      ASSERT_TRUE(output.rollback);
      ASSERT_TRUE(output.retry);
    }
    int status=-1;
    NativeOracle native(fixture[i],status);
    ASSERT_EQ(status,0);
    CompareReference(fixture[i],output.reference,native);
    double previous_time=0;
    for(unsigned step=0;step<Steps;++step) {
      SCOPED_TRACE(step);
      EXPECT_EQ(output.interval[step].sample_index,step+1);
      EXPECT_EQ(output.interval[step].base_time_s,previous_time);
      type45_native::Step expected;
      ASSERT_TRUE(native.Step(output.interval[step],expected));
      CompareStep(output.value[step],native,expected);
      previous_time=output.value[step].history.stamp().time_s;
      EXPECT_EQ(output.value[step].history.stamp().sample_index,step+1);
    }
  }
}
TEST(Type45Cuda, DeviceOwnedThreeKindHistoriesMatchCompleteNativeRecurrence) { RunDevice(false); }
TEST(Type45Cuda, LateForceOverflowPreservesHistoryAndExactRetry) { RunDevice(true); }
} // namespace type45_test
