// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#ifdef REAR18_ORIGINAL
#include "OriginalFixture.h"
#endif
#include <cuda_runtime.h>
#include <stdexcept>
#include <vector>

namespace rear_startup_test {
namespace {
template<class T> struct DeviceArray {
  T* pointer = nullptr;
  explicit DeviceArray(std::size_t count) {
    const auto status = cudaMalloc(&pointer,sizeof(T)*count);
    if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
  }
  ~DeviceArray() { cudaFree(pointer); }
  DeviceArray(const DeviceArray&) = delete;
  DeviceArray& operator=(const DeviceArray&) = delete;
};
struct Input {
  s::ReferenceInput reference;
  law::point::Material material;
  s::Vec3 velocity;
};
struct State {
  int status = -1;
  unsigned completed = 0;
  bool late_rejected = false;
  bool phase_rejected = false;
  bool initial_rejected = false;
  bool preserved = false;
};
struct Scratch {
  law::detail::ForceScratch force;
  law::Reference reference;
  law::Material material;
  law::History accepted, bad;
  law::HistoryValues bad_values;
  law::PrescribedInterval interval;
  unsigned char snapshot[sizeof(law::ForceTrial)];
};
__device__ bool SameBytes(const law::ForceTrial& value, const unsigned char* bytes) {
  const auto* current = reinterpret_cast<const unsigned char*>(&value);
  for (unsigned i = 0; i < sizeof(value); ++i) if (current[i] != bytes[i]) return false;
  return true;
}
__global__ void Calculate(const Input* inputs, unsigned parents, unsigned intervals,
    const law::PrescribedInterval* prescribed, const double* strain, const double* yield, unsigned knots, bool faults,
    Scratch* workspace, law::ForceTrial* values, State* states) {
  const unsigned worker = blockIdx.x*blockDim.x+threadIdx.x;
  auto& scratch = workspace[worker];
  for (unsigned row = worker; row < parents; row += gridDim.x*blockDim.x) {
    const auto& in = inputs[row];
    State state;
    auto& reference = scratch.reference;
    auto& material = scratch.material;
    if (law::InitializeReference(in.reference,reference) != s::Status::Success ||
        law::point::Prepare(in.material,{strain,yield,knots},material) != law::point::Status::Ok) {
      states[row] = state;
      continue;
    }
    auto& initial = values[row*(intervals+1)];
    auto status = law::detail::InitializeForceScratch(reference,material,in.velocity,scratch.force);
    if (status != s::Status::Success) { state.status = int(status); states[row] = state; continue; }
    initial = scratch.force.trial;
    scratch.accepted = initial.proposed_history;
    state.completed = 1;
    for (unsigned step = 0; step < intervals; ++step) {
      auto& interval = scratch.interval;
      interval = prescribed[row*intervals+step];
      interval.base_time_s = scratch.accepted.stamp().time_s;
      auto& output = values[row*(intervals+1)+step+1];
      status = law::detail::EvaluateForceScratch(reference,scratch.accepted,interval,material,scratch.force);
      if (status != s::Status::Success) break;
      output = scratch.force.trial;
      if (faults && step == 8) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&output);
        for (unsigned i = 0; i < sizeof(output); ++i) scratch.snapshot[i] = bytes[i];
        scratch.bad_values = scratch.accepted.data();
        scratch.bad_values.point[7].material.stress_pa[5] = 1e200;
        status = law::PreparePrescribedHistory(reference,material,scratch.bad_values,
                                               scratch.accepted.stamp(),scratch.bad);
        if (status != s::Status::Success) break;
        status = law::detail::EvaluateForceScratch(reference,scratch.bad,interval,material,scratch.force);
        state.late_rejected = status == s::Status::NonfiniteResult;
        if (status == s::Status::Success) output = scratch.force.trial;
        ++interval.sample_index;
        status = law::detail::EvaluateForceScratch(reference,scratch.accepted,interval,material,scratch.force);
        state.phase_rejected = status == s::Status::InvalidInput;
        if (status == s::Status::Success) output = scratch.force.trial;
        --interval.sample_index;
        status = law::detail::InitializeForceScratch(reference,material,{0,0,INFINITY},scratch.force);
        state.initial_rejected = status == s::Status::InvalidInput;
        if (status == s::Status::Success) output = scratch.force.trial;
        state.preserved = SameBytes(output,scratch.snapshot);
        status = law::detail::EvaluateForceScratch(reference,scratch.accepted,interval,material,scratch.force);
        if (status != s::Status::Success) break;
        output = scratch.force.trial;
      }
      scratch.accepted = output.proposed_history;
      ++state.completed;
    }
    state.status = int(status);
    states[row] = state;
  }
}
void DeviceCheck(const std::vector<Input>& input, unsigned intervals, bool faults) {
  const unsigned count = static_cast<unsigned>(input.size());
  const unsigned threads = std::min(count,64u);
  DeviceArray<Input> device_input(count);
  DeviceArray<law::PrescribedInterval> device_intervals(count*intervals);
  DeviceArray<double> x(law44_solid_test::Count), y(law44_solid_test::Count);
  DeviceArray<Scratch> scratch(threads);
  DeviceArray<law::ForceTrial> results(count*(intervals+1));
  DeviceArray<State> states(count);
  std::vector<law::PrescribedInterval> prescribed(count*intervals);
  for (unsigned row = 0; row < count; ++row) {
    law::Reference reference;
    ASSERT_EQ(law::InitializeReference(input[row].reference,reference),s::Status::Success);
    for (unsigned step = 0; step < intervals; ++step) prescribed[row*intervals+step] = Path(reference,step,true);
  }
  ASSERT_EQ(cudaMemcpy(device_input.pointer,input.data(),count*sizeof(Input),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device_intervals.pointer,prescribed.data(),prescribed.size()*sizeof(prescribed[0]),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(x.pointer,law44_solid_test::X,sizeof(law44_solid_test::X),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(y.pointer,law44_solid_test::Y,sizeof(law44_solid_test::Y),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemset(scratch.pointer,0,threads*sizeof(Scratch)),cudaSuccess);
  Calculate<<<1,threads>>>(device_input.pointer,count,intervals,device_intervals.pointer,x.pointer,y.pointer,
      law44_solid_test::Count,faults,scratch.pointer,results.pointer,states.pointer);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  std::vector<law::ForceTrial> actual(count*(intervals+1));
  std::vector<State> state(count);
  ASSERT_EQ(cudaMemcpy(actual.data(),results.pointer,actual.size()*sizeof(actual[0]),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(state.data(),states.pointer,count*sizeof(State),cudaMemcpyDeviceToHost),cudaSuccess);
  for (unsigned row = 0; row < count; ++row) {
    SCOPED_TRACE(input[row].reference.source_element_id);
    ASSERT_EQ(state[row].status,int(s::Status::Success));
    ASSERT_EQ(state[row].completed,intervals+1);
    if (faults) {
      EXPECT_TRUE(state[row].late_rejected);
      EXPECT_TRUE(state[row].phase_rejected);
      EXPECT_TRUE(state[row].initial_rejected);
      EXPECT_TRUE(state[row].preserved);
    }
    law::Reference reference;
    law::Material material;
    ASSERT_EQ(law::InitializeReference(input[row].reference,reference),s::Status::Success);
    ASSERT_EQ(law::point::Prepare(input[row].material,
        {law44_solid_test::X,law44_solid_test::Y,law44_solid_test::Count},material),law::point::Status::Ok);
    auto native = NativeInitialize(material,input[row].reference,input[row].velocity);
    ASSERT_EQ(native.status,0);
    ASSERT_TRUE(Agree(actual[row*(intervals+1)],native));
    for (unsigned step = 0; step < intervals; ++step) {
      SCOPED_TRACE(step);
      auto interval = Path(reference,step,true);
      interval.base_time_s = actual[row*(intervals+1)+step].proposed_history.stamp().time_s;
      native = Native(material,native.next,interval);
      ASSERT_EQ(native.status,0);
      ASSERT_TRUE(Agree(actual[row*(intervals+1)+step+1],native));
      EXPECT_EQ(actual[row*(intervals+1)+step+1].proposed_history.stamp().sample_index,step+1);
    }
  }
}
}  // namespace
TEST(Rear18StartupCuda, ExternalScratchConstructorRecurrenceAndLateFailureRetry) {
  std::vector<Input> input;
  for (bool collapsed : {false,true})
    for (const s::Vec3 velocity : {s::Vec3{},s::Vec3{11.123,-.37,.129}})
      input.push_back({Reference(collapsed).input(),law44_solid_test::Material(),velocity});
  DeviceCheck(input,32,true);
}
#ifdef REAR18_ORIGINAL
TEST(Rear18StartupCuda, EveryOriginalConstructorAndFirstPositiveInterval) {
  std::vector<Input> input;
  for (unsigned row = 0; row < std::size(rear18_test::original::Cells); ++row) {
    const auto source = rear18_test::original::Input(row);
    input.push_back({source,rear_force_test::OriginalMaterial(source.source_part_id),{11.123,-.37,.129}});
  }
  ASSERT_EQ(input.size(),306u);
  DeviceCheck(input,1,false);
}
#endif
}  // namespace rear_startup_test
