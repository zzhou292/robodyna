// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "Trajectory.h"
#ifdef REAR18_ORIGINAL
#include "OriginalFixture.h"
#endif
#include <cuda_runtime.h>
#include <stdexcept>
#include <vector>

namespace rear_force_test {
namespace {
template<class T> struct DeviceArray {
  T* pointer = nullptr;
  explicit DeviceArray(unsigned count) {
    const auto status = cudaMalloc(&pointer,sizeof(T)*count);
    if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
  }
  ~DeviceArray() { cudaFree(pointer); }
  DeviceArray(const DeviceArray&) = delete;
  DeviceArray& operator=(const DeviceArray&) = delete;
};
struct DeviceStatus {
  int status = -1;
  unsigned completed = 0;
  bool late_rejected = false;
  bool phase_rejected = false;
  bool output_preserved = false;
};
// Explicit one-thread fixture storage; do not reserve a large CUDA stack for
// every resident hardware thread just to retain retry snapshots and histories.
struct DeviceScratch {
  law::Reference reference;
  law::Material material;
  law::History accepted;
  law::HistoryValues bad_values;
  law::History bad;
  s::PrescribedInterval wrong_phase;
  unsigned char snapshot[sizeof(law::ForceTrial)]{};
};
__device__ bool SameBytes(const law::ForceTrial& a, const unsigned char* y) {
  const auto* x = reinterpret_cast<const unsigned char*>(&a);
  for (unsigned i = 0; i < sizeof(a); ++i) {
    if (x[i] != y[i]) return false;
  }
  return true;
}
__global__ void Trajectory(law::ReferenceInput input, const double* strain, const double* yield,
    unsigned points, law::point::Material source_material, const s::PrescribedInterval* intervals, unsigned count, bool faults,
    law::ForceTrial* results, DeviceStatus* output, DeviceScratch* scratch) {
  auto& reference = scratch->reference;
  auto& material = scratch->material;
  auto& accepted = scratch->accepted;
  DeviceStatus state;
  if (law::InitializeReference(input,reference) != s::Status::Success ||
      law::point::Prepare(source_material,{strain,yield,points},material) != law::point::Status::Ok ||
      law::InitializeHistory(reference,material,accepted) != s::Status::Success) {
    *output = state;
    return;
  }
  for (unsigned step = 0; step < count; ++step) {
    auto status = law::EvaluateForce(reference,accepted,intervals[step],material,results[step]);
    if (status != s::Status::Success) {
      state.status = static_cast<int>(status);
      *output = state;
      return;
    }
    if (faults && step == 20) {
      auto& snapshot = scratch->snapshot;
      const auto* original = reinterpret_cast<const unsigned char*>(&results[step]);
      for (unsigned i = 0; i < sizeof(snapshot); ++i) snapshot[i] = original[i];
      auto& bad_values = scratch->bad_values;
      bad_values = accepted.data();
      bad_values.point[7].material.stress_pa[5] = 1e200;
      auto& bad = scratch->bad;
      if (law::PreparePrescribedHistory(reference,material,bad_values,accepted.stamp(),bad) !=
          s::Status::Success) {
        *output = state;
        return;
      }
      state.late_rejected = law::EvaluateForce(reference,bad,intervals[step],material,results[step]) !=
                            s::Status::Success;
      state.output_preserved = SameBytes(results[step],snapshot);
      auto& wrong_phase = scratch->wrong_phase;
      wrong_phase = intervals[step];
      ++wrong_phase.sample_index;
      state.phase_rejected = law::EvaluateForce(reference,accepted,wrong_phase,material,results[step]) !=
                             s::Status::Success;
      state.output_preserved &= SameBytes(results[step],snapshot);
      status = law::EvaluateForce(reference,accepted,intervals[step],material,results[step]);
      if (status != s::Status::Success) {
        state.status = static_cast<int>(status);
        *output = state;
        return;
      }
    }
    accepted = results[step].proposed_history;
    state.completed = step+1;
  }
  state.status = 0;
  *output = state;
}

void RunDevice(bool faults, bool collapsed) {
  int count = 0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
  ASSERT_GT(count,0);
  const auto reference = Reference(collapsed);
  const auto material = Material();
  constexpr unsigned steps = 96;
  std::vector<s::PrescribedInterval> intervals;
  double time = 0;
  for (unsigned step = 0; step < steps; ++step) {
    auto interval = Path(reference,step,true);
    interval.base_time_s = time;
    time += interval.dt_s;
    intervals.push_back(interval);
  }
  const auto curve = material.curve;
  DeviceArray<double> strain(curve.count), yield(curve.count);
  DeviceArray<s::PrescribedInterval> motion(steps);
  DeviceArray<law::ForceTrial> trial(steps);
  DeviceArray<DeviceStatus> result(1);
  DeviceArray<DeviceScratch> scratch(1);
  const DeviceScratch initial_scratch{};
  ASSERT_EQ(cudaMemcpy(scratch.pointer,&initial_scratch,sizeof(initial_scratch),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(strain.pointer,curve.plastic_strain,sizeof(double)*curve.count,cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(yield.pointer,curve.yield_stress_pa,sizeof(double)*curve.count,cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(motion.pointer,intervals.data(),sizeof(intervals[0])*steps,cudaMemcpyHostToDevice),cudaSuccess);
  Trajectory<<<1,1>>>(reference.input(),strain.pointer,yield.pointer,curve.count,material.material,motion.pointer,
                       steps,faults,trial.pointer,result.pointer,scratch.pointer);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  DeviceStatus status;
  ASSERT_EQ(cudaMemcpy(&status,result.pointer,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(status.status,0);
  ASSERT_EQ(status.completed,steps);
  if (faults) {
    EXPECT_TRUE(status.late_rejected);
    EXPECT_TRUE(status.phase_rejected);
    EXPECT_TRUE(status.output_preserved);
  }
  std::vector<law::ForceTrial> actual(steps);
  ASSERT_EQ(cudaMemcpy(actual.data(),trial.pointer,sizeof(actual[0])*steps,cudaMemcpyDeviceToHost),cudaSuccess);
  auto native = NativeInitial(reference.input());
  for (unsigned step = 0; step < steps; ++step) {
    SCOPED_TRACE(step);
    const auto expected = Native(material,native,intervals[step]);
    ASSERT_EQ(expected.status,0);
    ASSERT_TRUE(Agree(actual[step],expected));
    ASSERT_EQ(actual[step].proposed_history.stamp().sample_index,step+1);
    ASSERT_EQ(actual[step].proposed_history.stamp().time_s,
              intervals[step].base_time_s+intervals[step].dt_s);
    native = expected.next;
  }
}
}
TEST(Rear18ForceCuda, DeviceEightPointRecurrenceMatchesIndependentNative) { RunDevice(false,false); RunDevice(false,true); }
TEST(Rear18ForceCuda, LatePointAndPhaseFailurePreserveOutputThenRetryNativeHistory) { RunDevice(true,false); RunDevice(true,true); }
#ifdef REAR18_ORIGINAL
TEST(Rear18ForceCuda, AllOriginal306Including109RepeatedSlotCells) {
  int devices = 0;
  ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);
  ASSERT_GT(devices,0);
  const auto base_material = Material();
  const auto curve = base_material.curve;
  DeviceArray<double> strain(curve.count),yield(curve.count);
  DeviceArray<s::PrescribedInterval> motion(1);
  DeviceArray<law::ForceTrial> result(1);
  DeviceArray<DeviceStatus> status_device(1);
  DeviceArray<DeviceScratch> scratch(1);
  ASSERT_EQ(cudaMemcpy(strain.pointer,curve.plastic_strain,curve.count*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(yield.pointer,curve.yield_stress_pa,curve.count*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  unsigned collapsed = 0;
  for (unsigned i = 0; i < std::size(rear18_test::original::Cells); ++i) {
    const auto input = rear18_test::original::Input(i);
    SCOPED_TRACE(input.source_element_id);
    law::Reference reference;
    ASSERT_EQ(law::InitializeReference(input,reference),s::Status::Success);
    const auto material = law44_solid_test::Parameters(input.source_part_id == 2000392);
    const auto interval = Path(reference,0);
    const DeviceScratch blank{};
    ASSERT_EQ(cudaMemcpy(scratch.pointer,&blank,sizeof(blank),cudaMemcpyHostToDevice),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(motion.pointer,&interval,sizeof(interval),cudaMemcpyHostToDevice),cudaSuccess);
    Trajectory<<<1,1>>>(input,strain.pointer,yield.pointer,curve.count,material.material,
        motion.pointer,1,false,result.pointer,status_device.pointer,scratch.pointer);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    DeviceStatus status;
    ASSERT_EQ(cudaMemcpy(&status,status_device.pointer,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(status.status,0);
    ASSERT_EQ(status.completed,1u);
    law::ForceTrial actual;
    ASSERT_EQ(cudaMemcpy(&actual,result.pointer,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    const auto expected = Native(material,NativeInitial(input),interval);
    ASSERT_EQ(expected.status,0);
    ASSERT_TRUE(Agree(actual,expected));
    collapsed += actual.diagnostics.caller_degeneracy == 12;
  }
  EXPECT_EQ(collapsed,109u);
}
#endif
} // namespace rear_force_test
