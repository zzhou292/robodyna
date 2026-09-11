// SPDX-License-Identifier: AGPL-3.0-or-later
#include "PacketValues.h"
#include <cuda_runtime.h>
#include <stdexcept>
#include <vector>

namespace solid18_force_test {
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
__device__ bool SameBytes(const s::ForceTrial& a, const unsigned char* y) {
  const auto* x = reinterpret_cast<const unsigned char*>(&a);
  for (unsigned i = 0; i < sizeof(a); ++i) {
    if (x[i] != y[i]) return false;
  }
  return true;
}
__global__ void Trajectory(s::ReferenceInput input, const double* strain, const double* yield,
    unsigned points, const s::PrescribedInterval* intervals, unsigned count, bool faults,
    s::ForceTrial* results, DeviceStatus* output) {
  s::Reference reference;
  s::Material material;
  s::History accepted;
  DeviceStatus state;
  if (s::InitializeReference(input,reference) != s::Status::Success ||
      tl::material::law36::Prepare(law36_test::E,law36_test::Nu,law36_test::Rho,
          {strain,yield,points},material) != tl::material::law36::Status::Ok ||
      s::InitializeHistory(reference,material,accepted) != s::Status::Success) {
    *output = state;
    return;
  }
  for (unsigned step = 0; step < count; ++step) {
    auto status = s::EvaluateForce(reference,accepted,intervals[step],material,results[step]);
    if (status != s::Status::Success) {
      state.status = static_cast<int>(status);
      *output = state;
      return;
    }
    if (faults && step == 100) {
      unsigned char snapshot[sizeof(s::ForceTrial)];
      const auto* original = reinterpret_cast<const unsigned char*>(&results[step]);
      for (unsigned i = 0; i < sizeof(snapshot); ++i) snapshot[i] = original[i];
      auto bad_values = accepted.data();
      bad_values.point[7].material.point.stress_pa[0] = 1e308;
      s::History bad;
      if (s::PreparePrescribedHistory(reference,material,bad_values,accepted.stamp(),bad) !=
          s::Status::Success) {
        *output = state;
        return;
      }
      state.late_rejected = s::EvaluateForce(reference,bad,intervals[step],material,results[step]) !=
                            s::Status::Success;
      state.output_preserved = SameBytes(results[step],snapshot);
      auto wrong_phase = intervals[step];
      ++wrong_phase.sample_index;
      state.phase_rejected = s::EvaluateForce(reference,accepted,wrong_phase,material,results[step]) !=
                             s::Status::Success;
      state.output_preserved &= SameBytes(results[step],snapshot);
      status = s::EvaluateForce(reference,accepted,intervals[step],material,results[step]);
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

void RunDevice(bool faults) {
  int count = 0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
  ASSERT_GT(count,0);
  ASSERT_EQ(cudaDeviceSetLimit(cudaLimitStackSize,256*1024),cudaSuccess);
  const auto reference = Reference();
  const auto material = Material();
  constexpr unsigned steps = 400;
  std::vector<s::PrescribedInterval> intervals;
  for (unsigned step = 0; step < steps; ++step) intervals.push_back(Path(reference,step));
  const auto curve = material.curve;
  DeviceArray<double> strain(curve.count), yield(curve.count);
  DeviceArray<s::PrescribedInterval> motion(steps);
  DeviceArray<s::ForceTrial> trial(steps);
  DeviceArray<DeviceStatus> result(1);
  ASSERT_EQ(cudaMemcpy(strain.pointer,curve.plastic_strain,sizeof(double)*curve.count,cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(yield.pointer,curve.yield_stress_pa,sizeof(double)*curve.count,cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(motion.pointer,intervals.data(),sizeof(intervals[0])*steps,cudaMemcpyHostToDevice),cudaSuccess);
  Trajectory<<<1,1>>>(reference.input(),strain.pointer,yield.pointer,curve.count,motion.pointer,
                       steps,faults,trial.pointer,result.pointer);
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
  std::vector<s::ForceTrial> actual(steps);
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
TEST(Solid18ForceCuda, DeviceEightPointRecurrenceMatchesIndependentNative) { RunDevice(false); }
TEST(Solid18ForceCuda, LatePointAndPhaseFailurePreserveOutputThenRetryNativeHistory) { RunDevice(true); }
} // namespace solid18_force_test
