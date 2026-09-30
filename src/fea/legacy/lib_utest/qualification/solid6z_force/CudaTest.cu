// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "Compare.h"
#include <cuda_runtime.h>
#include <vector>
#include <limits>

namespace solid6z_force_test {
namespace {
constexpr unsigned Steps = 400;
struct DeviceScratch {
  s::History accepted,bad;
  s::HistoryValues bad_values;
  s::ForceTrial trial;
  s::PrescribedInterval wrong;
  unsigned char before[sizeof(s::ForceTrial)];
  int status = 0;
  int rollback_checks = 0;
};
__global__ void Run(s::Reference reference,s::Material material,
    const s::PrescribedInterval* intervals,Values* values,DeviceScratch* scratch,bool retry) {
  if (threadIdx.x != 0 || blockIdx.x != 0) return;
  if (s::InitializeHistory(reference,material,{},scratch->accepted) != s::Status::Success) {
    scratch->status = 1;
    return;
  }
  for (unsigned step = 0; step < Steps; ++step) {
    if (s::EvaluateForce(reference,scratch->accepted,intervals[step],material,{},scratch->trial) != s::Status::Success) {
      scratch->status = 1000+step;
      return;
    }
    if (retry && step == 177) {
      for (unsigned i = 0; i < sizeof(s::ForceTrial); ++i)
        scratch->before[i] = reinterpret_cast<const unsigned char*>(&scratch->trial)[i];
      scratch->bad_values = scratch->accepted.data();
      scratch->bad_values.material.internal_energy_density_j_m3 = 1.7976931348623157e308;
      scratch->bad_values.hourglass_stress_pa[2][3] = ::copysign(1.7976931348623157e308,
          scratch->trial.stabilization.modal_velocity_m_s[2][3]);
      if (s::PreparePrescribedHistory(reference,material,{},scratch->bad_values,
          scratch->accepted.stamp(),scratch->bad) != s::Status::Success) {
        scratch->status = 2;
        return;
      }
      if (s::EvaluateForce(reference,scratch->bad,intervals[step],material,{},scratch->trial) == s::Status::Success) {
        scratch->status = 3;
        return;
      }
      for (unsigned i = 0; i < sizeof(s::ForceTrial); ++i) {
        if (scratch->before[i] != reinterpret_cast<const unsigned char*>(&scratch->trial)[i]) {
          scratch->status = 4;
          return;
        }
      }
      ++scratch->rollback_checks;
      scratch->wrong = intervals[step];
      ++scratch->wrong.sample_index;
      if (s::EvaluateForce(reference,scratch->accepted,scratch->wrong,material,{},scratch->trial) == s::Status::Success) {
        scratch->status = 5;
        return;
      }
      for (unsigned i = 0; i < sizeof(s::ForceTrial); ++i) {
        if (scratch->before[i] != reinterpret_cast<const unsigned char*>(&scratch->trial)[i]) {
          scratch->status = 6;
          return;
        }
      }
      ++scratch->rollback_checks;
      if (s::EvaluateForce(reference,scratch->accepted,intervals[step],material,{},scratch->trial) != s::Status::Success) {
        scratch->status = 7;
        return;
      }
    }
    values[step] = Pack(scratch->trial);
    scratch->accepted = scratch->trial.proposed_history;
  }
}
template<class T> struct Device {
  T* data = nullptr;
  ~Device() { if (data) cudaFree(data); }
};
void Check(bool retry) {
  const auto reference = Reference();
  const auto material = Material();
  NativeHistory native;
  ASSERT_TRUE(native.Initialize(reference.input(),material));
  std::vector<s::PrescribedInterval> intervals;
  std::vector<NativeResult> expected;
  double time = 0;
  for (unsigned step = 0; step < Steps; ++step) {
    auto interval = Path(reference,step);
    interval.base_time_s = time;
    time += interval.dt_s;
    intervals.push_back(interval);
    expected.push_back(native.Evaluate(interval));
    ASSERT_EQ(expected.back().status,0) << step;
    native.Accept(expected.back());
  }
  Device<s::PrescribedInterval> device_intervals;
  Device<Values> device_values;
  Device<DeviceScratch> device_scratch;
  ASSERT_EQ(cudaMalloc(&device_intervals.data,Steps*sizeof(s::PrescribedInterval)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&device_values.data,Steps*sizeof(Values)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&device_scratch.data,sizeof(DeviceScratch)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device_intervals.data,intervals.data(),Steps*sizeof(s::PrescribedInterval),cudaMemcpyHostToDevice),cudaSuccess);
  const DeviceScratch initial{};
  ASSERT_EQ(cudaMemcpy(device_scratch.data,&initial,sizeof(initial),cudaMemcpyHostToDevice),cudaSuccess);
  Run<<<1,1>>>(reference,material,device_intervals.data,device_values.data,device_scratch.data,retry);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  DeviceScratch scratch;
  ASSERT_EQ(cudaMemcpy(&scratch,device_scratch.data,sizeof(scratch),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(scratch.status,0);
  EXPECT_EQ(scratch.rollback_checks,retry ? 2 : 0);
  EXPECT_EQ(scratch.accepted.stamp().sample_index,Steps);
  std::vector<Values> result(Steps);
  ASSERT_EQ(cudaMemcpy(result.data(),device_values.data,Steps*sizeof(Values),cudaMemcpyDeviceToHost),cudaSuccess);
  for (unsigned step = 0; step < Steps; ++step) ASSERT_TRUE(Agree(result[step],expected[step])) << step;
}
}
TEST(Solid6zForceCuda, FourHundredStepsMatchIndependentNativeCaller) { Check(false); }
TEST(Solid6zForceCuda, LateHistoryAndClockFaultsPreserveTrialThenRetryOwnTrajectory) { Check(true); }
} // namespace solid6z_force_test
