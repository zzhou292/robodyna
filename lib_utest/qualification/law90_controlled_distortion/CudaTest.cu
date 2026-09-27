// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeHistory.h"
#include "CarriedAgreement.h"
#include <cuda_runtime.h>
using namespace law90_control_test;
namespace {
template<unsigned Steps> struct DeviceCase {
  static constexpr unsigned steps=Steps;
  c::Reference reference;
  c::Scratch scratch;
  c::Result accepted,output[Steps+1];
  s::PrescribedInterval intervals[Steps];
  PointOperands operands[Steps+1];
  f::HistoryValues native_history[Steps+1];
  double native_distortion_energy[Steps+1]{};
  d::Status status=d::Status::InvalidInput;
};
template<unsigned Steps> __global__ void RunControlledCases(DeviceCase<Steps>* values,unsigned count,bool native_incoming) {
  const unsigned row=blockIdx.x*blockDim.x+threadIdx.x;if(row>=count)return;
  auto& item=values[row];item.status=c::PrepareInitial(item.reference,{0,0,0},item.scratch);
  if(item.status!=d::Status::Success)return;
  item.status=c::Complete(item.scratch,item.scratch.activity.triggers_native_batch!=0,item.accepted);
  if(item.status!=d::Status::Success)return;item.output[0]=item.accepted;
  Capture(item.scratch,nullptr,item.operands[0]);
  for(unsigned step=0;step<Steps;++step) {
    if(native_incoming) {
      // Test-only conditional trial: seed ONLY previous accepted native history.
      // All current geometry/material forces, sound and control are GPU computed.
      f::History previous;
      const auto& r=item.reference;
      const auto status=f::force_detail::HistoryWriter::Prepare(r.native_reference(),r.material().native_material(),
          item.native_history[step],{item.intervals[step].base_time_s,step},previous);
      if(status!=s::Status::Success){item.status=d::Status::InvalidInput;return;}
      c::HistoryWriter::Set(previous,item.native_distortion_energy[step],r.material().units(),item.accepted.proposed_history);
    }
    item.status=c::PrepareCandidate(item.reference,item.accepted.proposed_history,item.intervals[step],item.scratch);
    if(item.status!=d::Status::Success)return;
    Capture(item.scratch,&item.accepted.proposed_history,item.operands[step+1]);
    item.status=c::Complete(item.scratch,item.scratch.activity.triggers_native_batch!=0,item.output[step+1]);
    if(item.status!=d::Status::Success)return;item.accepted=item.output[step+1];
  }
}
template<unsigned Steps> void CheckCases(bool native_incoming) {
  int count=0;ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0);
  const auto curve=law90_test::OriginalBlankHuCurve();double *x=nullptr,*y=nullptr;
  ASSERT_EQ(cudaMalloc(&x,curve.count*sizeof(double)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&y,curve.count*sizeof(double)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(x,curve.compression_strain,curve.count*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(y,curve.stress_pa,curve.count*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  using Case=DeviceCase<Steps>;constexpr unsigned frames=Steps+1;
  std::vector<Case> cases(4);std::vector<c::Result> expected(4*frames);
  std::vector<std::array<double,357>> native_packets(4*frames);
  std::vector<std::array<double,40>> native_modulus(4*frames);unsigned row=0;
  for(const auto units:{d::UnitScale{1,1,1},d::UnitScale{.001,1000,1}})for(bool reflected:{false,true}) {
    auto input=law90_force_test::Distorted();const auto material=law90_force_test::Material(true);
    input.density_kg_m3=material.reader().density_kg_m3;if(reflected)for(auto& v:input.position_m)v.x=-v.x;
    const auto source=Reference(input,material,units);
    ASSERT_EQ(c::RelocateReference(source,{x,y,curve.count},cases[row].reference),d::Status::Success);
    const auto native_material=law90_point_test::NativePrepared(WorkingInput(law90_test::OriginalBlankHuInput(),units),curve);
    NativeState native(native_material[1]);s::PrescribedInterval initial;
    for(unsigned n=0;n<8;++n)initial.position_endpoint_m[n]=input.position_m[n];
    expected[row*frames]=Native(native_material.data(),curve,input,initial,units,true,native);
    native_packets[row*frames]=native.force.values;native_modulus[row*frames]=native.modulus;
    cases[row].native_history[0]=NativeHistory(native);cases[row].native_distortion_energy[0]=native.distortion_energy;
    double base=0;
    for(unsigned step=1;step<=Steps;++step) {
      auto interval=Path(input,step);interval.base_time_s=base;base+=interval.dt_s;
      cases[row].intervals[step-1]=interval;
      expected[row*frames+step]=Native(native_material.data(),curve,input,interval,units,false,native);
      native_packets[row*frames+step]=native.force.values;native_modulus[row*frames+step]=native.modulus;
      cases[row].native_history[step]=NativeHistory(native);cases[row].native_distortion_energy[step]=native.distortion_energy;
    }
    ++row;
  }
  Case* device=nullptr;ASSERT_EQ(cudaMalloc(&device,cases.size()*sizeof(Case)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device,cases.data(),cases.size()*sizeof(Case),cudaMemcpyHostToDevice),cudaSuccess);
  RunControlledCases<<<1,32>>>(device,4,native_incoming);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(cases.data(),device,cases.size()*sizeof(Case),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(cudaFree(device),cudaSuccess);
  Drift drift;
  for(unsigned i=0;i<4;++i) {
    ASSERT_EQ(cases[i].status,d::Status::Success)<<i;
    if(native_incoming)for(unsigned step=0;step<=Steps;++step) {
      SCOPED_TRACE(i*frames+step);Compare(cases[i].output[step],expected[i*frames+step]);
      CheckNativeHistory(cases[i].output[step],cases[i].native_history[step]);
    } else CheckCarried(cases[i],native_packets,native_modulus,i,drift);
  }
  if(!native_incoming) {
    WriteDiagnostics(cases,native_packets,native_modulus);
    ::testing::Test::RecordProperty("conditioning_limited_modulus_relative_drift",drift.modulus_relative);
    ::testing::Test::RecordProperty("conditioning_limited_point_stiffness_relative_drift",drift.stiffness_relative);
  }
  EXPECT_EQ(cudaFree(x),cudaSuccess);EXPECT_EQ(cudaFree(y),cudaSuccess);
}
} // namespace
TEST(Law90ControlledDistortionCuda, IndependentNativeHistoryEachInterval) {CheckCases<64>(true);}
TEST(Law90ControlledDistortionCuda, CarriedHistorySourceEquationsAndConditioning) {CheckCases<32>(false);}
