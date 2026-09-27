// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeExpected.h"
#include "Diagnostics.h"
#include <cuda_runtime.h>
using namespace law90_control_test;
namespace {
struct DeviceCase {
  c::Reference reference;
  c::Scratch scratch;
  c::Result accepted;
  s::PrescribedInterval intervals[32];
  c::Result output[33];
  PointOperands operands[33];
  d::Status status=d::Status::InvalidInput;
};
__global__ void RunControlledCases(DeviceCase* values,unsigned count) {
  const unsigned row=blockIdx.x*blockDim.x+threadIdx.x;if(row>=count)return;
  auto& c=values[row];c.status=c::PrepareInitial(c.reference,{0,0,0},c.scratch);
  if(c.status!=d::Status::Success)return;
  c.status=c::Complete(c.scratch,c.scratch.activity.triggers_native_batch!=0,c.accepted);
  if(c.status!=d::Status::Success)return;c.output[0]=c.accepted;Capture(c.scratch,nullptr,c.operands[0]);
  for(unsigned step=0;step<32;++step) {
    c.status=c::PrepareCandidate(c.reference,c.accepted.proposed_history,c.intervals[step],c.scratch);
    if(c.status!=d::Status::Success)return;
    Capture(c.scratch,&c.accepted.proposed_history,c.operands[step+1]);
    c.status=c::Complete(c.scratch,c.scratch.activity.triggers_native_batch!=0,c.output[step+1]);
    if(c.status!=d::Status::Success)return;c.accepted=c.output[step+1];
  }
}
}
TEST(Law90ControlledDistortionCuda, NativeCarriedHistoryBothUnitsAndOrientations) {
  int count=0;ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0);
  const auto curve=law90_test::OriginalBlankHuCurve();double *x=nullptr,*y=nullptr;
  ASSERT_EQ(cudaMalloc(&x,curve.count*sizeof(double)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&y,curve.count*sizeof(double)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(x,curve.compression_strain,curve.count*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(y,curve.stress_pa,curve.count*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  std::vector<DeviceCase> cases(4);std::vector<c::Result> expected(4*33);std::vector<std::array<double,357>> native_packets(4*33);std::vector<std::array<double,40>> native_modulus(4*33);unsigned row=0;
  for(const auto units:{d::UnitScale{1,1,1},d::UnitScale{.001,1000,1}})for(bool reflected:{false,true}) {
    auto input=law90_force_test::Distorted();const auto material=law90_force_test::Material(true);
    input.density_kg_m3=material.reader().density_kg_m3;if(reflected)for(auto& v:input.position_m)v.x=-v.x;
    const auto source=Reference(input,material,units);
    ASSERT_EQ(c::RelocateReference(source,{x,y,curve.count},cases[row].reference),d::Status::Success);
    const auto native_material=law90_point_test::NativePrepared(WorkingInput(law90_test::OriginalBlankHuInput(),units),curve);
    NativeState native(native_material[1]);s::PrescribedInterval initial;
    for(unsigned n=0;n<8;++n)initial.position_endpoint_m[n]=input.position_m[n];
    expected[row*33]=Native(native_material.data(),curve,input,initial,units,true,native);native_packets[row*33]=native.force.values;native_modulus[row*33]=native.modulus;
    double base=0;
    for(unsigned step=1;step<=32;++step) {
      auto interval=Path(input,step);interval.base_time_s=base;base+=interval.dt_s;
      cases[row].intervals[step-1]=interval;
      expected[row*33+step]=Native(native_material.data(),curve,input,interval,units,false,native);native_packets[row*33+step]=native.force.values;native_modulus[row*33+step]=native.modulus;
    }
    ++row;
  }
  DeviceCase* device=nullptr;ASSERT_EQ(cudaMalloc(&device,cases.size()*sizeof(DeviceCase)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device,cases.data(),cases.size()*sizeof(DeviceCase),cudaMemcpyHostToDevice),cudaSuccess);
  RunControlledCases<<<1,32>>>(device,4);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(cases.data(),device,cases.size()*sizeof(DeviceCase),cudaMemcpyDeviceToHost),cudaSuccess);
  WriteDiagnostics(cases,native_packets,native_modulus);
  for(unsigned i=0;i<4;++i) {
    ASSERT_EQ(cases[i].status,d::Status::Success)<<i;
    for(unsigned step=0;step<=32;++step){SCOPED_TRACE(i*33+step);Compare(cases[i].output[step],expected[i*33+step]);}
  }
  EXPECT_EQ(cudaFree(device),cudaSuccess);EXPECT_EQ(cudaFree(x),cudaSuccess);EXPECT_EQ(cudaFree(y),cudaSuccess);
}
