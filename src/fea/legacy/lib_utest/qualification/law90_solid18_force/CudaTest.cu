// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeSupport.h"
#include "lib_utest/qualification/law90_solid18_reference/SourceFixture.h"
#include <cuda_runtime.h>
#include <memory>
using namespace law90_force_test;
namespace {
// All sizable per-element storage is explicit device allocation. No global CUDA
// stack-limit request; production scratch and its retained state are visible.
struct DevicePacket {
  law::PreparationInput material_input;
  double curve_x[28]{},curve_y[28]{};unsigned curve_count=0;
  law::PreparedMaterial material;
  s::ReferenceInput input;
  f::Reference reference;
  f::ReferenceScratch reference_scratch;
  s::PrescribedInterval interval;
  s::Vec3 initial_velocity;
  f::ForceScratch scratch;
  f::ForceTrial published;
  f::History bad;
  f::HistoryValues prescribed;
  s::Status status=s::Status::InvalidInput;
  unsigned stage=0;
};
__global__ void Initialize(DevicePacket* p) {
  const auto result=law::PrepareSI(p->material_input,{p->curve_x,p->curve_y,p->curve_count},p->material);
  if(result!=law::Status::Ok){p->stage=1;return;}
  p->status=f::InitializeReference90Scratch(p->input,p->reference_scratch);
  if(p->status!=s::Status::Success){p->stage=2;return;}
  p->reference=p->reference_scratch.staged;
  p->status=f::InitializeForce90Scratch(p->reference,p->material,p->initial_velocity,p->scratch);
  if(p->status==s::Status::Success)p->published=p->scratch.staged;
  p->stage=3;
}
__global__ void Advance(DevicePacket* p,bool fail) {
  const f::History* accepted=&p->published.proposed_history;
  if(fail) {
    p->prescribed=accepted->data();p->prescribed.point[7].internal_energy_density_j_m3=DBL_MAX;
    p->status=f::force_detail::HistoryWriter::Prepare(p->reference,p->material,p->prescribed,accepted->stamp(),p->bad);
    if(p->status!=s::Status::Success){p->stage=4;return;}
    accepted=&p->bad;
  }
  p->status=f::EvaluateForce90Scratch(p->reference,*accepted,p->interval,p->material,p->scratch);
  if(p->status==s::Status::Success)p->published=p->scratch.staged;
  p->stage=5;
}
struct Device {
  DevicePacket* pointer=nullptr;
  Device(){if(cudaMalloc(&pointer,sizeof(DevicePacket))!=cudaSuccess)pointer=nullptr;}
  ~Device(){if(pointer)cudaFree(pointer);}
};
void SetMaterial(DevicePacket& p,const law::PreparationInput& input,law::CurveView curve) {
  p.material_input=input;p.curve_count=curve.count;
  std::copy_n(curve.compression_strain,curve.count,p.curve_x);std::copy_n(curve.stress_pa,curve.count,p.curve_y);
}
void PushInterval(DevicePacket* device,const s::PrescribedInterval& interval) {
  ASSERT_EQ(cudaMemcpy(&device->interval,&interval,sizeof(interval),cudaMemcpyHostToDevice),cudaSuccess);
}
void Read(DevicePacket* device,DevicePacket& host) {
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&host,device,sizeof(host),cudaMemcpyDeviceToHost),cudaSuccess);
}
}
void RecurrentDevice(bool blank_hu) {
  Device device;ASSERT_NE(device.pointer,nullptr);auto host=std::make_unique<DevicePacket>();
  law90_point_test::ToyCurve toy;
  const auto material_input=blank_hu ? law90_test::OriginalBlankHuInput() : ElementToyInput();
  const auto curve=blank_hu ? law90_test::OriginalBlankHuCurve() : toy.view();
  SetMaterial(*host,material_input,curve);host->input=Distorted();host->input.density_kg_m3=material_input.density_kg_m3;
  for(auto& p:host->input.position_m){p.x*=10;p.y*=10;p.z*=10;}
  host->initial_velocity={3,-1,2};
  const auto prepared=law90_point_test::NativePrepared(material_input,curve);
  NativeForce native(host->input.density_kg_m3);s::PrescribedInterval virgin;
  for(unsigned n=0;n<8;++n){virgin.position_endpoint_m[n]=host->input.position_m[n];virgin.velocity_midpoint_m_s[n]=host->initial_velocity;}
  AdvanceNative(prepared.data(),curve,host->input,virgin,true,native);ASSERT_EQ(native.status,0);
  ASSERT_EQ(cudaMemcpy(device.pointer,host.get(),sizeof(*host),cudaMemcpyHostToDevice),cudaSuccess);
  Initialize<<<1,1>>>(device.pointer);Read(device.pointer,*host);ASSERT_EQ(host->stage,3u);ASSERT_EQ(host->status,s::Status::Success);
  ASSERT_TRUE(ForceAgreement(ForceValues(host->published),native));CheckCursors(host->published,native);
  for(unsigned step=1;step<=160;++step) {
    SCOPED_TRACE(step);auto interval=Path(host->input,step);MatchBase(host->published.proposed_history,interval);
    PushInterval(device.pointer,interval);
    if(step==40) {
      const auto before=Bytes(host->published);Advance<<<1,1>>>(device.pointer,true);Read(device.pointer,*host);
      ASSERT_EQ(host->stage,5u);ASSERT_NE(host->status,s::Status::Success);ASSERT_EQ(Bytes(host->published),before);
      if(blank_hu)EXPECT_GT(host->scratch.next.point[6].point.instantaneous_quasistatic_energy_pa,0);
      else EXPECT_GT(host->scratch.next.point[6].point.strain_norm,0);
    }
    AdvanceNative(prepared.data(),curve,host->input,interval,false,native);ASSERT_EQ(native.status,0);
    Advance<<<1,1>>>(device.pointer,false);Read(device.pointer,*host);ASSERT_EQ(host->status,s::Status::Success);
    ASSERT_TRUE(ForceAgreement(ForceValues(host->published),native));CheckCursors(host->published,native);
    ASSERT_EQ(host->published.proposed_history.stamp().sample_index,step);
  }
  ::testing::Test::RecordProperty("explicit_device_packet_bytes",int(sizeof(DevicePacket)));
}
TEST(Law90Solid18Cuda, IndependentRecurrentNativeValuesLateFailureAndRetry) { RecurrentDevice(false); }
TEST(Law90Solid18Cuda, ActualBlankHuRawCurveNativeValuesLateFailureAndRetry) { RecurrentDevice(true); }
void AllOriginalDevice(bool blank_hu) {
  Device device;ASSERT_NE(device.pointer,nullptr);auto host=std::make_unique<DevicePacket>();
  const auto input_material=blank_hu ? law90_test::OriginalBlankHuInput() : law90_test::OriginalInput();
  const auto curve=blank_hu ? law90_test::OriginalBlankHuCurve() : law90_test::OriginalCurve();
  const auto prepared=law90_point_test::NativePrepared(input_material,curve);
  unsigned observed=0;
  for(unsigned row=0;row<law90_reference_test::fixture::element_count;++row) {
    SetMaterial(*host,input_material,curve);host->input=law90_reference_test::Original(row);if(blank_hu)host->input.density_kg_m3=input_material.density_kg_m3;host->initial_velocity={15.6464,0,0};
    SCOPED_TRACE(host->input.source_element_id);NativeForce native(host->input.density_kg_m3);s::PrescribedInterval virgin;
    for(unsigned n=0;n<8;++n){virgin.position_endpoint_m[n]=host->input.position_m[n];virgin.velocity_midpoint_m_s[n]=host->initial_velocity;}
    AdvanceNative(prepared.data(),curve,host->input,virgin,true,native);ASSERT_EQ(native.status,0);
    ASSERT_EQ(cudaMemcpy(device.pointer,host.get(),sizeof(*host),cudaMemcpyHostToDevice),cudaSuccess);
    Initialize<<<1,1>>>(device.pointer);Read(device.pointer,*host);ASSERT_EQ(host->status,s::Status::Success);
    ASSERT_TRUE(ForceAgreement(ForceValues(host->published),native));CheckCursors(host->published,native);++observed;
    for(unsigned step=1;step<=2;++step) {
      auto interval=Path(host->input,step,1e-6,.03,true);MatchBase(host->published.proposed_history,interval);
      AdvanceNative(prepared.data(),curve,host->input,interval,false,native);ASSERT_EQ(native.status,0);
      PushInterval(device.pointer,interval);Advance<<<1,1>>>(device.pointer,false);Read(device.pointer,*host);
      ASSERT_EQ(host->status,s::Status::Success);ASSERT_TRUE(ForceAgreement(ForceValues(host->published),native));
      CheckCursors(host->published,native);++observed;
    }
  }
  ::testing::Test::RecordProperty("original_device_native_packets",observed);EXPECT_EQ(observed,1345u*3u);
}

TEST(Law90Solid18Cuda, All1345OriginalConstructorAndFirstNativeIntervals) { AllOriginalDevice(false); }
TEST(Law90Solid18Cuda, ActualBlankHuRawCurveAll1345ConstructorAndCarriedIntervals) { AllOriginalDevice(true); }
