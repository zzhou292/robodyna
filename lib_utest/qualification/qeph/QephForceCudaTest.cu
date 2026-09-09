#include "QephForceFixture.h"
#include <cuda_runtime.h>

namespace {
using namespace qeph_force_port_test;
constexpr std::uint64_t kCanary=0x51455048464f5243ULL;
struct Packet {
  std::uint64_t before=kCanary;
  port::ReferenceData reference;
  port::History base;
  port::PrescribedInterval interval;
  port::ForceTrial output;
  port::Status status=port::Status::kInvalidInput;
  std::uint64_t after=kCanary;
};
static_assert(std::is_trivially_copyable_v<Packet>);
static_assert(sizeof(Packet)<8192);
__global__ void Evaluate(Packet* p) { p->status=port::EvaluateForce(p->reference,p->base,p->interval,p->output); }
class DevicePacket {
 public:
  cudaError_t Allocate() { return cudaMalloc(reinterpret_cast<void**>(&p_),sizeof(Packet)); }
  cudaError_t Run(Packet& h) {
    auto e=cudaMemcpy(p_,&h,sizeof(Packet),cudaMemcpyHostToDevice); if(e!=cudaSuccess) return e;
    Evaluate<<<1,1>>>(p_); e=cudaGetLastError(); if(e!=cudaSuccess) return e;
    e=cudaDeviceSynchronize(); if(e!=cudaSuccess) return e;
    return cudaMemcpy(&h,p_,sizeof(Packet),cudaMemcpyDeviceToHost);
  }
  ~DevicePacket() { if(p_) EXPECT_EQ(cudaFree(p_),cudaSuccess); }
  DevicePacket()=default;
  DevicePacket(const DevicePacket&)=delete;
  DevicePacket& operator=(const DevicePacket&)=delete;
 private:
  Packet* p_=nullptr;
};
void Guards(const Packet& p) { EXPECT_EQ(p.before,kCanary); EXPECT_EQ(p.after,kCanary); }

TEST(QephForceCuda, CompleteNativeAndHostParityAcrossAllGeometryRateHistoryConfigurations) {
  DevicePacket device;
  ASSERT_EQ(device.Allocate(),cudaSuccess); // Missing CUDA is failure, never skip.
  for(unsigned fixture=0;fixture<kCases;++fixture) for(unsigned shift=0;shift<4;++shift)
    for(bool transformed:{false,true}) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(shift);
      SCOPED_TRACE(transformed);
      const auto raw=Case(fixture),input=qeph_startup_test::Reparameterize(raw,shift,transformed);
      Packet p; native::Reference reference;
      ASSERT_EQ(port::InitializeReference(input,p.reference),port::Status::kSuccess);
      ASSERT_EQ(native::Initialize(NativeInput(input),reference),native::Status::kSuccess);
      for(unsigned pattern=0;pattern<3;++pattern) for(bool seeded:{false,true}) {
        SCOPED_TRACE(pattern);
        SCOPED_TRACE(seeded);
        p.interval=qeph_kinematics_test::Reparameterize(Pattern(raw,pattern),shift,transformed);
        const auto values=Seed(input,seeded); native::History history;
        ASSERT_EQ(port::PreparePrescribedHistory(p.reference,values,
            {p.interval.base_time,p.interval.sample_index-1},p.base),port::Status::kSuccess);
        ASSERT_EQ(native::PreparePrescribedHistory(reference,NativeValues(values),
            {p.interval.base_time,p.interval.sample_index-1},history),native::Status::kSuccess);
        port::ForceTrial host; native::ForceTrial truth;
        ASSERT_EQ(port::EvaluateForce(p.reference,p.base,p.interval,host),port::Status::kSuccess);
        ASSERT_EQ(native::EvaluateForce(reference,history,NativeInterval(p.interval),truth),native::Status::kSuccess);
        const auto saved_ref=Bytes(p.reference);
        const auto saved_base=Bytes(p.base);
        const auto saved_interval=Bytes(p.interval);
        ASSERT_EQ(device.Run(p),cudaSuccess); Guards(p);
        ASSERT_EQ(p.status,port::Status::kSuccess);
        ForceAgreement(p.output,truth,input,p.interval); ForceAgreement(p.output,host,input,p.interval);
        EXPECT_EQ(Bytes(p.reference),saved_ref); EXPECT_EQ(Bytes(p.base),saved_base); EXPECT_EQ(Bytes(p.interval),saved_interval);
      }
    }
  RecordProperty("native_parity_configurations",432);
  RecordProperty("owned_device_bytes",static_cast<int>(sizeof(Packet)));
  RecordProperty("kernel_threads",1); RecordProperty("dynamics_qualified","false");
}

TEST(QephForceCuda, IndependentPhysicalStressMomentsAndAcceptedByValueSequence) {
  DevicePacket device;
  ASSERT_EQ(device.Allocate(),cudaSuccess);
  for(unsigned fixture:{0u,2u,4u}) {
    const auto input=Case(fixture); Packet p;
    ASSERT_EQ(port::InitializeReference(input,p.reference),port::Status::kSuccess);
    ASSERT_EQ(port::InitializeHistory(p.reference,{},p.base),port::Status::kSuccess);
    for(unsigned mode=0;mode<8;++mode) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(mode);
      p.interval=Next(input,p.base); const double rate=mode<5?.02:.02/Scale(input);
      ApplyMode(p.interval,mode,rate);
      ASSERT_EQ(device.Run(p),cudaSuccess); Guards(p);
      ASSERT_EQ(p.status,port::Status::kSuccess);
      ModeStressTruth(p.output,input,mode,rate,p.interval.dt); Balance(p.output,p.interval);
    }
  }
  for(unsigned fixture:{0u,1u,3u}) {
    const auto input=Case(fixture); Packet p; native::Reference native_ref; native::History native_history;
    ASSERT_EQ(port::InitializeReference(input,p.reference),port::Status::kSuccess);
    ASSERT_EQ(native::Initialize(NativeInput(input),native_ref),native::Status::kSuccess);
    const auto values=Seed(input,true);
    ASSERT_EQ(port::PreparePrescribedHistory(p.reference,values,{},p.base),port::Status::kSuccess);
    ASSERT_EQ(native::PreparePrescribedHistory(native_ref,NativeValues(values),{},native_history),native::Status::kSuccess);
    for(double sign:{1.,1.,0.,-1.,-1.}) {
      p.interval=Next(input,p.base,.001);
      for(unsigned n=0;n<4;++n) {
        const double s=sign*(n%2?-1.:1.);
        p.interval.velocity_midpoint[n]={s*.003,s*.004,s*.002};
        p.interval.omega_midpoint[n]={s*.005,s*.006,0};
      }
      native::ForceTrial truth;
      ASSERT_EQ(native::EvaluateForce(native_ref,native_history,NativeInterval(p.interval),truth),native::Status::kSuccess);
      const auto saved_base=Bytes(p.base);
      ASSERT_EQ(device.Run(p),cudaSuccess); Guards(p);
      ASSERT_EQ(p.status,port::Status::kSuccess); EXPECT_EQ(Bytes(p.base),saved_base);
      ForceAgreement(p.output,truth,input,p.interval); Balance(p.output,p.interval);
      native_history=truth.proposed_history; p.base=p.output.proposed_history;
    }
  }
}

TEST(QephForceCuda, ForeignStaleAndLateThicknessFailurePreserveCompleteOutputThenRetry) {
  DevicePacket device;
  ASSERT_EQ(device.Allocate(),cudaSuccess);
  const auto input=Case(0); Packet p;
  ASSERT_EQ(port::InitializeReference(input,p.reference),port::Status::kSuccess);
  ASSERT_EQ(port::PreparePrescribedHistory(p.reference,Seed(input,true),{.125,72},p.base),port::Status::kSuccess);
  p.interval=Next(input,p.base,.001);
  ASSERT_EQ(device.Run(p),cudaSuccess);
  ASSERT_EQ(p.status,port::Status::kSuccess);
  const auto good_ref=p.reference; const auto good_base=p.base; const auto good_interval=p.interval;
  const auto output_bytes=Bytes(p.output);
  for(unsigned kind=0;kind<8;++kind) {
    SCOPED_TRACE(kind);
    p.reference=good_ref; p.base=good_base; p.interval=good_interval;
    if(kind==0) p.reference.prepared=false;
    if(kind==1) ++p.reference.input.node_ids[3];
    if(kind==2) p.interval.base_time=std::nextafter(p.interval.base_time,1.);
    if(kind==3) ++p.interval.sample_index;
    if(kind==4) p.interval.dt=std::numeric_limits<double>::denorm_min();
    if(kind==5) ApplyMode(p.interval,0,4./p.interval.dt);
    if(kind==6) p.interval.omega_midpoint[3].x=std::numeric_limits<double>::infinity();
    if(kind==7) {
      ASSERT_EQ(port::InitializeHistory(p.reference,{.125,UINT64_MAX},p.base),port::Status::kSuccess);
      p.interval.sample_index=0;
    }
    const auto ref_bytes=Bytes(p.reference);
    const auto base_bytes=Bytes(p.base);
    const auto interval_bytes=Bytes(p.interval);
    ASSERT_EQ(device.Run(p),cudaSuccess); Guards(p);
    EXPECT_NE(p.status,port::Status::kSuccess); EXPECT_EQ(Bytes(p.output),output_bytes);
    EXPECT_EQ(Bytes(p.reference),ref_bytes); EXPECT_EQ(Bytes(p.base),base_bytes); EXPECT_EQ(Bytes(p.interval),interval_bytes);
  }
  p.reference=good_ref; p.base=good_base; p.interval=good_interval; port::ForceTrial clean;
  ASSERT_EQ(port::EvaluateForce(p.reference,p.base,p.interval,clean),port::Status::kSuccess);
  ASSERT_EQ(device.Run(p),cudaSuccess); Guards(p);
  ASSERT_EQ(p.status,port::Status::kSuccess); ForceAgreement(p.output,clean,input,p.interval);
}
} // namespace
