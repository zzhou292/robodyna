#include "QephKinematicsFixture.h"
#include <cuda_runtime.h>

namespace {
using namespace qeph_kinematics_test;
constexpr std::uint64_t kCanary=0x5145504847454f4dULL;
struct Packet {
  std::uint64_t before=kCanary;
  port::ReferenceData reference;
  port::PrescribedInterval interval;
  port::Kinematics output;
  port::Status status=port::Status::kInvalidInput;
  std::uint64_t after=kCanary;
};
static_assert(std::is_trivially_copyable_v<Packet>);
static_assert(sizeof(Packet)<4096);
__global__ void Evaluate(Packet* packet) {
  packet->status=port::EvaluatePrescribed(packet->reference,packet->interval,packet->output);
}
class DevicePacket {
 public:
  cudaError_t Allocate() { return cudaMalloc(reinterpret_cast<void**>(&packet_),sizeof(Packet)); }
  cudaError_t Run(Packet& host) {
    auto error=cudaMemcpy(packet_,&host,sizeof(Packet),cudaMemcpyHostToDevice);
    if(error!=cudaSuccess) return error;
    Evaluate<<<1,1>>>(packet_);
    error=cudaGetLastError(); if(error!=cudaSuccess) return error;
    error=cudaDeviceSynchronize(); if(error!=cudaSuccess) return error;
    return cudaMemcpy(&host,packet_,sizeof(Packet),cudaMemcpyDeviceToHost);
  }
  ~DevicePacket() { if(packet_) EXPECT_EQ(cudaFree(packet_),cudaSuccess); }
  DevicePacket()=default;
  DevicePacket(const DevicePacket&)=delete;
  DevicePacket& operator=(const DevicePacket&)=delete;
 private:
  Packet* packet_=nullptr;
};
void Guards(const Packet& p) { EXPECT_EQ(p.before,kCanary); EXPECT_EQ(p.after,kCanary); }

TEST(QephKinematicsCuda, ActualNativeAndHostAllFieldParityForEveryDeclaredPattern) {
  DevicePacket device;
  ASSERT_EQ(device.Allocate(),cudaSuccess); // No available CUDA is a test failure.
  for(unsigned fixture=0;fixture<kCases;++fixture) for(unsigned shift=0;shift<4;++shift)
    for(bool transform:{false,true}) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(shift);
      SCOPED_TRACE(transform);
      const auto original=Case(fixture),input=qeph_startup_test::Reparameterize(original,shift,transform);
      Packet packet; native::Reference native_ref;
      ASSERT_EQ(port::InitializeReference(input,packet.reference),port::Status::kSuccess);
      ASSERT_EQ(native::Initialize(NativeInput(input),native_ref),native::Status::kSuccess);
      for(unsigned pattern=0;pattern<3;++pattern) {
        SCOPED_TRACE(pattern);
        packet.interval=qeph_kinematics_test::Reparameterize(Pattern(original,pattern),shift,transform);
        const auto ref_bytes=Bytes(packet.reference);
        const auto interval_bytes=Bytes(packet.interval);
        port::Kinematics host; native::Kinematics truth;
        ASSERT_EQ(port::EvaluatePrescribed(packet.reference,packet.interval,host),port::Status::kSuccess);
        ASSERT_EQ(native::EvaluatePrescribed(native_ref,NativeInterval(packet.interval),truth),native::Status::kSuccess);
        ASSERT_EQ(device.Run(packet),cudaSuccess); Guards(packet);
        ASSERT_EQ(packet.status,port::Status::kSuccess);
        EXPECT_EQ(Bytes(packet.reference),ref_bytes); EXPECT_EQ(Bytes(packet.interval),interval_bytes);
        qeph_kinematics_test::Agreement(packet.output,host,packet.interval);
        qeph_kinematics_test::Agreement(packet.output,truth,packet.interval);
      }
    }
  RecordProperty("native_parity_configurations",216);
  RecordProperty("owned_device_bytes",static_cast<int>(sizeof(Packet)));
  RecordProperty("kernel_threads",1);
  RecordProperty("force_or_dynamics_qualified","false");
}

TEST(QephKinematicsCuda, IndependentAffineAndRigidPhaseTruthAtSourceScale) {
  DevicePacket device;
  ASSERT_EQ(device.Allocate(),cudaSuccess);
  for(unsigned fixture:{0u,2u,4u}) {
    const auto input=Case(fixture); Packet packet;
    ASSERT_EQ(port::InitializeReference(input,packet.reference),port::Status::kSuccess);
    for(unsigned mode=0;mode<8;++mode) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(mode);
      packet.interval=Affine(input,mode);
      ASSERT_EQ(device.Run(packet),cudaSuccess); Guards(packet);
      ASSERT_EQ(packet.status,port::Status::kSuccess); AffineTruth(packet.output,input,mode);
      Field(packet.output.characteristic_length,NativeRectangleLength(input.position[1].x,input.position[2].y),
            LengthScale(packet.interval),kRoundoff,"characteristic_length");
    }
    for(Vec3 omega:{Vec3{0,0,1},Unit({1,2,3})}) for(double dt:{.04,.02,.01}) {
      packet.interval=Rigid(input,omega,dt);
      ASSERT_EQ(device.Run(packet),cudaSuccess); Guards(packet);
      ASSERT_EQ(packet.status,port::Status::kSuccess); RigidTruth(packet.output,packet.interval,omega);
    }
  }
  for(double warp:{1e-6,1e-3}) {
    auto input=Case(0); Packet packet;
    for(unsigned n=0;n<4;++n) input.position[n].z=n%2?-warp:warp;
    ASSERT_EQ(port::InitializeReference(input,packet.reference),port::Status::kSuccess);
    packet.interval=Stationary(input);
    ASSERT_EQ(device.Run(packet),cudaSuccess); Guards(packet);
    ASSERT_EQ(packet.status,port::Status::kSuccess);
    EXPECT_EQ(packet.output.planar,warp*warp<1.25e-8);
    Field(packet.output.raw_warpage_abs,warp,Scale(input));
    Field(packet.output.effective_warpage,packet.output.planar?0:warp,Scale(input));
    ZeroRates(packet.output,packet.interval);
  }
}

TEST(QephKinematicsCuda, CurrentGeometryExtensionUsesTheSameFiniteHostAndDeviceOperator) {
  DevicePacket device;ASSERT_EQ(device.Allocate(),cudaSuccess);
  Packet packet;ASSERT_EQ(port::InitializeReference(Case(1),packet.reference),port::Status::kSuccess);
  const auto reference=Bytes(packet.reference);
  for(unsigned kind:{3u,7u}) {
    SCOPED_TRACE(kind);packet.interval=ExtendedCurrentInterval(kind);
    port::Kinematics expected;
    ASSERT_EQ(port::EvaluatePrescribed(packet.reference,packet.interval,expected),port::Status::kSuccess);
    ASSERT_EQ(device.Run(packet),cudaSuccess);Guards(packet);
    ASSERT_EQ(packet.status,port::Status::kSuccess);
    qeph_kinematics_test::Agreement(packet.output,expected,packet.interval);
    EXPECT_EQ(Bytes(packet.reference),reference);
  }
}

TEST(QephKinematicsCuda, MalformedPODAndLateFailurePreserveEntirePublicationAndRetry) {
  DevicePacket device;
  ASSERT_EQ(device.Allocate(),cudaSuccess);
  Packet packet;
  ASSERT_EQ(port::InitializeReference(Case(1),packet.reference),port::Status::kSuccess);
  const auto good_ref=packet.reference;
  packet.interval=Pattern(Case(1),1);
  const auto good_interval=packet.interval;
  ASSERT_EQ(device.Run(packet),cudaSuccess);
  ASSERT_EQ(packet.status,port::Status::kSuccess);
  const auto saved_output=Bytes(packet.output);
  for(unsigned kind=0;kind<5;++kind) {
    packet.reference=good_ref; CorruptReference(packet.reference,kind);
    const auto saved_ref=Bytes(packet.reference);
    ASSERT_EQ(device.Run(packet),cudaSuccess); Guards(packet);
    EXPECT_EQ(packet.status,port::Status::kInvalidReference);
    EXPECT_EQ(Bytes(packet.output),saved_output); EXPECT_EQ(Bytes(packet.reference),saved_ref);
  }
  packet.reference=good_ref;
  for(unsigned kind=0;kind<8;++kind) {
    SCOPED_TRACE(kind);
    packet.interval=InvalidInterval(kind);
    const auto saved_interval=Bytes(packet.interval);
    ASSERT_EQ(device.Run(packet),cudaSuccess); Guards(packet);
    EXPECT_NE(packet.status,port::Status::kSuccess);
    if(kind==5) EXPECT_EQ(packet.status,port::Status::kNonfiniteResult);
    EXPECT_EQ(Bytes(packet.output),saved_output); EXPECT_EQ(Bytes(packet.interval),saved_interval);
  }
  packet.interval=good_interval; port::Kinematics clean;
  ASSERT_EQ(port::EvaluatePrescribed(good_ref,good_interval,clean),port::Status::kSuccess);
  ASSERT_EQ(device.Run(packet),cudaSuccess); Guards(packet);
  ASSERT_EQ(packet.status,port::Status::kSuccess);
  qeph_kinematics_test::Agreement(packet.output,clean,packet.interval);
}
} // namespace
