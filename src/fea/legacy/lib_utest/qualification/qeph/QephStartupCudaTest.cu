#include "QephStartupFixture.h"
#include <cuda_runtime.h>

namespace {
using namespace qeph_startup_test;
constexpr std::uint64_t kGuard=0x5145504853544152ULL;
struct Packet {
  std::uint64_t before=kGuard;
  port::ReferenceInput input;
  port::ReferenceData output;
  port::Status status=port::Status::kInvalidInput;
  std::uint64_t after=kGuard;
};
static_assert(std::is_trivially_copyable_v<Packet>);
static_assert(sizeof(Packet)<4096);
__global__ void Startup(Packet* packet) {
  packet->status=port::InitializeReference(packet->input,packet->output);
}
// Bounded qualification transport only: one packet, one thread, no physical
// state, accepted history, batch, second solver owner or independent clock.
class DevicePacket {
 public:
  cudaError_t Allocate() { return cudaMalloc(reinterpret_cast<void**>(&packet_),sizeof(Packet)); }
  cudaError_t Run(Packet& host) {
    auto error=cudaMemcpy(packet_,&host,sizeof(Packet),cudaMemcpyHostToDevice);
    if (error!=cudaSuccess) return error;
    Startup<<<1,1>>>(packet_);
    error=cudaGetLastError(); if (error!=cudaSuccess) return error;
    error=cudaDeviceSynchronize(); if (error!=cudaSuccess) return error;
    return cudaMemcpy(&host,packet_,sizeof(Packet),cudaMemcpyDeviceToHost);
  }
  ~DevicePacket() { if (packet_) EXPECT_EQ(cudaFree(packet_),cudaSuccess); }
  DevicePacket()=default;
  DevicePacket(const DevicePacket&)=delete;
  DevicePacket& operator=(const DevicePacket&)=delete;
 private:
  Packet* packet_=nullptr;
};
void Guards(const Packet& packet) { EXPECT_EQ(packet.before,kGuard); EXPECT_EQ(packet.after,kGuard); }

TEST(QephStartupCuda, OneCellKernelMatchesActualNativeAndHostAcrossAllDeclaredFixtures) {
  DevicePacket device;
  ASSERT_EQ(device.Allocate(),cudaSuccess); // Missing CUDA is a failure, not a skip.
  for (unsigned fixture=0;fixture<kCases;++fixture) for (unsigned shift=0;shift<4;++shift)
    for (bool transform:{false,true}) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(shift);
      SCOPED_TRACE(transform);
      Packet packet; packet.input=Reparameterize(Case(fixture),shift,transform);
      const auto source_input=packet.input; native::Reference reference; port::ReferenceData host;
      ASSERT_EQ(native::Initialize(NativeInput(packet.input),reference),native::Status::kSuccess);
      ASSERT_EQ(port::InitializeReference(packet.input,host),port::Status::kSuccess);
      ASSERT_EQ(device.Run(packet),cudaSuccess); Guards(packet);
      ASSERT_EQ(packet.status,port::Status::kSuccess);
      SameInput(packet.input,source_input); SameInput(packet.output.input,source_input);
      Agreement(packet.output,reference.data()); SameResult(packet.output,host);
    }
  RecordProperty("owned_device_bytes",static_cast<int>(sizeof(Packet)));
  RecordProperty("kernel_threads",1);
  RecordProperty("force_or_dynamics_qualified","false");
}

TEST(QephStartupCuda, DeviceDomainAndLateArithmeticRejectionsPreserveBytesAndCleanRetry) {
  DevicePacket device;
  ASSERT_EQ(device.Allocate(),cudaSuccess);
  Packet packet; packet.input=Case(2);
  ASSERT_EQ(port::InitializeReference(packet.input,packet.output),port::Status::kSuccess);
  const auto before=Bytes(packet.output);
  for (unsigned kind=0;kind<9;++kind) {
    SCOPED_TRACE(kind); packet.input=Failure(kind); const auto source_bytes=Bytes(packet.input);
    ASSERT_EQ(device.Run(packet),cudaSuccess); Guards(packet);
    EXPECT_NE(packet.status,port::Status::kSuccess);
    EXPECT_EQ(Bytes(packet.output),before); EXPECT_EQ(Bytes(packet.input),source_bytes);
  }
  // Every adjacent represented input remains within the declared exclusion
  // band. A separate predeclared margin verifies that rejection can be retried.
  for (double half_side:{std::nextafter(2.5e-11,0.),2.5e-11,
                         std::nextafter(2.5e-11,std::numeric_limits<double>::infinity())}) {
    packet.input=Threshold(half_side);
    ASSERT_EQ(device.Run(packet),cudaSuccess);
    EXPECT_EQ(packet.status,port::Status::kUnsupportedGeometry);
    EXPECT_EQ(Bytes(packet.output),before); Guards(packet);
  }
  packet.input=Threshold(2.5e-11*(1+512*std::numeric_limits<double>::epsilon()));
  native::Reference threshold;
  ASSERT_EQ(native::Initialize(NativeInput(packet.input),threshold),native::Status::kSuccess);
  ASSERT_EQ(device.Run(packet),cudaSuccess);
  ASSERT_EQ(packet.status,port::Status::kSuccess); Agreement(packet.output,threshold.data());
  packet.input=Case(2); port::ReferenceData clean;
  ASSERT_EQ(port::InitializeReference(packet.input,clean),port::Status::kSuccess);
  ASSERT_EQ(device.Run(packet),cudaSuccess);
  ASSERT_EQ(packet.status,port::Status::kSuccess); Guards(packet); SameResult(packet.output,clean);
}
}  // namespace
