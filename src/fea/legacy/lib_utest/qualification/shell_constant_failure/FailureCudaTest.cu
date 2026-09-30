#include "FailureTestSupport.h"
#include <cuda_runtime.h>
#include <limits>

namespace failure_test {
namespace {
constexpr unsigned Count = 8;
struct Packet {
  f::ConstantPlasticFailureParameters parameters;
  f::ConstantPlasticFailureHistory base;
  f::ConstantPlasticFailureInput input;
  f::ConstantPlasticFailureResult result{{0.125, 0, true}, false};
  bool success = false;
};
__global__ void Advance(Packet* packets) {
  const auto i = threadIdx.x;
  if (i < Count) {
    auto& p = packets[i];
    p.success = f::UpdateConstantPlasticFailure(p.parameters, p.base, p.input, p.result);
  }
}
struct DevicePackets {
  Packet* data = nullptr;
  ~DevicePackets() { if (data) cudaFree(data); }
};
}
TEST(ConstantPlasticFailureCuda, ActiveInactiveThresholdAndLateInvalidMatchNativeWithoutTouchingRejectedOutput) {
  int devices = 0;
  ASSERT_EQ(cudaGetDeviceCount(&devices), cudaSuccess);
  ASSERT_GT(devices, 0);
  std::array<Packet, Count> packets;
  for (unsigned i = 0; i < Count; ++i) {
    packets[i].parameters = {0.25};
    packets[i].base = {0.5, 0, true};
    packets[i].input = {i*0.03125, 0.5, true};
  }
  packets[4].input.element_active = false;
  packets[5].base = {1, 0.25, false};
  packets[7].input.native_evaluation_time_s = std::numeric_limits<double>::quiet_NaN();
  const auto before = packets;
  DevicePackets device;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.data), sizeof(packets)), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device.data, packets.data(), sizeof(packets), cudaMemcpyHostToDevice), cudaSuccess);
  Advance<<<1, Count>>>(device.data);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(packets.data(), device.data, sizeof(packets), cudaMemcpyDeviceToHost), cudaSuccess);
  for (unsigned i = 0; i + 1 < Count; ++i) {
    ASSERT_TRUE(packets[i].success);
    CompareNative(before[i].parameters, before[i].base, before[i].input,
        packets[i].result, {1.e6, -2.e5, 3.e4, 0, 0});
  }
  EXPECT_FALSE(packets.back().success);
  EXPECT_EQ(packets.back().result.history.damage, before.back().result.history.damage);
  EXPECT_EQ(packets.back().result.history.failure_time_s, before.back().result.history.failure_time_s);
  EXPECT_EQ(packets.back().result.history.point_active, before.back().result.history.point_active);
  EXPECT_EQ(packets.back().result.failed_now, before.back().result.failed_now);
}
} // namespace failure_test
