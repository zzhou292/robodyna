#include <iomanip>
#include "NativeSupport.h"
#include <cuda_runtime.h>

namespace spectrum_test {
constexpr unsigned DeviceCount = 64;
struct DevicePacket {
  Packet inputs[DeviceCount]{};
  double values[DeviceCount][15]{};
  int status[DeviceCount]{};
  bool preserved[DeviceCount]{};
  bool retried[DeviceCount]{};
  unsigned count = 0;
};
__global__ void Evaluate(DevicePacket* packet, bool fault) {
  const unsigned i = blockIdx.x*blockDim.x + threadIdx.x;
  if (i >= packet->count) return;
  Spectrum result;
  if (!tl::math::NativeSymmetricEigen3(packet->inputs[i].tensor, result)) {
    packet->status[i] = 1;
    return;
  }
  Pack(result, packet->inputs[i].rate, packet->values[i]);
  if (!fault) return;
  auto invalid = packet->inputs[i];
  invalid.tensor[5] = HUGE_VAL;
  const bool rejected = !tl::math::NativeSymmetricEigen3(invalid.tensor, result);
  double after[15];
  Pack(result, packet->inputs[i].rate, after);
  bool preserved = rejected;
  for (unsigned k = 0; k < 15; ++k)
    preserved = preserved && __double_as_longlong(after[k]) == __double_as_longlong(packet->values[i][k]);
  packet->preserved[i] = preserved;
  const bool retry = tl::math::NativeSymmetricEigen3(packet->inputs[i].tensor, result);
  Pack(result, packet->inputs[i].rate, after);
  bool same = retry;
  for (unsigned k = 0; k < 15; ++k)
    same = same && __double_as_longlong(after[k]) == __double_as_longlong(packet->values[i][k]);
  packet->retried[i] = same;
}

void CheckDevice(bool fault) {
  const auto inputs = Cases();
  const auto native = Native(inputs);
  ASSERT_EQ(native.size(), inputs.size());
  DevicePacket* device = nullptr;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device), sizeof(DevicePacket)), cudaSuccess);
  struct Release { DevicePacket* p; ~Release() { cudaFree(p); } } release{device};
  for (std::size_t first = 0; first < inputs.size(); first += DeviceCount) {
    DevicePacket packet;
    packet.count = static_cast<unsigned>(std::min<std::size_t>(DeviceCount, inputs.size()-first));
    for (unsigned i = 0; i < packet.count; ++i) packet.inputs[i] = inputs[first+i];
    ASSERT_EQ(cudaMemcpy(device, &packet, sizeof(packet), cudaMemcpyHostToDevice), cudaSuccess);
    Evaluate<<<1, DeviceCount>>>(device, fault);
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet, device, sizeof(packet), cudaMemcpyDeviceToHost), cudaSuccess);
    for (unsigned i = 0; i < packet.count; ++i) {
      ASSERT_EQ(packet.status[i], 0) << first+i;
      ASSERT_TRUE(Compare(packet.values[i], native[first+i].data(), inputs[first+i])) << "packet " << first+i;
      if (fault) {
        ASSERT_TRUE(packet.preserved[i]) << first+i;
        ASSERT_TRUE(packet.retried[i]) << first+i;
      }
    }
  }
}
TEST(NativeSpectrumCuda, MixedNativeOrderedDirectionsAndProjection) { CheckDevice(false); }
TEST(NativeSpectrumCuda, DeviceOwnedLateInvalidPreservationAndRetry) { CheckDevice(true); }
} // namespace spectrum_test
