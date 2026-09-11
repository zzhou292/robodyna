#include "TestSupport.h"
#include "native/NativeOracle.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>

namespace law90_test {
constexpr unsigned QueryCount = 58;
struct DevicePacket {
  law::PreparationInput input{};
  double x[28]{}, y[28]{}, query[QueryCount]{};
  double prepared[33]{};
  law::CurveResult curves[QueryCount]{};
  int status = 0;
  bool late_prepare_preserved = false;
  bool late_lookup_preserved = false;
  bool retry_succeeded = false;
};

__global__ void Evaluate(DevicePacket* packet) {
  law::PreparedMaterial material;
  const law::CurveView curve{packet->x, packet->y, 28};
  auto status = law::PrepareSI(packet->input, curve, material);
  if (status != law::Status::Ok) {
    packet->status = 1;
    return;
  }
  Pack(material, packet->prepared);
  std::uint32_t cursor = 0;
  for (unsigned i = 0; i < QueryCount; ++i) {
    status = law::LookupCurve(material, packet->query[i], cursor, packet->curves[i]);
    if (status != law::Status::Ok) {
      packet->status = 2;
      return;
    }
    cursor = packet->curves[i].cursor;
  }

  const double last = packet->x[27];
  packet->x[27] = packet->x[26];
  status = law::PrepareSI(packet->input, curve, material);
  double after[33];
  Pack(material, after);
  bool same = true;
  for (unsigned i = 0; i < 33; ++i) {
    same = same && after[i] == packet->prepared[i];
  }
  packet->late_prepare_preserved = status == law::Status::InvalidCurve && same;
  law::CurveResult result{13, 17, 19};
  status = law::LookupCurve(material, .01, 0, result);
  packet->late_lookup_preserved = status == law::Status::InvalidCurve &&
      result.stress_pa == 13 && result.slope_pa == 17 && result.cursor == 19;
  packet->x[27] = last;
  packet->retry_succeeded = law::PrepareSI(packet->input, curve, material) == law::Status::Ok &&
      law::LookupCurve(material, .01, 0, result) == law::Status::Ok;
}

TEST(Law90Cuda, OriginalNativePreparationCursorTrajectoryAndLateRetry) {
  DevicePacket packet;
  packet.input = OriginalInput();
  std::copy_n(original_radiator::strain, 28, packet.x);
  std::copy_n(original_radiator::stress_pa, 28, packet.y);
  unsigned at = 0;
  packet.query[at++] = -.1;
  for (unsigned i = 0; i < 28; ++i) packet.query[at++] = packet.x[i];
  packet.query[at++] = .99;
  for (int i = 27; i >= 0; --i) packet.query[at++] = packet.x[i];
  ASSERT_EQ(at, QueryCount);
  DevicePacket* device = nullptr;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device), sizeof(packet)), cudaSuccess);
  struct Release { DevicePacket* p; ~Release() { cudaFree(p); } } release{device};
  ASSERT_EQ(cudaMemcpy(device, &packet, sizeof(packet), cudaMemcpyHostToDevice), cudaSuccess);
  Evaluate<<<1, 1>>>(device);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&packet, device, sizeof(packet), cudaMemcpyDeviceToHost), cudaSuccess);
  ASSERT_EQ(packet.status, 0);
  EXPECT_TRUE(packet.late_prepare_preserved);
  EXPECT_TRUE(packet.late_lookup_preserved);
  EXPECT_TRUE(packet.retry_succeeded);
  double expected[33];
  const auto values = InputValues(packet.input);
  const auto flags = InputFlags(packet.input);
  const int count = 28;
  law90_native_prepare(values.data(), flags.data(), packet.x, packet.y, &count, expected);
  for (unsigned i = 0; i < 33; ++i) {
    ASSERT_TRUE(SameBits(packet.prepared[i], expected[i])) << i;
  }
  int cursor = 0;
  for (unsigned i = 0; i < QueryCount; ++i) {
    double curve[2];
    int next = -1;
    law90_native_curve(packet.x, packet.y, &count, &packet.query[i], &cursor, curve, &next);
    ASSERT_EQ(packet.curves[i].cursor, unsigned(next)) << i;
    ASSERT_TRUE(SameBits(packet.curves[i].stress_pa, curve[0])) << i;
    ASSERT_TRUE(SameBits(packet.curves[i].slope_pa, curve[1])) << i;
    cursor = next;
  }
}
}  // namespace law90_test
