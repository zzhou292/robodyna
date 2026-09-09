#include "lib_src/elements/ReissnerShellAssembly.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace {
namespace shell = tl::fea::reissner;
struct Packet {
  shell::ShellResult result[2];
  std::size_t node[2][4] = {{0, 1, 2, 3}, {4, 0, 3, 5}};
  double shared[6][6]{};
  unsigned count = 2;
  shell::ShellAssemblyStatus status = shell::ShellAssemblyStatus::kSuccess;
};
__global__ void Scatter(Packet* packet) {
  if (blockIdx.x || threadIdx.x) return;
  tl::fea::DeviceNodalForceView view{packet->shared[0], packet->shared[1], packet->shared[2],
                                   packet->shared[3], packet->shared[4], packet->shared[5], 6, 17};
  for (unsigned element = 0; element < packet->count; ++element) {
    packet->status = shell::AccumulateShellForces(packet->node[element], packet->result[element], view);
    if (packet->status != shell::ShellAssemblyStatus::kSuccess) return;
  }
}
class ReissnerShellAssembly : public ::testing::Test {
 protected:
  Packet* device = nullptr;
  Packet packet;
  void SetUp() override {
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device), sizeof(Packet)), cudaSuccess);
    for (unsigned e = 0; e < 2; ++e) for (unsigned n = 0; n < 4; ++n) {
      const double value = 1 + e * 4 + n;
      packet.result[e].force[n] = {value, -2 * value, 3 * value};
      packet.result[e].couple[n] = {-4 * value, 5 * value, -6 * value};
    }
    for (auto& component : packet.shared) for (auto& value : component) value = .25;
  }
  void TearDown() override {
    if (device) EXPECT_EQ(cudaFree(device), cudaSuccess);
  }
  void Run() {
    ASSERT_EQ(cudaMemcpy(device, &packet, sizeof(Packet), cudaMemcpyHostToDevice), cudaSuccess);
    Scatter<<<1, 1>>>(device);
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet, device, sizeof(Packet), cudaMemcpyDeviceToHost), cudaSuccess);
  }
};

TEST_F(ReissnerShellAssembly, TwoElementSharedNodesAccumulateAllWorldComponents) {
  Run();
  ASSERT_EQ(packet.status, shell::ShellAssemblyStatus::kSuccess);
  // Source contribution magnitudes at shared nodes 0 and 3 sum independently.
  const double expected[6] = {7, 2, 3, 11, 5, 8};
  const double multiplier[6] = {1, -2, 3, -4, 5, -6};
  for (unsigned c = 0; c < 6; ++c) for (unsigned n = 0; n < 6; ++n)
    EXPECT_DOUBLE_EQ(packet.shared[c][n], .25 + multiplier[c] * expected[n]);
}

TEST_F(ReissnerShellAssembly, InvalidConnectivityPreservesContributionAndRetry) {
  packet.count = 1;
  const auto valid = packet;
  for (const auto invalid : {std::size_t(0), std::size_t(6)}) {
    packet = valid;
    packet.node[0][3] = invalid;
    Run();
    EXPECT_EQ(packet.status, shell::ShellAssemblyStatus::kInvalidConnectivity);
    EXPECT_EQ(std::memcmp(packet.shared, valid.shared, sizeof(packet.shared)), 0);
  }
  packet = valid;
  Run();
  EXPECT_EQ(packet.status, shell::ShellAssemblyStatus::kSuccess);
}

TEST_F(ReissnerShellAssembly, LastComponentOverflowPreservesContributionAndCleanRetry) {
  packet.count = 1;
  const auto valid = packet;
  packet.shared[5][3] = -std::numeric_limits<double>::max();
  packet.result[0].couple[3].z = -std::numeric_limits<double>::max();
  const auto before = packet;
  Run();
  EXPECT_EQ(packet.status, shell::ShellAssemblyStatus::kNonfiniteResult);
  EXPECT_EQ(std::memcmp(packet.shared, before.shared, sizeof(packet.shared)), 0);
  packet = valid;
  Run();
  EXPECT_EQ(packet.status, shell::ShellAssemblyStatus::kSuccess);
  EXPECT_DOUBLE_EQ(packet.shared[5][3], .25 - 24);
}

TEST(ReissnerShellAssemblyHost, IncompleteOrAliasedViewIsRejectedBeforePublication) {
  double value = 7;
  const std::size_t node[4] = {0, 1, 2, 3};
  shell::ShellResult result;
  tl::fea::DeviceNodalForceView view;
  EXPECT_EQ(shell::AccumulateShellForces(node, result, view), shell::ShellAssemblyStatus::kInvalidView);
  view = {&value, &value, &value, &value, &value, &value, 4, 1};
  EXPECT_EQ(shell::AccumulateShellForces(node, result, view), shell::ShellAssemblyStatus::kInvalidView);
  EXPECT_DOUBLE_EQ(value, 7);
}
}  // namespace
