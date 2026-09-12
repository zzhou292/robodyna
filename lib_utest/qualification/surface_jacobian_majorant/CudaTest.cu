// SPDX-License-Identifier: MIT
#include "ExactOracle.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace majorant_test {
namespace {
struct Case {
  ct::RepresentedJacobianTerm terms[8];
  unsigned count = 0, node_count = 8;
  double stiffness = 1;
  bool common_normal = false;
  ct::SignedNodeWeight weights[8];
  std::uint8_t fixed[8]{};
  ct::Vec3 normal;
};
struct Result {
  ct::SurfaceMajorantStatus status;
  ct::SurfaceJacobianMajorant packet;
};
__global__ void Evaluate(const Case* input, Result* output, unsigned count) {
  const auto i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= count) return;
  const auto& value = input[i];
  output[i].status = value.common_normal ?
      ct::BuildSignedNormalMajorant(value.weights, value.count, value.fixed, value.node_count,
          value.normal, value.stiffness, &output[i].packet) :
      ct::BuildRepresentedJacobianMajorant(value.terms, value.count,
          value.node_count, value.stiffness, &output[i].packet);
}
template<class T> struct Device {
  T* data = nullptr;
  ~Device() { if (data) cudaFree(data); }
};
std::uint64_t Bits(double value) {
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}
void Same(const ct::SurfaceJacobianMajorant& a, const ct::SurfaceJacobianMajorant& b) {
  ASSERT_EQ(a.valid, b.valid);
  ASSERT_EQ(a.count, b.count);
  EXPECT_EQ(Bits(a.stiffness_n_m), Bits(b.stiffness_n_m));
  EXPECT_EQ(Bits(a.norm_sum_upper), Bits(b.norm_sum_upper));
  for (unsigned i = 0; i < a.count; ++i) {
    EXPECT_EQ(a.nodes[i].node, b.nodes[i].node);
    EXPECT_EQ(a.nodes[i].translation_fixed_bits, b.nodes[i].translation_fixed_bits);
    EXPECT_EQ(Bits(a.nodes[i].jacobian.x), Bits(b.nodes[i].jacobian.x));
    EXPECT_EQ(Bits(a.nodes[i].jacobian.y), Bits(b.nodes[i].jacobian.y));
    EXPECT_EQ(Bits(a.nodes[i].jacobian.z), Bits(b.nodes[i].jacobian.z));
    EXPECT_EQ(Bits(a.nodes[i].norm_upper), Bits(b.nodes[i].norm_upper));
    EXPECT_EQ(Bits(a.nodes[i].diagonal_n_m), Bits(b.nodes[i].diagonal_n_m));
  }
}
}
TEST(SurfaceJacobianMajorantCuda, EightNodeValuesBoundsAndFailurePreservation) {
  constexpr unsigned count = 16;
  Case input[count];
  Result expected[count], actual[count];
  const ct::RepresentedJacobianTerm seed{0, {1, 2, 3}, 0};
  ct::SurfaceJacobianMajorant initial;
  ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(&seed, 1, 1, 4, &initial), ct::SurfaceMajorantStatus::Ok);
  for (unsigned p = 0; p < count; ++p) {
    input[p].count = 8;
    input[p].stiffness = p + 1;
    for (unsigned i = 0; i < 8; ++i) {
      const unsigned node = i % (p % 3 + 5);
      const double sign = i % 2 ? -1 : 1;
      input[p].terms[i] = {node, {sign * 0x1.0000000000001p-2,
          (static_cast<int>(i) - 3) * .125, .03125 * p}, static_cast<std::uint8_t>(node % 8)};
    }
    expected[p].packet = actual[p].packet = initial;
  }
  for (unsigned p = 0; p < 2; ++p) {
    input[p].common_normal = true;
    input[p].count = 3;
    input[p].weights[0] = {1, -.5};
    input[p].weights[1] = {0, .25};
    input[p].weights[2] = {0, .25};
    input[p].normal = {p ? -0x1.0000000000001p0 : 0x1.0000000000001p0, 0, -0.0};
  }
  input[10].terms[7].value.x = std::numeric_limits<double>::quiet_NaN();
  input[11].terms[7].translation_fixed_bits = 8;
  input[12].count = 9;
  input[13].count = 1;
  input[13].terms[0] = {0, {std::numeric_limits<double>::denorm_min(), 0, 0}, 0};
  input[14].count = 1;
  input[14].terms[0] = {0, {1e-160, 0, 0}, 0};
  input[15].count = 1;
  input[15].terms[0] = {0, {std::numeric_limits<double>::max(), 0, 0}, 0};
  for (unsigned p = 0; p < count; ++p) {
    const auto& value = input[p];
    expected[p].status = value.common_normal ?
        ct::BuildSignedNormalMajorant(value.weights, value.count, value.fixed, value.node_count,
            value.normal, value.stiffness, &expected[p].packet) :
        ct::BuildRepresentedJacobianMajorant(value.terms, value.count,
            value.node_count, value.stiffness, &expected[p].packet);
  }
  Device<Case> device_input;
  Device<Result> device_output;
  ASSERT_EQ(cudaMalloc(&device_input.data, sizeof(input)), cudaSuccess);
  ASSERT_EQ(cudaMalloc(&device_output.data, sizeof(actual)), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device_input.data, input, sizeof(input), cudaMemcpyHostToDevice), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device_output.data, actual, sizeof(actual), cudaMemcpyHostToDevice), cudaSuccess);
  Evaluate<<<1, 32>>>(device_input.data, device_output.data, count);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(actual, device_output.data, sizeof(actual), cudaMemcpyDeviceToHost), cudaSuccess);
  for (unsigned p = 0; p < count; ++p) {
    SCOPED_TRACE(p);
    EXPECT_EQ(actual[p].status, expected[p].status);
    Same(actual[p].packet, expected[p].packet);
    EXPECT_TRUE(ExactBounds(actual[p].packet));
  }
}
} // namespace majorant_test
