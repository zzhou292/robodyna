#include "TestSupport.h"
#include "../surface_jacobian_majorant/ExactOracle.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace pair_test {
namespace {
struct Result {
  ct::SurfacePenaltyStatus status;
  ct::SurfacePenaltyPacket packet;
};
__global__ void Evaluate(const Case* input, Result* output, unsigned count) {
  const auto i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < count)
    output[i].status = ct::EvaluateSurfacePenaltyPair(input[i].input(), &output[i].packet);
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
void Same(ct::Vec3 a, ct::Vec3 b) {
  EXPECT_EQ(Bits(a.x), Bits(b.x));
  EXPECT_EQ(Bits(a.y), Bits(b.y));
  EXPECT_EQ(Bits(a.z), Bits(b.z));
}
void Same(const ct::WeightedNodalForces& a, const ct::WeightedNodalForces& b) {
  EXPECT_EQ(a.count, b.count);
  for (unsigned i = 0; i < 4; ++i) {
    EXPECT_EQ(a.nodes[i], b.nodes[i]);
    Same(a.forces[i], b.forces[i]);
    Same(a.couples[i], b.couples[i]);
  }
}
void Same(const ct::SurfacePenaltyPacket& a, const ct::SurfacePenaltyPacket& b) {
  EXPECT_EQ(a.valid, b.valid);
  EXPECT_EQ(a.active, b.active);
  EXPECT_EQ(a.count, b.count);
  Same(a.a.position, b.a.position);
  Same(a.a.velocity, b.a.velocity);
  Same(a.b.position, b.b.position);
  Same(a.b.velocity, b.b.velocity);
  EXPECT_EQ(Bits(a.distance_m), Bits(b.distance_m));
  EXPECT_EQ(Bits(a.gap_m), Bits(b.gap_m));
  Same(a.normal, b.normal);
  EXPECT_EQ(Bits(a.normal_velocity_m_s), Bits(b.normal_velocity_m_s));
  EXPECT_EQ(Bits(a.normal_force_n), Bits(b.normal_force_n));
  EXPECT_EQ(Bits(a.elastic_energy_j), Bits(b.elastic_energy_j));
  Same(a.force_a_n, b.force_a_n);
  Same(a.force_b_n, b.force_b_n);
  Same(a.endpoint_a, b.endpoint_a);
  Same(a.endpoint_b, b.endpoint_b);
  const auto& ma = a.normal_majorant;
  const auto& mb = b.normal_majorant;
  EXPECT_EQ(ma.count, mb.count);
  EXPECT_EQ(ma.valid, mb.valid);
  EXPECT_EQ(Bits(ma.stiffness_n_m), Bits(mb.stiffness_n_m));
  EXPECT_EQ(Bits(ma.norm_sum_upper), Bits(mb.norm_sum_upper));
  for (unsigned i = 0; i < 8; ++i) {
    EXPECT_EQ(a.nodes[i].node, b.nodes[i].node);
    EXPECT_EQ(a.nodes[i].translation_fixed_bits, b.nodes[i].translation_fixed_bits);
    Same(a.nodes[i].force_n, b.nodes[i].force_n);
    Same(a.nodes[i].free_force_n, b.nodes[i].free_force_n);
    EXPECT_EQ(ma.nodes[i].node, mb.nodes[i].node);
    EXPECT_EQ(ma.nodes[i].translation_fixed_bits, mb.nodes[i].translation_fixed_bits);
    Same(ma.nodes[i].jacobian, mb.nodes[i].jacobian);
    EXPECT_EQ(Bits(ma.nodes[i].norm_upper), Bits(mb.nodes[i].norm_upper));
    EXPECT_EQ(Bits(ma.nodes[i].diagonal_n_m), Bits(mb.nodes[i].diagonal_n_m));
  }
}
TEST(SurfacePenaltyPairCuda, ConcurrentPacketsExactFieldsRejectionsAndRetry) {
  constexpr unsigned count = 12;
  Case input[count];
  Result actual[count], expected[count];
  ct::SurfacePenaltyPacket seed;
  ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(input[0].input(), &seed), ct::SurfacePenaltyStatus::Ok);
  input[1].a.reference_half_thickness_m = .125;
  input[1].b.reference_half_thickness_m = .125;
  ASSERT_TRUE(RefreshGap(input[1]));
  input[2].a.reference_half_thickness_m = .25;
  input[2].b.reference_half_thickness_m = .25;
  input[2].gap = -0.;
  input[3].b.point.nodes[0] = 0;
  input[3].a.translation_fixed_bits[0] = 5;
  input[3].b.translation_fixed_bits[0] = 5;
  input[3].a.translation_fixed_bits[1] = 4;
  ASSERT_TRUE(RefreshGap(input[3]));
  // Exact 3-4-5 distance exercises a non-axis represented normal.
  for (unsigned i = 0; i < 4; ++i) {
    input[4].x[3 * i] += 3;
    input[4].x[3 * i + 1] += 4;
    input[4].x[3 * i + 2] = 0;
  }
  input[4].a.reference_half_thickness_m = 3;
  input[4].b.reference_half_thickness_m = 3;
  ASSERT_TRUE(RefreshGap(input[4]));
  input[5].v[23] = std::numeric_limits<double>::quiet_NaN();
  input[6] = input[3];
  input[6].b.translation_fixed_bits[0] = 1;
  input[7].b.point = input[7].a.point;
  input[8].b.point.nodes[3] = 8;
  input[9].gap = std::nextafter(input[9].gap, 0.);
  input[10].k = std::numeric_limits<double>::max();
  for (unsigned i = 0; i < 4; ++i) {
    input[10].a.point.weights[i] = i ? 0 : 1;
    input[10].b.point.weights[i] = i ? 0 : 1;
  }
  input[11].a.point.count = input[11].b.point.count = 3;
  input[11].a.point.weights[2] = input[11].b.point.weights[2] = .5;
  ASSERT_TRUE(RefreshGap(input[11]));
  Device<Case> device_input;
  Device<Result> device_output;
  ASSERT_EQ(cudaMalloc(&device_input.data, sizeof(input)), cudaSuccess);
  ASSERT_EQ(cudaMalloc(&device_output.data, sizeof(actual)), cudaSuccess);
  for (unsigned pass = 0; pass < 2; ++pass) {
    for (unsigned i = 0; i < count; ++i) {
      if (pass) input[i] = Case{};
      expected[i].packet = actual[i].packet = seed;
      expected[i].status = ct::EvaluateSurfacePenaltyPair(input[i].input(), &expected[i].packet);
    }
    ASSERT_EQ(cudaMemcpy(device_input.data, input, sizeof(input), cudaMemcpyHostToDevice), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(device_output.data, actual, sizeof(actual), cudaMemcpyHostToDevice), cudaSuccess);
    Evaluate<<<1, 32>>>(device_input.data, device_output.data, count);
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(actual, device_output.data, sizeof(actual), cudaMemcpyDeviceToHost), cudaSuccess);
    for (unsigned i = 0; i < count; ++i) {
      SCOPED_TRACE(i);
      EXPECT_EQ(actual[i].status, expected[i].status);
      Same(actual[i].packet, expected[i].packet);
      EXPECT_TRUE(majorant_test::ExactBounds(actual[i].packet.normal_majorant));
    }
  }
}
} // namespace
} // namespace pair_test
