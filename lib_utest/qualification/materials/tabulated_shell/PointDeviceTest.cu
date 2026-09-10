#include "PointSequence.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using tl::material::point_test::SequenceResult;
__global__ void Sequences(SequenceResult* output,bool rate) {
  if (threadIdx.x < 8) output[threadIdx.x] = tl::material::point_test::RunSequence(threadIdx.x,rate);
}
void Same(double a, double b) {
  ASSERT_TRUE(std::isfinite(a)); ASSERT_TRUE(std::isfinite(b));
  EXPECT_LE(std::abs(a - b), 64*std::numeric_limits<double>::epsilon()*
      std::max({1., std::abs(a), std::abs(b)}));
}
void CheckSequences(bool rate) {
  SequenceResult* device = nullptr;
  ASSERT_EQ(cudaMalloc(&device, 8*sizeof(SequenceResult)), cudaSuccess);
  struct Release { SequenceResult* p; ~Release() { cudaFree(p); } } release{device};
  Sequences<<<1, 8>>>(device,rate);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  SequenceResult results[8];
  ASSERT_EQ(cudaMemcpy(results, device, sizeof results, cudaMemcpyDeviceToHost), cudaSuccess);
  for (unsigned i = 0; i < 8; ++i) {
    SCOPED_TRACE(i);
    const auto host = tl::material::point_test::RunSequence(i,rate);
    ASSERT_EQ(results[i].status, host.status);
    ASSERT_EQ(results[i].accepted_intervals, i == 7 ? 0u : 64u);
    ASSERT_EQ(results[i].accepted_intervals, host.accepted_intervals);
    const auto& a = results[i].endpoint;
    const auto& b = host.endpoint;
    for (int j = 0; j < 5; ++j) Same(a.history.stress[j], b.history.stress[j]);
    Same(a.history.plastic_strain, b.history.plastic_strain);
    Same(a.history.filtered_rate_per_s,b.history.filtered_rate_per_s);
    Same(a.plastic_increment, b.plastic_increment); Same(a.tangent_ratio, b.tangent_ratio);
    Same(a.elastic_thickness_strain, b.elastic_thickness_strain);
    Same(a.plastic_thickness_strain, b.plastic_thickness_strain);
    Same(a.yield_before_pa, b.yield_before_pa);
    Same(a.equivalent_stress_pa, b.equivalent_stress_pa);
    Same(a.plastic_work_density, b.plastic_work_density);
  }
}
TEST(TabulatedShellPlasticityDevice, CyclicSequencesAndRejectionMatchHost) { CheckSequences(false); }
TEST(TabulatedShellPlasticityDevice, FilteredSourceRateSequencesMatchHost) { CheckSequences(true); }
} // namespace
