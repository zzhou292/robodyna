// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "Assertions.h"
#include <cuda_runtime.h>
#include <algorithm>
namespace type25_normal_test {
namespace {
struct Result { normal::NativeNormalResult response; normal::NormalStatus status; };
__global__ void Evaluate(const Case* input, Result* output, std::size_t count) {
  for (std::size_t i = blockIdx.x * blockDim.x + threadIdx.x; i < count;
       i += blockDim.x * gridDim.x)
    output[i].status = normal::EvaluateNativeNormal(input[i].config, input[i].input,
        input[i].history, &output[i].response);
}
struct Drain {
  cudaStream_t stream;
  ~Drain() { EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess); }
};
class Type25NormalCuda : public ::testing::Test {
 protected:
  static constexpr std::size_t Capacity = 512;
  cudaStream_t stream = nullptr;
  Case* input = nullptr;
  Result* output = nullptr;
  void SetUp() override {
    ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&input), Capacity * sizeof(Case)), cudaSuccess);
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&output), Capacity * sizeof(Result)), cudaSuccess);
  }
  void TearDown() override {
    if (stream) EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    if (output) EXPECT_EQ(cudaFree(output), cudaSuccess);
    if (input) EXPECT_EQ(cudaFree(input), cudaSuccess);
    if (stream) EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
  }
  void Check(const std::vector<Case>& rows, unsigned threads, bool invalid = false) {
    ASSERT_LE(rows.size(), Capacity); ASSERT_FALSE(rows.empty());
    std::vector<Result> results(rows.size());
    for (auto& r : results) { r.response = Sentinel(); r.status = normal::NormalStatus::UnsupportedProfile; }
    Drain drain{stream};
    ASSERT_EQ(cudaMemcpyAsync(input, rows.data(), rows.size() * sizeof(Case), cudaMemcpyHostToDevice, stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(output, results.data(), results.size() * sizeof(Result), cudaMemcpyHostToDevice, stream), cudaSuccess);
    Evaluate<<<2, threads, 0, stream>>>(input, output, rows.size());
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(results.data(), output, results.size() * sizeof(Result), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    for (std::size_t i = 0; i < rows.size(); ++i) {
      SCOPED_TRACE(i); const auto& c = rows[i];
      auto expected = Sentinel();
      const auto status = normal::EvaluateNativeNormal(c.config, c.input, c.history, &expected);
      ASSERT_EQ(results[i].status, status);
      Same(results[i].response, expected, invalid);
      if (!invalid) {
        ASSERT_EQ(status, normal::NormalStatus::Ok);
        Same(results[i].response, Oracle(c.config, c.input, c.history));
      } else EXPECT_NE(status, normal::NormalStatus::Ok);
    }
  }
};
TEST_F(Type25NormalCuda, AllBranchesMatchIndependentFortranAndHost) { Check(Cases(), 64); }
TEST_F(Type25NormalCuda, LaunchOrderAndRepeatPreserveCompleteOutputs) {
  auto rows = Cases();
  for (unsigned threads : {32u, 128u}) for (unsigned repeat = 0; repeat < 3; ++repeat) {
    std::reverse(rows.begin(), rows.end()); Check(rows, threads);
  }
}
TEST_F(Type25NormalCuda, FailedPacketsPreserveOutputAndNextCallRetriesCleanly) {
  std::vector<Case> rows(3, Basic());
  rows[0].config.engine.kdtint = -1;
  rows[1].input.stiffness = std::numeric_limits<double>::max(); rows[1].input.penetration = 10.;
  rows[2].input.main_mass[3] = std::numeric_limits<double>::quiet_NaN();
  Check(rows, 32, true); Check(Cases(), 64);
}
__global__ void EvaluateSiPacket(normal::ResolvedNormalConfig config, normal::UnitScale units,
    normal::SiNormalInput input, normal::SiNormalHistory history,
    normal::SiNormalResult* result, normal::NormalStatus* status) {
  *status = normal::EvaluateSiNormal(config, units, input, history, result);
}
struct SiMemory {
  cudaStream_t stream;
  normal::SiNormalResult* result = nullptr;
  normal::NormalStatus* status = nullptr;
  ~SiMemory() {
    EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    if (status) EXPECT_EQ(cudaFree(status), cudaSuccess);
    if (result) EXPECT_EQ(cudaFree(result), cudaSuccess);
  }
};
TEST_F(Type25NormalCuda, ExplicitSiBoundaryAndMmThresholdRunOnDevice) {
  SiMemory memory{stream};
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&memory.result), sizeof(*memory.result)), cudaSuccess);
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&memory.status), sizeof(*memory.status)), cudaSuccess);
  auto config = Basic().config;
  normal::SiNormalInput si{2e-6, 400000., -0.02, 1e-5, 0., 2.,
      {4., 2., 8., 1.}, {0.25, 0.25, 0.25, 0.25}, 0.};
  normal::SiNormalHistory history;
  for (unsigned test = 0; test < 3; ++test) {
    normal::UnitScale units{0.001, 1000., 1.};
    if (test == 1) {
      si.time = 1.; si.dt = 0; si.normal_velocity = 0;
      config.damping_factor = 0;
      history.previous_penetration = 1e-6; history.previous_stiffness = 300000.;
      si.penetration = (0.001 + 2 * normal::native_constant::epp) * 0.001;
    }
    if (test == 2) units = {};
    normal::SiNormalResult result; result.normal_force = 321.;
    auto expected = result;
    const auto expected_status = normal::EvaluateSiNormal(config, units, si, history, &expected);
    normal::NormalStatus status;
    Drain drain{stream}; // Host result/status remain alive through every queued copy.
    ASSERT_EQ(cudaMemcpyAsync(memory.result, &result, sizeof(result), cudaMemcpyHostToDevice, stream), cudaSuccess);
    EvaluateSiPacket<<<1, 1, 0, stream>>>(config, units, si, history, memory.result, memory.status);
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&result, memory.result, sizeof(result), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&status, memory.status, sizeof(status), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    ASSERT_EQ(status, expected_status); Same(result, expected, test == 2);
    if (test == 1) EXPECT_LT(result.history.staged_stiffness, 300000.);
  }
}
} // namespace
} // namespace type25_normal_test
