// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "UnitFixture.h"
#include "../radioss_type25_friction/CudaFixture.h"
namespace type25_coefficient_test {
namespace {
using Drain = type25_friction_test::Drain;
class CoefficientCuda : public type25_friction_test::FrictionCuda {};
template<class P> __global__ void EvaluateKernel(const P* packets, Result* results, std::size_t count) {
  for (std::size_t i = blockIdx.x * blockDim.x + threadIdx.x; i < count; i += gridDim.x * blockDim.x)
    results[i] = Evaluate(packets[i]);
}
TEST_F(CoefficientCuda, AllNativePacketsAndSchedulingOrdersMatchTheSourceOracle) {
  auto packets = Cases(); static_assert(sizeof(Packet) <= RowBytes && sizeof(Result) <= RowBytes);
  ASSERT_LE(packets.size(), Capacity);
  std::vector<Result> actual(packets.size());
  Drain drain{stream};
  for (unsigned threads : {32u, 128u}) for (unsigned repeat = 0; repeat < 4; ++repeat) {
    std::reverse(packets.begin(), packets.end());
    ASSERT_EQ(cudaMemcpyAsync(input, packets.data(), packets.size()*sizeof(Packet), cudaMemcpyHostToDevice, stream), cudaSuccess);
    EvaluateKernel<<<2, threads, 0, stream>>>(static_cast<Packet*>(input), static_cast<Result*>(output), packets.size());
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(actual.data(), output, actual.size()*sizeof(Result), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    for (std::size_t i = 0; i < packets.size(); ++i) {
      SCOPED_TRACE(i); ASSERT_EQ(actual[i].status, n::CoefficientStatus::Ok);
      Same(actual[i], Reference(packets[i])); Same(actual[i], Evaluate(packets[i]));
    }
  }
}
TEST_F(CoefficientCuda, AllSiEntryPointsUseTheActualNativeUnitBoundaryOnDevice) {
  static_assert(sizeof(SiPacket) <= RowBytes);
  const auto native = Cases(); const n::UnitScale units{.001, 1000, 1};
  std::vector<SiPacket> packets; for (const auto& p : native) packets.push_back(ToSi(p, units));
  std::vector<Result> actual(packets.size()); Drain drain{stream};
  ASSERT_EQ(cudaMemcpyAsync(input, packets.data(), packets.size()*sizeof(SiPacket), cudaMemcpyHostToDevice, stream), cudaSuccess);
  EvaluateKernel<<<2, 128, 0, stream>>>(static_cast<SiPacket*>(input), static_cast<Result*>(output), packets.size());
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(actual.data(), output, actual.size()*sizeof(Result), cudaMemcpyDeviceToHost, stream), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  for (std::size_t i = 0; i < packets.size(); ++i) Same(actual[i], ScaledReference(native[i], units));
}
struct Rejection {
  n::NativeShellMainCoefficientInput shell;
  n::NativeScalarCoefficient value{73};
  n::CoefficientStatus status = n::CoefficientStatus::Ok;
};
__global__ void RejectKernel(Rejection* packet) {
  packet->status = n::EvaluateNativeShellMainCoefficient(packet->shell, &packet->value);
}
TEST_F(CoefficientCuda, UnsupportedAndNonfiniteContributionPreserveOutputThenRetry) {
  Rejection packet; packet.shell = Shell(); packet.shell.face = n::MainFaceKind::Coating;
  static_assert(sizeof(Rejection) <= RowBytes); Drain drain{stream};
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (attempt == 1) {
      packet.shell = Shell(); packet.shell.scale = 2.;
      packet.shell.element_thickness = std::numeric_limits<double>::max(); packet.shell.young = 0.;
    } else if (attempt == 2) packet.shell = Shell();
    ASSERT_EQ(cudaMemcpyAsync(input, &packet, sizeof(packet), cudaMemcpyHostToDevice, stream), cudaSuccess);
    RejectKernel<<<1, 1, 0, stream>>>(static_cast<Rejection*>(input));
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&packet, input, sizeof(packet), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    EXPECT_EQ(packet.status, attempt == 0 ? n::CoefficientStatus::UnsupportedProfile :
        (attempt == 1 ? n::CoefficientStatus::NonfiniteResult : n::CoefficientStatus::Ok));
    if (attempt < 2) EXPECT_EQ(packet.value.value, 73.);
    else Number(packet.value.value, Oracle(packet.shell).value);
  }
}
} // namespace
} // namespace type25_coefficient_test
