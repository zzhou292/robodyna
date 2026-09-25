// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
namespace type25_friction_test {
namespace {
struct Response { n::NativeFrictionResult value; n::NormalStatus status; };
__global__ void Evaluate(const Case* in, Response* out, std::size_t count) {
  for (std::size_t i = blockIdx.x * blockDim.x + threadIdx.x; i < count; i += blockDim.x * gridDim.x)
    out[i].status = n::EvaluateNativeFriction(in[i].normal_config, in[i].controls, in[i].coefficients,
        in[i].input, in[i].history, &out[i].value);
}
class FrictionPointCuda : public FrictionCuda {
 protected:
  void Check(const std::vector<Case>& cases, unsigned threads, bool invalid = false) {
    static_assert(sizeof(Case) <= RowBytes && sizeof(Response) <= RowBytes);
    ASSERT_FALSE(cases.empty()); ASSERT_LE(cases.size(), Capacity);
    std::vector<Response> actual(cases.size());
    for (auto& r : actual) r.value = Sentinel<n::NativeUnitsTag>();
    Drain drain{stream};
    ASSERT_EQ(cudaMemcpyAsync(input, cases.data(), cases.size() * sizeof(Case), cudaMemcpyHostToDevice, stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(output, actual.data(), actual.size() * sizeof(Response), cudaMemcpyHostToDevice, stream), cudaSuccess);
    Evaluate<<<2, threads, 0, stream>>>(static_cast<Case*>(input), static_cast<Response*>(output), cases.size());
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(actual.data(), output, actual.size() * sizeof(Response), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    for (std::size_t i = 0; i < cases.size(); ++i) {
      const auto& c = cases[i]; auto expected = Sentinel<n::NativeUnitsTag>();
      const auto status = n::EvaluateNativeFriction(c.normal_config, c.controls, c.coefficients, c.input, c.history, &expected);
      ASSERT_EQ(actual[i].status, status); Same(actual[i].value, expected, invalid);
      if (!invalid) { ASSERT_EQ(status, n::NormalStatus::Ok); Same(actual[i].value, Oracle(c)); }
      else EXPECT_NE(status, n::NormalStatus::Ok);
    }
  }
};
TEST_F(FrictionPointCuda, FullNativeCorpusAndPermutationMatchAllMeaningfulFields) {
  auto cases = Cases();
  for (unsigned threads : {32u, 128u}) {
    Check(cases, threads); std::reverse(cases.begin(), cases.end());
    Check(cases, threads);
  }
}
TEST_F(FrictionPointCuda, InvalidPacketsPreservePublicationAndNextCallRetries) {
  std::vector<Case> cases(3, Basic());
  cases[0].controls.formulation = 0;
  cases[1].input.normal.normal_velocity += 1;
  cases[2].coefficients.c[5] = std::numeric_limits<double>::max();
  Check(cases, 32, true); Check(Cases(), 64);
}
struct SiPacket {
  n::ResolvedNormalConfig normal_config; n::FrictionControls controls; n::UnitScale units;
  n::SiFrictionCoefficients coefficients; n::SiFrictionInput input; n::SiFrictionHistory history;
};
struct SiResponse { n::SiFrictionResult value; n::NormalStatus status; };
__global__ void EvaluateSi(const SiPacket* in, SiResponse* out) {
  out->status = n::EvaluateSiFriction(in->normal_config, in->controls, in->units,
      in->coefficients, in->input, in->history, &out->value);
}
TEST_F(FrictionPointCuda, PhysicalSiCoefficientAndPhaseBoundaryExecuteOnDevice) {
  static_assert(sizeof(SiPacket) <= RowBytes && sizeof(SiResponse) <= RowBytes);
  const auto native = Basic(); SiPacket packet;
  packet.normal_config = native.normal_config; packet.controls = native.controls;
  packet.units = {0.001, 1000., 1.}; packet.coefficients = {0.1, {0, 0, 0, 0, 0.1, -1.}};
  packet.input.normal = {2e-6, 400000., -0.02, 1e-5, 0., 2., {4., 2., 8., 1.}, {0.25, 0.25, 0.25, 0.25}, 0.};
  packet.input.normal_axis = {0, 0, 1}; packet.input.relative_velocity = {0.005, -0.003, -0.02};
  packet.input.main_vertices[0] = {0, 0, 0}; packet.input.main_vertices[1] = {0.01, 0, 0};
  packet.input.main_vertices[2] = {0.01, 0.01, 0}; packet.input.main_vertices[3] = {0, 0.01, 0};
  packet.input.dt12 = 0.5e-5;
  for (bool boundary : {false, true}) {
    if (boundary) {
      packet.input.normal_axis = Float32BoundaryNormal().input.normal_axis;
      packet.input.normal.normal_velocity = tl::math::fixed3::Dot(packet.input.normal_axis, packet.input.relative_velocity);
    }
    SiResponse actual; auto expected = actual.value;
    const auto status = n::EvaluateSiFriction(packet.normal_config, packet.controls, packet.units,
        packet.coefficients, packet.input, packet.history, &expected);
    ASSERT_EQ(status, n::NormalStatus::Ok);
    Drain drain{stream};
    ASSERT_EQ(cudaMemcpyAsync(input, &packet, sizeof(packet), cudaMemcpyHostToDevice, stream), cudaSuccess);
    EvaluateSi<<<1, 1, 0, stream>>>(static_cast<SiPacket*>(input), static_cast<SiResponse*>(output));
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&actual, output, sizeof(actual), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    ASSERT_EQ(actual.status, status); Same(actual.value, expected);
    const auto reference = Oracle(boundary ? Float32BoundaryNormal() : native);
    EXPECT_NEAR(actual.value.tangent_force.x, reference.tangent_force.x, 1e-14);
    EXPECT_NEAR(actual.value.friction_work, reference.friction_work * 0.001, 1e-18);
  }
}
} // namespace
} // namespace type25_friction_test
