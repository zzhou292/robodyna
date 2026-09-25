// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
#include "NativeOracle.h"
#include "HistoryAssertions.h"
namespace type25_friction_test {
namespace {
struct PhaseCase { n::NativeContactRow row; n::HistoryPhaseInput input; bool ending = false; };
struct PhaseResponse { n::HistoryPhaseResult value; n::NormalStatus status; };
__global__ void Phases(const PhaseCase* in, PhaseResponse* out, std::size_t count) {
  const auto i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= count) return;
  if (in[i].ending) {
    out[i].value.retained_candidate = false;
    out[i].status = n::EndNativeContact(in[i].row, &out[i].value.row);
  } else out[i].status = n::BeginNativeHistory(in[i].row, in[i].input, &out[i].value);
}
TEST_F(FrictionCuda, EveryRowPhaseBranchMatchesIndependentNativeOnDevice) {
  static_assert(sizeof(PhaseCase) <= RowBytes && sizeof(PhaseResponse) <= RowBytes);
  std::vector<PhaseCase> cases;
  for (unsigned i = 0; i < 6; ++i) {
    PhaseCase c{Row(), {1, 1, 1}};
    if (i == 1) c.input.secondary_stiffness = 0;
    if (i == 2) c.input.main_stiffness = 0;
    if (i == 3) c.input.local_processor = 2;
    if (i == 4) c.row.irtlm[0] = 0;
    if (i == 5) c.row.irtlm[0] = -7;
    cases.push_back(c);
  }
  for (int marker : {-10, -1, 0, 5}) for (bool lost_time : {false, true}) {
    PhaseCase c{Row(), {1, 1, 1}, true}; c.row.irtlm[1] = marker;
    if (lost_time) c.row.time_s[0] = n::native_constant::ep20;
    cases.push_back(c);
  }
  std::vector<PhaseResponse> actual(cases.size()); Drain drain{stream};
  ASSERT_EQ(cudaMemcpyAsync(input, cases.data(), cases.size() * sizeof(PhaseCase), cudaMemcpyHostToDevice, stream), cudaSuccess);
  Phases<<<1, 32, 0, stream>>>(static_cast<PhaseCase*>(input), static_cast<PhaseResponse*>(output), cases.size());
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(actual.data(), output, actual.size() * sizeof(PhaseResponse), cudaMemcpyDeviceToHost, stream), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  for (std::size_t i = 0; i < cases.size(); ++i) {
    ASSERT_EQ(actual[i].status, n::NormalStatus::Ok);
    if (cases[i].ending) SameRow(actual[i].value.row, EndOracle(cases[i].row));
    else {
      const auto reference = BeginOracle(cases[i].row, cases[i].input);
      SameRow(actual[i].value.row, reference.row);
      EXPECT_EQ(actual[i].value.retained_candidate, reference.retained_candidate);
    }
  }
}
struct StepCase { Case response; n::NativeContactRow row; int marker[4]; double time[2]; };
struct StepResult { n::NativeFrictionResult response; n::NativeContactRow row; n::NormalStatus status; };
__global__ void CoupledStep(const StepCase* in, StepResult* out) {
  n::HistoryPhaseResult phase;
  out->status = n::BeginNativeHistory(in->row, {1, 1, 1}, &phase);
  if (out->status != n::NormalStatus::Ok) return;
  // Prescribed native classification is fixture input; no search is fabricated.
  for (unsigned i = 0; i < 4; ++i) phase.row.irtlm[i] = in->marker[i];
  phase.row.time_s[0] = in->time[0]; phase.row.time_s[1] = in->time[1];
  out->status = n::EndNativeContact(phase.row, &phase.row);
  if (out->status != n::NormalStatus::Ok) return;
  n::NativeFrictionResult response;
  const auto& c = in->response;
  out->status = n::EvaluateNativeFriction(c.normal_config, c.controls, c.coefficients,
      c.input, phase.row.history, &response);
  if (out->status != n::NormalStatus::Ok) return;
  phase.row.history = response.history;
  out->response = response; out->row = phase.row;
}
TEST_F(FrictionCuda, MultiStepStickSlipNormalChangeLossAndRecontactMatchNative) {
  static_assert(sizeof(StepCase) <= RowBytes && sizeof(StepResult) <= RowBytes);
  auto actual_row = Row(), reference_row = actual_row;
  const double p[]{0.002, 0.0022, 0.001, 0., 0.003, 0.0031};
  const double speed[]{5, 2000, -2000, 0, 20, -20};
  for (unsigned i = 0; i < 6; ++i) {
    StepCase step; step.response = Basic(); step.row = actual_row;
    auto& c = step.response; c.input.normal.penetration = p[i]; c.input.normal.time = (i + 1) * 1e-5;
    c.input.relative_velocity.x = speed[i]; c.input.dt12 = i == 0 ? 0.5e-5 : 1e-5;
    if (i == 2) {
      c.input.normal_axis = {0.6, 0, 0.8};
      for (auto& v : c.input.main_vertices) v = {0.8*v.x + 0.6*v.z, v.y, -0.6*v.x + 0.8*v.z};
    }
    RefreshNormalVelocity(c);
    step.marker[0] = 7; step.marker[1] = p[i] == 0 ? -5 : 0; step.marker[2] = 1; step.marker[3] = 1;
    step.time[0] = p[i] == 0 ? n::native_constant::ep20 : c.input.normal.time; step.time[1] = 0.2;
    reference_row = BeginOracle(reference_row, {1, 1, 1}).row;
    for (unsigned j = 0; j < 4; ++j) reference_row.irtlm[j] = step.marker[j];
    reference_row.time_s[0] = step.time[0]; reference_row.time_s[1] = step.time[1];
    reference_row = EndOracle(reference_row); c.history = reference_row.history;
    const auto expected = Oracle(c); reference_row.history = expected.history;
    StepResult actual; Drain drain{stream};
    ASSERT_EQ(cudaMemcpyAsync(input, &step, sizeof(step), cudaMemcpyHostToDevice, stream), cudaSuccess);
    CoupledStep<<<1, 1, 0, stream>>>(static_cast<StepCase*>(input), static_cast<StepResult*>(output));
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&actual, output, sizeof(actual), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    ASSERT_EQ(actual.status, n::NormalStatus::Ok); Same(actual.response, expected);
    SameRow(actual.row, reference_row, false); actual_row = actual.row;
  }
}
} // namespace
} // namespace type25_friction_test
