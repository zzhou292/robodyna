#include "ReissnerGaussContractFixture.h"

#include <algorithm>

namespace crash::qualification {
namespace {
__global__ void EvaluateGaussContractParity(GaussContractPacket* packet) {
    if (blockIdx.x || threadIdx.x) return;
    packet->status[0] = tl_shell::ComputeShellForce(packet->reference, packet->section, packet->configuration, packet->result[0]);
    packet->status[1] = tl::qualification::reissner_ans_rows::ComputeShellForceAnsRows(
        packet->reference, packet->section, packet->configuration, packet->result[1]);
    packet->status[2] = gauss_contract::ComputeShellForceGaussContract(
        packet->reference, packet->section, packet->configuration, packet->result[2]);
}
__global__ void EvaluateGaussPointParity(GaussPointPacket* packet) {
    if (blockIdx.x || threadIdx.x) return;
    tl_shell::shell_detail::Kinematics state;
    const auto prepared = tl_shell::shell_detail::PrepareKinematics(packet->reference, packet->configuration, state);
    for (unsigned p = 0; p < 4; ++p) {
        auto& status = packet->status[p]; status = prepared;
        if (status != tl_shell::Status::kSuccess) continue;
        status = gauss_contract::PrepareGaussPoint(state, packet->configuration, packet->reference.gauss[p], packet->point[p]);
        if (status != tl_shell::Status::kSuccess) continue;
        status = gauss_contract::AccumulateGaussPointForce(state, packet->reference.gauss[p], packet->point[p],
                                                         packet->ans_derivative, packet->resultant[p], packet->force[p]);
    }
}
}  // namespace

void ReissnerGaussContract::SetUp() {
    ReissnerReference::SetUp(); ASSERT_FALSE(HasFatalFailure());
    crash::reference::CopyReissnerShellSetup(*element, packet.reference, packet.section);
    ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
    constexpr auto bytes = std::max(sizeof(packet), sizeof(points));
    ASSERT_EQ(cudaMalloc(&device, bytes), cudaSuccess);
    RecordProperty("scope", "P2_Gauss_contraction_prescribed_parity_no_state_clock_or_timing_claim");
    RecordProperty("owned_device_bytes", std::to_string(bytes));
}
void ReissnerGaussContract::TearDown() {
    if (stream) EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    if (device) EXPECT_EQ(cudaFree(device), cudaSuccess);
    if (stream) EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
}
void ReissnerGaussContract::RunPacket() {
    ASSERT_LT(full_evaluations, 100U); ++full_evaluations;
    ASSERT_EQ(cudaMemcpyAsync(device, &packet, sizeof(packet), cudaMemcpyHostToDevice, stream), cudaSuccess);
    EvaluateGaussContractParity<<<1, 1, 0, stream>>>(static_cast<GaussContractPacket*>(device));
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&packet, device, sizeof(packet), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
}
void ReissnerGaussContract::RunPoints() {
    ASSERT_EQ(cudaMemcpyAsync(device, &points, sizeof(points), cudaMemcpyHostToDevice, stream), cudaSuccess);
    EvaluateGaussPointParity<<<1, 1, 0, stream>>>(static_cast<GaussPointPacket*>(device));
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&points, device, sizeof(points), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
}
void ReissnerGaussContract::ConfigurePoints(const Frames& frames) {
    points = {};
    points.reference = packet.reference; points.configuration = ToShell(frames);
    tl_shell::shell_detail::Kinematics state;
    ASSERT_EQ(tl_shell::shell_detail::PrepareKinematics(points.reference, points.configuration, state), tl_shell::Status::kSuccess);
    for (unsigned p = 0; p < 4; ++p) {
        // The point oracle uses the unchanged full operation to obtain ANS B.
        tl_shell::shell_detail::PointResponse sample;
        ASSERT_EQ(tl_shell::shell_detail::EvaluatePoint(state, points.configuration, points.reference.ans[p], false, sample),
                  tl_shell::Status::kSuccess);
        for (unsigned a = 0; a < 2; ++a) for (unsigned c = 0; c < 24; ++c)
            points.ans_derivative[p][a][c] = sample.derivative[3 * a + 2][c];
    }
}
tl_shell::ShellResult ReissnerGaussContract::Sample(const Frames& frames) {
    packet.configuration = ToShell(frames); RunPacket();
    for (const auto status : packet.status) EXPECT_EQ(status, tl_shell::ShellStatus::kSuccess);
    const auto& result = packet.result[2];
    ExpectShellAgreement(result, ToEvaluation(packet.result[0]));
    ExpectShellAgreement(result, ToEvaluation(packet.result[1]));
    ExpectShellAgreement(result, Evaluate(frames));
    tl_shell::ShellResult host;
    EXPECT_EQ(gauss_contract::ComputeShellForceGaussContract(packet.reference, packet.section, packet.configuration, host),
              tl_shell::ShellStatus::kSuccess);
    ExpectShellAgreement(result, ToEvaluation(host));
    return result;
}
}  // namespace crash::qualification
