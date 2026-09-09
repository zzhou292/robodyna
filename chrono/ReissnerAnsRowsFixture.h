#pragma once

// Deliberate baseline/variant parity kernel. Measurement executables include
// only ReissnerShellHostFixture.h and their separately compiled force wrapper.
#include "ReissnerShellHostFixture.h"
#include "lib_utest/qualification/reissner_batch/ReissnerShellForceAnsRows.h"
#include <cuda_runtime.h>
#include <cstring>
#include <limits>

namespace crash::qualification {
namespace ans_rows = tl::qualification::reissner_ans_rows;

struct AnsRowsPacket {
    tl_shell::ShellReference reference;
    tl_shell::ElasticSection section;
    tl_shell::ShellConfiguration configuration;
    tl_shell::ShellResult scalar, compact;
    ans_rows::AnsPointResponse scalar_ans[4], compact_ans[4];
    tl_shell::ShellStatus scalar_status = tl_shell::ShellStatus::kSuccess;
    tl_shell::ShellStatus compact_status = tl_shell::ShellStatus::kSuccess;
    tl_shell::Status scalar_point_status[4]{}, compact_point_status[4]{};
    bool points_only = false;
};
static_assert(sizeof(AnsRowsPacket) < 16 * 1024);

__global__ void EvaluateAnsRowsParity(AnsRowsPacket* packet) {
    if (blockIdx.x || threadIdx.x) return;
    if (!packet->points_only) {
        packet->scalar_status = tl_shell::ComputeShellForce(
            packet->reference, packet->section, packet->configuration, packet->scalar);
        packet->compact_status = ans_rows::ComputeShellForceAnsRows(
            packet->reference, packet->section, packet->configuration, packet->compact);
        return;
    }
    tl_shell::shell_detail::Kinematics state;
    const auto prepared = tl_shell::shell_detail::PrepareKinematics(packet->reference, packet->configuration, state);
    for (unsigned p = 0; p < 4; ++p) {
        packet->scalar_point_status[p] = packet->compact_point_status[p] = prepared;
        if (prepared != tl_shell::Status::kSuccess) continue;
        tl_shell::shell_detail::PointResponse scalar;
        packet->scalar_point_status[p] = tl_shell::shell_detail::EvaluatePoint(
            state, packet->configuration, packet->reference.ans[p], false, scalar);
        if (packet->scalar_point_status[p] == tl_shell::Status::kSuccess) {
            ans_rows::AnsPointResponse extracted;
            for (unsigned axis = 0; axis < 2; ++axis) {
                extracted.strain[axis] = scalar.strain[3 * axis + 2];
                for (unsigned c = 0; c < 24; ++c) extracted.derivative[axis][c] = scalar.derivative[3 * axis + 2][c];
            }
            packet->scalar_ans[p] = extracted;
        }
        packet->compact_point_status[p] = ans_rows::EvaluateTransversePoint(
            state, packet->configuration, packet->reference.ans[p], packet->compact_ans[p]);
    }
}

inline void ExpectAnsAgreement(const ans_rows::AnsPointResponse& actual, const ans_rows::AnsPointResponse& expected) {
    for (unsigned axis = 0; axis < 2; ++axis) {
        EXPECT_NEAR(actual.strain[axis], expected.strain[axis], 3e-12 * (1 + std::abs(expected.strain[axis])));
        for (unsigned c = 0; c < 24; ++c)
            EXPECT_NEAR(actual.derivative[axis][c], expected.derivative[axis][c],
                        3e-12 * (1 + std::abs(expected.derivative[axis][c])));
    }
}

class ReissnerAnsRows : public ReissnerReference {
  protected:
    explicit ReissnerAnsRows(const Frames& rest = Neutral()) : ReissnerReference(rest) {}
    AnsRowsPacket packet;
    AnsRowsPacket* device = nullptr;
    cudaStream_t stream = nullptr;
    unsigned full_evaluations = 0;

    void SetUp() override {
        ReissnerReference::SetUp();
        ASSERT_FALSE(HasFatalFailure());
        crash::reference::CopyReissnerShellSetup(*element, packet.reference, packet.section);
        ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
        ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device), sizeof(packet)), cudaSuccess);
        RecordProperty("scope", "P2_prescribed_ANS_rows_parity_no_dynamics_no_timing_claim");
        RecordProperty("owned_device_bytes", std::to_string(sizeof(packet)));
    }
    void TearDown() override {
        if (stream) EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
        if (device) EXPECT_EQ(cudaFree(device), cudaSuccess);
        if (stream) EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
    }
    void RunPacket() {
        if (!packet.points_only) {
            ASSERT_LT(full_evaluations, 100U);
            ++full_evaluations;
        }
        ASSERT_EQ(cudaMemcpyAsync(device, &packet, sizeof(packet), cudaMemcpyHostToDevice, stream), cudaSuccess);
        EvaluateAnsRowsParity<<<1, 1, 0, stream>>>(device);
        ASSERT_EQ(cudaGetLastError(), cudaSuccess);
        ASSERT_EQ(cudaMemcpyAsync(&packet, device, sizeof(packet), cudaMemcpyDeviceToHost, stream), cudaSuccess);
        ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    }
    tl_shell::ShellResult Sample(const Frames& frames) {
        packet.points_only = false;
        packet.configuration = ToShell(frames);
        RunPacket();
        EXPECT_EQ(packet.scalar_status, tl_shell::ShellStatus::kSuccess);
        EXPECT_EQ(packet.compact_status, tl_shell::ShellStatus::kSuccess);
        ExpectShellAgreement(packet.compact, ToEvaluation(packet.scalar));
        ExpectShellAgreement(packet.compact, Evaluate(frames));
        tl_shell::ShellResult host;
        EXPECT_EQ(ans_rows::ComputeShellForceAnsRows(packet.reference, packet.section, packet.configuration, host),
                  tl_shell::ShellStatus::kSuccess);
        ExpectShellAgreement(packet.compact, ToEvaluation(host));
        return packet.compact;
    }
    void SamplePoints(const Frames& frames) {
        packet.points_only = true;
        packet.configuration = ToShell(frames);
        RunPacket();
        ASSERT_FALSE(HasFatalFailure());
        for (unsigned p = 0; p < 4; ++p) {
            ASSERT_EQ(packet.scalar_point_status[p], tl_shell::Status::kSuccess);
            ASSERT_EQ(packet.compact_point_status[p], tl_shell::Status::kSuccess);
            ExpectAnsAgreement(packet.compact_ans[p], packet.scalar_ans[p]);
        }
    }
    using AnsStrains = std::array<std::array<double, 2>, 4>;
    AnsStrains ActualAnsStrains(const Frames& frames) {
        const auto full = Evaluate(frames);  // Actual Chrono refreshes public ANS fields.
        EXPECT_TRUE(full.finite);
        AnsStrains value;
        for (unsigned p = 0; p < 4; ++p) {
            value[p][0] = element->eps_tilde_1_A[p].z();
            value[p][1] = element->eps_tilde_2_A[p].z();
        }
        return value;
    }
    void CheckPointDerivatives(const Frames& frames) {
        SamplePoints(frames);
        ASSERT_FALSE(HasFatalFailure());
        std::array<ans_rows::AnsPointResponse, 4> analytic;
        std::copy(std::begin(packet.compact_ans), std::end(packet.compact_ans), analytic.begin());
        const auto actual = ActualAnsStrains(frames);
        tl_shell::shell_detail::Kinematics state;
        ASSERT_EQ(tl_shell::shell_detail::PrepareKinematics(packet.reference, ToShell(frames), state), tl_shell::Status::kSuccess);
        for (unsigned p = 0; p < 4; ++p) {
            ans_rows::AnsPointResponse host;
            ASSERT_EQ(ans_rows::EvaluateTransversePoint(state, ToShell(frames), packet.reference.ans[p], host),
                      tl_shell::Status::kSuccess);
            ExpectAnsAgreement(analytic[p], host);
            for (unsigned axis = 0; axis < 2; ++axis)
                EXPECT_NEAR(analytic[p].strain[axis], actual[p][axis], 3e-12 * (1 + std::abs(actual[p][axis])));
        }
        // Independent value-only oracle: actual Chrono's public ANS strains,
        // not a finite difference of the new row helper. These dimensioned
        // budgets are declared before execution for the metre-scale fixture.
        for (unsigned node = 0; node < 4; ++node) for (unsigned dof = 0; dof < 6; ++dof) {
            SCOPED_TRACE(node);
            SCOPED_TRACE(dof);
            Vec direction(0, 0, 0); direction[dof % 3] = 1;
            const double scale = dof < 3 ? 1 / kLength : 1;
            double maximum_error[2]{};
            unsigned level = 0;
            for (double h : {2e-5, 1e-5}) {
                AnsStrains value[2];
                for (unsigned sign = 0; sign < 2; ++sign) {
                    auto changed = frames;
                    const double delta = sign ? h : -h;
                    if (dof < 3) changed.x[node][dof] += delta;
                    else changed.q[node] = Rotation(delta, direction) * frames.q[node];
                    value[sign] = ActualAnsStrains(changed);
                }
                for (unsigned p = 0; p < 4; ++p) for (unsigned axis = 0; axis < 2; ++axis) {
                    const double derivative = (value[1][p][axis] - value[0][p][axis]) / (2 * h);
                    const double expected = analytic[p].derivative[axis][6 * node + dof];
                    const double error = std::abs(derivative - expected);
                    EXPECT_LE(error, 1e-8 * scale + 2e-7 * std::max(std::abs(derivative), std::abs(expected)));
                    maximum_error[level] = std::max(maximum_error[level], error / scale);
                }
                ++level;
            }
            EXPECT_LE(maximum_error[1], .35 * maximum_error[0] + 2e-9);
        }
    }
};
}  // namespace crash::qualification
