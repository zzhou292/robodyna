#pragma once

// Integration-only harness: reuse the owning Chrono setup/force fixture and
// actual TL operation. It neither advances time nor supplies a second model.
#include "ReissnerReferenceFixture.h"
#include "ReissnerShellSetup.h"
#include "lib_src/elements/ReissnerShellForce.h"
#include "lib_src/elements/ReissnerShellAssembly.h"
#include <cuda_runtime.h>
#include <cstring>
#include <limits>

#ifndef CH_REISSNER_CONSISTENT_FRAME_REFERENCE
#error "Complete CUDA forces must be checked against the qualified coherent Chrono reference"
#endif

namespace crash::qualification {
namespace tl_shell = tl::fea::reissner;

struct ShellPacket {
    tl_shell::ShellReference reference;
    tl_shell::ElasticSection section;
    tl_shell::ShellConfiguration configuration;
    tl_shell::ShellResult result;
    tl_shell::ShellResult second_result;
    double shared[6][6]{};
    tl_shell::ShellAssemblyStatus assembly_status = tl_shell::ShellAssemblyStatus::kSuccess;
    tl_shell::ShellStatus status = tl_shell::ShellStatus::kSuccess;
};
static_assert(sizeof(ShellPacket) < 16 * 1024, "Keep the prescribed force fixture bounded");

__global__ void EvaluateShellPacket(ShellPacket* packet) {
    if (blockIdx.x || threadIdx.x) return;
    packet->status = tl_shell::ComputeShellForce(packet->reference, packet->section,
                                               packet->configuration, packet->result);
}

__global__ void AssembleShellPair(ShellPacket* packet) {
    if (blockIdx.x || threadIdx.x) return;
    const std::size_t connectivity[2][4] = {{0, 1, 2, 3}, {4, 0, 3, 5}};
    tl::fea::DeviceNodalForceView view{packet->shared[0], packet->shared[1], packet->shared[2],
                                     packet->shared[3], packet->shared[4], packet->shared[5], 6, 1};
    packet->assembly_status = tl_shell::AccumulateShellForces(connectivity[0], packet->result, view);
    if (packet->assembly_status != tl_shell::ShellAssemblyStatus::kSuccess) return;
    packet->assembly_status = tl_shell::AccumulateShellForces(connectivity[1], packet->second_result, view);
}

inline tl_shell::ShellConfiguration ToShell(const Frames& frames) {
    tl_shell::ShellConfiguration result;
    for (unsigned n = 0; n < 4; ++n) {
        result.position[n] = {frames.x[n].x(), frames.x[n].y(), frames.x[n].z()};
        result.rotation[n] = {frames.q[n].e0(), frames.q[n].e1(), frames.q[n].e2(), frames.q[n].e3()};
    }
    return result;
}

inline Evaluation ToEvaluation(const tl_shell::ShellResult& result) {
    Evaluation value;
    value.energy = result.energy;
    value.bending_energy = result.bending_energy;
    for (unsigned n = 0; n < 4; ++n) {
        value.force[n] = Vec(result.force[n].x, result.force[n].y, result.force[n].z);
        value.couple[n] = Vec(result.couple[n].x, result.couple[n].y, result.couple[n].z);
        for (unsigned c = 0; c < 12; ++c) {
            value.strain[n][c] = result.strain[n][c];
            value.stress[n][c] = result.resultant[n][c];
        }
    }
    return value;
}

inline Frames GeneralShellDeformation() {
    auto frames = Bending(.12, .002);
    for (unsigned n = 0; n < 4; ++n) {
        const auto p = Neutral().x[n];
        frames.x[n].y() += .008 * p.x() * p.y();
        frames.x[n].z() += .015 * p.x() * p.y();
        frames.q[n] = frames.q[n] * Rotation(-.08 * p.y(), Vec(1, 0, 0)) *
                      Rotation(.025 * p.x() * p.y(), Vec(0, 0, 1));
    }
    return frames;
}

inline void ExpectShellAgreement(const tl_shell::ShellResult& actual, const Evaluation& expected) {
    // Declared before execution. Direct arithmetic budgets are separate from
    // independent objectivity/energy derivative gates in the owning fixture.
    constexpr double direct = 3e-12;
    for (unsigned n = 0; n < 4; ++n) {
        for (unsigned c = 0; c < 3; ++c) {
            EXPECT_NEAR(tl_shell::detail::Component(actual.force[n], c), expected.force[n][c],
                        direct * (kForceScale + std::abs(expected.force[n][c])));
            EXPECT_NEAR(tl_shell::detail::Component(actual.couple[n], c), expected.couple[n][c],
                        direct * (kMomentScale + std::abs(expected.couple[n][c])));
        }
        for (unsigned c = 0; c < 12; ++c) {
            EXPECT_NEAR(actual.strain[n][c], expected.strain[n][c], direct * (1 + std::abs(expected.strain[n][c])));
            const double section_scale = c < 6 ? kC : kD;
            EXPECT_NEAR(actual.resultant[n][c], expected.stress[n][c], direct * (section_scale + std::abs(expected.stress[n][c])));
        }
    }
    EXPECT_NEAR(actual.energy, expected.energy, direct * (1 + std::abs(expected.energy)));
    EXPECT_NEAR(actual.bending_energy, expected.bending_energy, direct * (1 + std::abs(expected.bending_energy)));
}

class ReissnerShellCuda : public ReissnerReference {
  protected:
    explicit ReissnerShellCuda(const Frames& rest = Neutral()) : ReissnerReference(rest) {}
    ShellPacket packet;
    ShellPacket* device = nullptr;
    void SetUp() override {
        ReissnerReference::SetUp();
        ASSERT_FALSE(HasFatalFailure());
        crash::reference::CopyReissnerShellSetup(*element, packet.reference, packet.section);
        int count = 0;
        ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess);
        ASSERT_GT(count, 0);
        ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device), sizeof(ShellPacket)), cudaSuccess);
        RecordProperty("owned_device_bytes", std::to_string(sizeof(ShellPacket)));
        RecordProperty("scope", "prescribed_complete_elastic_Q4_force_no_dynamics");
    }
    void TearDown() override {
        if (device) EXPECT_EQ(cudaFree(device), cudaSuccess);
    }
    void RunPacket() {
        ASSERT_EQ(cudaMemcpy(device, &packet, sizeof(packet), cudaMemcpyHostToDevice), cudaSuccess);
        EvaluateShellPacket<<<1, 1>>>(device);
        ASSERT_EQ(cudaGetLastError(), cudaSuccess);
        ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
        ASSERT_EQ(cudaMemcpy(&packet, device, sizeof(packet), cudaMemcpyDeviceToHost), cudaSuccess);
    }
    tl_shell::ShellResult Sample(const Frames& frames) {
        packet.configuration = ToShell(frames);
        RunPacket();
        EXPECT_EQ(packet.status, tl_shell::ShellStatus::kSuccess);
        return packet.result;
    }
    void CheckEnergyDerivatives(const Frames& frames, unsigned first_dof = 0) {
        const auto base = Sample(frames);
        for (unsigned node = 0; node < 4; ++node) for (unsigned dof = first_dof; dof < 6; ++dof) {
            SCOPED_TRACE(node);
            SCOPED_TRACE(dof);
            Vec axis(0, 0, 0); axis[dof % 3] = 1;
            const double restoring = tl_shell::detail::Component(dof < 3 ? base.force[node] : base.couple[node], dof % 3);
            double error[2]; unsigned level = 0;
            for (double h : {2e-5, 1e-5}) {
                double energy[2];
                for (unsigned sign = 0; sign < 2; ++sign) {
                    const double delta = sign ? h : -h;
                    auto changed = frames;
                    if (dof < 3) changed.x[node][dof] += delta;
                    else changed.q[node] = Rotation(delta, axis) * frames.q[node];
                    energy[sign] = Sample(changed).energy;
                }
                const double derivative = (energy[1] - energy[0]) / (2 * h);
                error[level++] = std::abs(derivative + restoring);
                EXPECT_LE(error[level-1], 1e-7 + 2e-6 * std::max(std::abs(derivative), std::abs(restoring)));
            }
            EXPECT_LE(error[1], .35 * error[0] + 2e-7 * (1 + std::abs(restoring)));
        }
        ExpectShellAgreement(Sample(frames), ToEvaluation(base));
    }
};
}  // namespace crash::qualification
