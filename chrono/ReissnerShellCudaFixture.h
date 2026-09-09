#pragma once

// Integration-only harness: reuse the owning Chrono setup/force fixture and
// actual TL operation. It neither advances time nor supplies a second model.
#include "ReissnerShellHostFixture.h"
#include "lib_src/elements/ReissnerShellAssembly.h"
#include <cuda_runtime.h>
#include <cstring>
#include <limits>

namespace crash::qualification {

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
        CheckShellEnergyDerivatives(frames, [this](const Frames& changed) { return Sample(changed); }, first_dof);
    }
};
}  // namespace crash::qualification
