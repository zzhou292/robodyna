#pragma once

// Shared HOST test composition. Deliberately mixed parity kernels live in the
// separate .cu implementation, never in a measurement translation unit.
#include "ReissnerShellHostFixture.h"
#include "lib_utest/qualification/reissner_batch/ReissnerShellForceGaussContract.h"
#include <cuda_runtime.h>

namespace crash::qualification {
namespace gauss_contract = tl::qualification::reissner_gauss_contract;

struct GaussContractPacket {
    tl_shell::ShellReference reference;
    tl_shell::ElasticSection section;
    tl_shell::ShellConfiguration configuration;
    tl_shell::ShellResult result[3];  // Scalar, stage-1 ANS, stage-2 contraction.
    tl_shell::ShellStatus status[3]{};
};
struct GaussPointPacket {
    tl_shell::ShellReference reference;
    tl_shell::ShellConfiguration configuration;
    double ans_derivative[4][2][24]{};
    double resultant[4][12]{};
    gauss_contract::GaussPointKinematics point[4];
    double force[4][24]{};
    tl_shell::Status status[4]{};
};
static_assert(sizeof(GaussContractPacket) < 16 * 1024 && sizeof(GaussPointPacket) < 16 * 1024);

class ReissnerGaussContract : public ReissnerReference {
  protected:
    explicit ReissnerGaussContract(const Frames& rest = Neutral()) : ReissnerReference(rest) {}
    GaussContractPacket packet;
    GaussPointPacket points;
    void* device = nullptr;
    cudaStream_t stream = nullptr;
    unsigned full_evaluations = 0;
    void SetUp() override;
    void TearDown() override;
    void RunPacket();
    void RunPoints();
    void ConfigurePoints(const Frames&);
    tl_shell::ShellResult Sample(const Frames&);
    void CheckPointIntermediatesAndContraction(const Frames&);
    void CheckPointDerivatives(const Frames&);
};
}  // namespace crash::qualification
