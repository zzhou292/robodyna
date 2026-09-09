#include "ReissnerGaussContractFixture.h"

#include <cstring>
#include <limits>

namespace crash::qualification {
namespace {
Frames OffsetRest() {
    auto value = Neutral();
    value.q = {{Rotation(1.4, Vec(1, 0, 0)), Rotation(-1.6, Vec(1, 2, -1)),
                Rotation(2.3, Vec(-2, .5, 1)), Rotation(-2.2, Vec(.4, -1, 2))}};
    return value;
}
Frames AspectRest() {
    auto value = Neutral();
    for (auto& x : value.x) { x.x() *= .8; x.y() *= 1.25; }
    return value;
}
class OffsetGaussContract : public ReissnerGaussContract {
  protected:
    OffsetGaussContract() : ReissnerGaussContract(OffsetRest()) {}
    Frames Parameterize(const Frames& physical) const {
        auto value = physical;
        for (unsigned n = 0; n < 4; ++n) value.q[n] = value.q[n] * initial.q[n];
        return value;
    }
};
class AspectGaussContract : public ReissnerGaussContract {
  protected:
    AspectGaussContract() : ReissnerGaussContract(AspectRest()) {}
};
class RotatedRestGaussContract : public ReissnerGaussContract {
  protected:
    RotatedRestGaussContract() : ReissnerGaussContract(Transform(Neutral(), Rotation(2.1, Vec(1, -2, .4)), Vec(-.4, .2, .7))) {}
};
}

TEST_F(ReissnerGaussContract, CompleteOutputsMatchScalarStageOneAndActualChrono) {
    StressFree(ToEvaluation(Sample(Neutral())), "gauss_contract_neutral");
    auto shear = Neutral();
    for (auto& x : shear.x) x.z() = .012 * x.x() - .007 * x.y();
    for (const auto& frames : {Axial(.002), shear, Bending(.12), GeneralShellDeformation()})
        Balance(ToEvaluation(Sample(frames)), frames, "gauss_contract_balance");
    const auto original = GeneralShellDeformation(); const auto base = Sample(original);
    for (const auto& q : {Rotation(1.7, Vec(1, 2, -1)), Rotation(2.8, Vec(-2, 1, .3)), Rotation(-2.2, Vec(.4, -1, 2))}) {
        const auto moved = Transform(original, q, Vec(2.1, -1.3, .7)); const auto result = Sample(moved);
        Covariant(ToEvaluation(base), ToEvaluation(result), q, "gauss_contract_loaded");
        Balance(ToEvaluation(result), moved, "gauss_contract_loaded_balance");
    }
    ReferenceUnchanged();
}
TEST_F(ReissnerGaussContract, AllWorldDofsDifferentiateEnergyWithUnchangedBudgets) {
    CheckShellEnergyDerivatives(GeneralShellDeformation(), [this](const Frames& f) { return Sample(f); });
    EXPECT_EQ(full_evaluations, 98U); ReferenceUnchanged();
}
TEST_F(ReissnerGaussContract, AllWorldDofsDifferentiateEnergyAfterCommonMotion) {
    const auto moved = Transform(GeneralShellDeformation(), Rotation(1.7, Vec(1, 2, -1)), Vec(2.1, -1.3, .7));
    CheckShellEnergyDerivatives(moved, [this](const Frames& f) { return Sample(f); });
    EXPECT_EQ(full_evaluations, 98U); ReferenceUnchanged();
}
TEST_F(ReissnerGaussContract, InvalidInputsAndLastGaussFailuresPreserveAllThreeCompleteResultsThenRetry) {
    const auto frames = GeneralShellDeformation(); const auto clean = Sample(frames); const auto valid = packet;
    const auto reject = [this, &valid, &clean](tl_shell::ShellStatus expected) {
        for (unsigned i = 0; i < 3; ++i) std::memset(&packet.result[i], 0x35 + i, sizeof(packet.result[i]));
        const auto seeded = packet; RunPacket();
        for (unsigned i = 0; i < 3; ++i) {
            EXPECT_EQ(packet.status[i], expected);
            EXPECT_EQ(std::memcmp(&packet.result[i], &seeded.result[i], sizeof(packet.result[i])), 0);
        }
        auto host = seeded.result[2];
        EXPECT_EQ(gauss_contract::ComputeShellForceGaussContract(packet.reference, packet.section, packet.configuration, host), expected);
        EXPECT_EQ(std::memcmp(&host, &seeded.result[2], sizeof(host)), 0);
        packet = valid; RunPacket();
        for (const auto status : packet.status) EXPECT_EQ(status, tl_shell::ShellStatus::kSuccess);
        EXPECT_EQ(std::memcmp(&packet.result[2], &clean, sizeof(clean)), 0);
    };
    packet.reference.prepared = false; reject(tl_shell::ShellStatus::kInvalidReference);
    packet.reference.gauss[3].natural[0] = 2; reject(tl_shell::ShellStatus::kInvalidReference);
    packet.section.prepared = false; reject(tl_shell::ShellStatus::kInvalidSection);
    packet.section.stiffness[143] = std::numeric_limits<double>::quiet_NaN(); reject(tl_shell::ShellStatus::kInvalidSection);
    packet.configuration.rotation[2].w *= 2; reject(tl_shell::ShellStatus::kInvalidConfiguration);
    auto outside = frames; outside.q[1] = Rotation(2.2, Vec(1, 0, 0)) * frames.q[1];
    packet.configuration = ToShell(outside); reject(tl_shell::ShellStatus::kOutsideChart);
    packet.reference.ans[3].gradient[0][0] = std::numeric_limits<double>::max(); reject(tl_shell::ShellStatus::kNonfiniteResult);
    packet.reference.gauss[3].gradient[0][0] = std::numeric_limits<double>::max(); reject(tl_shell::ShellStatus::kNonfiniteResult);
    packet.reference.gauss[3].area_weight = std::numeric_limits<double>::max(); reject(tl_shell::ShellStatus::kNonfiniteResult);
    ReferenceUnchanged();
}
TEST_F(OffsetGaussContract, InitialFrameOffsetsPreserveActualPointFieldsForcesAndCovariance) {
    StressFree(ToEvaluation(Sample(initial)), "gauss_contract_offset_rest");
    const auto physical = GeneralShellDeformation(); const auto frames = Parameterize(physical); const auto base = Sample(frames);
    const auto q = Rotation(1.7, Vec(1, 2, -1)); const auto moved = Parameterize(Transform(physical, q, Vec(2.1, -1.3, .7)));
    Covariant(ToEvaluation(base), ToEvaluation(Sample(moved)), q, "gauss_contract_offset");
    CheckPointIntermediatesAndContraction(frames); CheckPointIntermediatesAndContraction(moved);
    CheckPointDerivatives(frames); ReferenceUnchanged();
}
TEST_F(OffsetGaussContract, ReparameterizedWorldSpinsDifferentiateEnergy) {
    const auto physical = GeneralShellDeformation();
    for (const auto& f : {physical, Transform(physical, Rotation(1.7, Vec(1, 2, -1)), Vec(2.1, -1.3, .7))})
        CheckShellEnergyDerivatives(Parameterize(f), [this](const Frames& changed) { return Sample(changed); }, 3);
    EXPECT_EQ(full_evaluations, 100U); ReferenceUnchanged();
}
TEST_F(AspectGaussContract, ChangedAspectMatchesCompleteForcesAndEveryPointColumn) {
    StressFree(ToEvaluation(Sample(initial)), "gauss_contract_aspect_rest");
    auto frames = GeneralShellDeformation();
    for (auto& x : frames.x) { x.x() *= .8; x.y() *= 1.25; }
    Sample(frames); CheckPointIntermediatesAndContraction(frames); CheckPointDerivatives(frames); ReferenceUnchanged();
}
TEST_F(RotatedRestGaussContract, CommonInitialRotationUsesActualChronoPointTables) {
    StressFree(ToEvaluation(Sample(initial)), "gauss_contract_rotated_rest");
    const auto frames = Transform(GeneralShellDeformation(), initial.q[0], Vec(-.4, .2, .7));
    Sample(frames); CheckPointIntermediatesAndContraction(frames); CheckPointDerivatives(frames); ReferenceUnchanged();
}
}  // namespace crash::qualification
