#include "ReissnerAnsRowsFixture.h"

namespace crash::qualification {
namespace {
Frames TransverseShear() {
    auto value = Neutral();
    for (auto& x : value.x) x.z() = .012 * x.x() - .007 * x.y();
    return value;
}
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
class OffsetAnsRows : public ReissnerAnsRows {
  protected:
    OffsetAnsRows() : ReissnerAnsRows(OffsetRest()) {}
    Frames Parameterize(const Frames& physical) const {
        auto value = physical;
        for (unsigned n = 0; n < 4; ++n) value.q[n] = value.q[n] * initial.q[n];
        return value;
    }
};
class AspectAnsRows : public ReissnerAnsRows {
  protected:
    AspectAnsRows() : ReissnerAnsRows(AspectRest()) {}
};
class RotatedRestAnsRows : public ReissnerAnsRows {
  protected:
    RotatedRestAnsRows() : ReissnerAnsRows(Transform(Neutral(), Rotation(2.1, Vec(1, -2, .4)), Vec(-.4, .2, .7))) {}
};
}  // namespace

TEST_F(ReissnerAnsRows, CompleteHostAndDeviceForcesMatchActualChrono) {
    StressFree(ToEvaluation(Sample(Neutral())), "ans_rows_neutral");
    for (const auto& frames : {Axial(.002), TransverseShear(), Bending(.12), GeneralShellDeformation()}) {
        const auto result = Sample(frames);
        Balance(ToEvaluation(result), frames, "ans_rows_balance");
    }
    ReferenceUnchanged();
}

TEST_F(ReissnerAnsRows, LoadedCommonMotionPreservesForcesCouplesStrainsAndEnergy) {
    const auto original = GeneralShellDeformation();
    const auto base = Sample(original);
    for (const auto& q : {Rotation(1.7, Vec(1, 2, -1)), Rotation(2.8, Vec(-2, 1, .3)),
                          Rotation(-2.2, Vec(.4, -1, 2))}) {
        const auto moved = Transform(original, q, Vec(2.1, -1.3, .7));
        const auto result = Sample(moved);
        Covariant(ToEvaluation(base), ToEvaluation(result), q, "ans_rows_loaded");
        Balance(ToEvaluation(result), moved, "ans_rows_loaded_balance");
    }
    ReferenceUnchanged();
}

TEST_F(ReissnerAnsRows, AllAnsRowsDifferentiateActualChronoInTwentyFourWorldDofs) {
    CheckPointDerivatives(GeneralShellDeformation());
    CheckPointDerivatives(Transform(GeneralShellDeformation(), Rotation(1.7, Vec(1, 2, -1)), Vec(2.1, -1.3, .7)));
    ReferenceUnchanged();
}

TEST_F(ReissnerAnsRows, AllWorldDofsDifferentiateEnergyWithUnchangedBudgets) {
    CheckShellEnergyDerivatives(GeneralShellDeformation(), [this](const Frames& frames) { return Sample(frames); });
    EXPECT_EQ(full_evaluations, 98U);
    ReferenceUnchanged();
}

TEST_F(ReissnerAnsRows, AllWorldDofsDifferentiateEnergyAfterLargeCommonMotion) {
    const auto moved = Transform(GeneralShellDeformation(), Rotation(1.7, Vec(1, 2, -1)), Vec(2.1, -1.3, .7));
    CheckShellEnergyDerivatives(moved, [this](const Frames& frames) { return Sample(frames); });
    EXPECT_EQ(full_evaluations, 98U);
    ReferenceUnchanged();
}

TEST_F(ReissnerAnsRows, DeclaredInvalidInputsAndLateFailuresPreserveCompleteResultsThenRetry) {
    const auto frames = GeneralShellDeformation();
    const auto clean = Sample(frames);
    const auto valid = packet;
    const auto reject = [this, &valid, &clean](tl_shell::ShellStatus expected) {
        std::memset(&packet.scalar, 0x35, sizeof(packet.scalar));
        std::memset(&packet.compact, 0x36, sizeof(packet.compact));
        std::array<unsigned char, sizeof(tl_shell::ShellResult)> scalar_before, compact_before;
        std::memcpy(scalar_before.data(), &packet.scalar, scalar_before.size());
        std::memcpy(compact_before.data(), &packet.compact, compact_before.size());
        RunPacket();
        EXPECT_EQ(packet.scalar_status, expected);
        EXPECT_EQ(packet.compact_status, expected);
        EXPECT_EQ(std::memcmp(scalar_before.data(), &packet.scalar, scalar_before.size()), 0);
        EXPECT_EQ(std::memcmp(compact_before.data(), &packet.compact, compact_before.size()), 0);
        // The host operation is a separate invocation with its own sentinel.
        tl_shell::ShellResult host;
        std::memcpy(&host, compact_before.data(), compact_before.size());
        EXPECT_EQ(ans_rows::ComputeShellForceAnsRows(packet.reference, packet.section, packet.configuration, host), expected);
        EXPECT_EQ(std::memcmp(compact_before.data(), &host, compact_before.size()), 0);
        packet = valid;
        RunPacket();
        EXPECT_EQ(packet.scalar_status, tl_shell::ShellStatus::kSuccess);
        EXPECT_EQ(packet.compact_status, tl_shell::ShellStatus::kSuccess);
        EXPECT_EQ(std::memcmp(&packet.compact, &clean, sizeof(clean)), 0);
    };
    packet.reference.prepared = false;
    reject(tl_shell::ShellStatus::kInvalidReference);
    packet.reference.ans[3].natural[0] = 2;
    reject(tl_shell::ShellStatus::kInvalidReference);
    packet.section.prepared = false;
    reject(tl_shell::ShellStatus::kInvalidSection);
    packet.section.stiffness[143] = std::numeric_limits<double>::quiet_NaN();
    reject(tl_shell::ShellStatus::kInvalidSection);
    packet.configuration.rotation[2].w *= 2;
    reject(tl_shell::ShellStatus::kInvalidConfiguration);
    auto outside = frames;
    outside.q[1] = Rotation(2.2, Vec(1, 0, 0)) * frames.q[1];
    packet.configuration = ToShell(outside);
    reject(tl_shell::ShellStatus::kOutsideChart);
    packet.configuration.position[2] = {1e308, -1e308, 1e308};
    reject(tl_shell::ShellStatus::kNonfiniteResult);
    packet.reference.ans[3].gradient[0][0] = std::numeric_limits<double>::max();
    reject(tl_shell::ShellStatus::kNonfiniteResult);
    packet.reference.gauss[3].area_weight = std::numeric_limits<double>::max();
    reject(tl_shell::ShellStatus::kNonfiniteResult);
    ReferenceUnchanged();
}

TEST_F(ReissnerAnsRows, LastAnsPointFailurePreservesItsRowsAndCleanRetry) {
    SamplePoints(GeneralShellDeformation());
    const auto clean = packet;
    packet.reference.ans[3].gradient[0][0] = std::numeric_limits<double>::max();
    std::memset(&packet.scalar_ans[3], 0x35, sizeof(packet.scalar_ans[3]));
    std::memset(&packet.compact_ans[3], 0x36, sizeof(packet.compact_ans[3]));
    const auto seeded = packet;
    RunPacket();
    for (unsigned p = 0; p < 3; ++p) {
        EXPECT_EQ(packet.scalar_point_status[p], tl_shell::Status::kSuccess);
        EXPECT_EQ(packet.compact_point_status[p], tl_shell::Status::kSuccess);
    }
    EXPECT_EQ(packet.scalar_point_status[3], tl_shell::Status::kNonfiniteResult);
    EXPECT_EQ(packet.compact_point_status[3], tl_shell::Status::kNonfiniteResult);
    EXPECT_EQ(std::memcmp(&packet.scalar_ans[3], &seeded.scalar_ans[3], sizeof(packet.scalar_ans[3])), 0);
    EXPECT_EQ(std::memcmp(&packet.compact_ans[3], &seeded.compact_ans[3], sizeof(packet.compact_ans[3])), 0);
    tl_shell::shell_detail::Kinematics state;
    ASSERT_EQ(tl_shell::shell_detail::PrepareKinematics(packet.reference, packet.configuration, state), tl_shell::Status::kSuccess);
    auto host = seeded.compact_ans[3];
    EXPECT_EQ(ans_rows::EvaluateTransversePoint(state, packet.configuration, packet.reference.ans[3], host),
              tl_shell::Status::kNonfiniteResult);
    EXPECT_EQ(std::memcmp(&host, &seeded.compact_ans[3], sizeof(host)), 0);
    packet = clean;
    RunPacket();
    EXPECT_EQ(std::memcmp(packet.compact_ans, clean.compact_ans, sizeof(packet.compact_ans)), 0);
}

TEST_F(OffsetAnsRows, InitialNodeFrameOffsetsPreservePhysicalForcesAndPointDerivatives) {
    StressFree(ToEvaluation(Sample(initial)), "ans_rows_offset_rest");
    const auto physical = GeneralShellDeformation();
    const auto base_frames = Parameterize(physical);
    const auto base = Sample(base_frames);
    const auto q = Rotation(1.7, Vec(1, 2, -1));
    const auto moved = Parameterize(Transform(physical, q, Vec(2.1, -1.3, .7)));
    Covariant(ToEvaluation(base), ToEvaluation(Sample(moved)), q, "ans_rows_offset_motion");
    CheckPointDerivatives(base_frames);
    CheckPointDerivatives(moved);
    ReferenceUnchanged();
}

TEST_F(OffsetAnsRows, ReparameterizedWorldSpinsDifferentiateEnergy) {
    const auto physical = GeneralShellDeformation();
    for (const auto& frames : {physical, Transform(physical, Rotation(1.7, Vec(1, 2, -1)), Vec(2.1, -1.3, .7))})
        CheckShellEnergyDerivatives(Parameterize(frames), [this](const Frames& changed) { return Sample(changed); }, 3);
    EXPECT_EQ(full_evaluations, 100U);
    ReferenceUnchanged();
}

TEST_F(AspectAnsRows, ChangedAspectReferenceMatchesForcesAndActualPointDerivatives) {
    StressFree(ToEvaluation(Sample(initial)), "ans_rows_aspect_rest");
    auto frames = GeneralShellDeformation();
    for (auto& x : frames.x) { x.x() *= .8; x.y() *= 1.25; }
    Sample(frames);
    CheckPointDerivatives(frames);
    ReferenceUnchanged();
}

TEST_F(RotatedRestAnsRows, CommonInitialRotationUsesActualChronoReferenceTables) {
    StressFree(ToEvaluation(Sample(initial)), "ans_rows_rotated_rest");
    const auto frames = Transform(GeneralShellDeformation(), initial.q[0], Vec(-.4, .2, .7));
    Sample(frames);
    CheckPointDerivatives(frames);
    ReferenceUnchanged();
}
}  // namespace crash::qualification
