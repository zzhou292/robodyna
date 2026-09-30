#include "ReissnerShellCudaFixture.h"

namespace crash::qualification {

TEST_F(ReissnerShellCuda, NeutralAndCommonRigidMotionRemainStressFree) {
    StressFree(ToEvaluation(Sample(Neutral())), "cuda_neutral");
    for (const auto& rotation : {Rotation(1.7, Vec(1, 2, -1)), Rotation(3.14159265358979323846, Vec(-2, 1, .3))}) {
        const auto moved = Transform(Neutral(), rotation, Vec(2.1, -1.3, .7));
        StressFree(ToEvaluation(Sample(moved)), "cuda_rigid");
    }
    ReferenceUnchanged();
}

TEST_F(ReissnerShellCuda, CompleteMembraneBendingAndNoncoaxialForcesMatchActualChrono) {
    for (const auto& frames : {Axial(.002), Bending(.12), GeneralShellDeformation()}) {
        const auto cpu = Evaluate(frames);
        const auto gpu = Sample(frames);
        ExpectShellAgreement(gpu, cpu);
        Balance(ToEvaluation(gpu), frames, "cuda_balance");
        tl_shell::ShellResult host;
        ASSERT_EQ(tl_shell::ComputeShellForce(packet.reference, packet.section, ToShell(frames), host),
                  tl_shell::ShellStatus::kSuccess);
        ExpectShellAgreement(gpu, ToEvaluation(host));
    }
    ReferenceUnchanged();
}

TEST_F(ReissnerShellCuda, LoadedLargeCommonRotationsPreserveForcesCouplesAndEnergy) {
    const auto frames = GeneralShellDeformation();
    const auto before = Sample(frames);
    ASSERT_GT(before.bending_energy, 1e-4);
    for (const auto& rotation : {Rotation(1.7, Vec(1, 2, -1)), Rotation(2.8, Vec(-2, 1, .3)),
                                 Rotation(-2.2, Vec(.4, -1, 2))}) {
        const auto moved = Transform(frames, rotation, Vec(2.1, -1.3, .7));
        const auto value = Sample(moved);
        ExpectShellAgreement(value, Evaluate(moved));
        Covariant(ToEvaluation(before), ToEvaluation(value), rotation, "cuda_loaded_rotation");
        Balance(ToEvaluation(value), moved, "cuda_loaded_balance");
    }
}

TEST_F(ReissnerShellCuda, AllTwentyFourWorldDofsDifferentiateEnergyOnActualGpu) {
    const auto original = GeneralShellDeformation();
    for (const auto& frames : {original, Transform(original, Rotation(1.7, Vec(1, 2, -1)), Vec(2.1, -1.3, .7))}) {
        CheckEnergyDerivatives(frames);
    }
    ReferenceUnchanged();
}

TEST_F(ReissnerShellCuda, TwoActualQ4ResultsAccumulateAtConsistentSharedPhysicalNodes) {
    auto left = Neutral();
    auto right = Transform(Neutral(), chrono::QUNIT, Vec(2, 0, 0));
    for (auto* frames : {&left, &right}) for (unsigned n = 0; n < 4; ++n) {
        const double x = frames->x[n].x();
        frames->x[n].x() *= 1.002;
        frames->x[n].z() = -.5 * .08 * x * x;
        frames->q[n] = Rotation(.08 * x, Vec(0, 1, 0));
    }
    ASSERT_EQ(left.x[0], right.x[1]);
    ASSERT_EQ(left.x[3], right.x[2]);
    ASSERT_EQ(left.q[0], right.q[1]);
    ASSERT_EQ(left.q[3], right.q[2]);
    // Both rectangles share the same immutable rest map up to translation.
    // The independent Chrono force path evaluates each current configuration.
    const auto cpu_left = Evaluate(left), cpu_right = Evaluate(right);
    const auto gpu_left = Sample(left), gpu_right = Sample(right);
    ExpectShellAgreement(gpu_left, cpu_left);
    ExpectShellAgreement(gpu_right, cpu_right);
    packet.result = gpu_left;
    packet.second_result = gpu_right;
    for (auto& component : packet.shared) for (auto& value : component) value = .125;
    ASSERT_EQ(cudaMemcpy(device, &packet, sizeof(packet), cudaMemcpyHostToDevice), cudaSuccess);
    AssembleShellPair<<<1, 1>>>(device);
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&packet, device, sizeof(packet), cudaMemcpyDeviceToHost), cudaSuccess);
    ASSERT_EQ(packet.assembly_status, tl_shell::ShellAssemblyStatus::kSuccess);
    const std::array<Vec,6> force{{cpu_left.force[0]+cpu_right.force[1], cpu_left.force[1], cpu_left.force[2],
                                  cpu_left.force[3]+cpu_right.force[2], cpu_right.force[0], cpu_right.force[3]}};
    const std::array<Vec,6> couple{{cpu_left.couple[0]+cpu_right.couple[1], cpu_left.couple[1], cpu_left.couple[2],
                                   cpu_left.couple[3]+cpu_right.couple[2], cpu_right.couple[0], cpu_right.couple[3]}};
    for (unsigned n = 0; n < 6; ++n) for (unsigned c = 0; c < 3; ++c) {
        EXPECT_NEAR(packet.shared[c][n], .125+force[n][c], 6e-12*(kForceScale+std::abs(force[n][c])));
        EXPECT_NEAR(packet.shared[3+c][n], .125+couple[n][c], 6e-12*(kMomentScale+std::abs(couple[n][c])));
    }
}

TEST_F(ReissnerShellCuda, InvalidInputsAndLateOverflowPreserveAllOutputsAndRetry) {
    const auto frames = GeneralShellDeformation();
    const auto expected = Sample(frames);
    const auto valid = packet;
    const auto reject = [this, &valid](tl_shell::ShellStatus expected_status) {
        RunPacket();
        EXPECT_EQ(packet.status, expected_status);
        EXPECT_EQ(std::memcmp(&packet.result, &valid.result, sizeof(packet.result)), 0);
        packet = valid;
        RunPacket();
        EXPECT_EQ(packet.status, tl_shell::ShellStatus::kSuccess);
        ExpectShellAgreement(packet.result, ToEvaluation(valid.result));
    };
    packet.reference.prepared = false;
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
    packet.reference.gauss[3].area_weight = std::numeric_limits<double>::max();
    reject(tl_shell::ShellStatus::kNonfiniteResult);
    ExpectShellAgreement(packet.result, ToEvaluation(expected));
}

namespace {
Frames OffsetInitialFrames() {
    auto frames = Neutral();
    frames.q = {{Rotation(1.4, Vec(1, 0, 0)), Rotation(-1.6, Vec(1, 2, -1)),
                 Rotation(2.3, Vec(-2, .5, 1)), Rotation(-2.2, Vec(.4, -1, 2))}};
    return frames;
}
class OffsetShellCuda : public ReissnerShellCuda {
  protected:
    OffsetShellCuda() : ReissnerShellCuda(OffsetInitialFrames()) {}
};
class RotatedShellCuda : public ReissnerShellCuda {
  protected:
    RotatedShellCuda() : ReissnerShellCuda(Transform(Neutral(), Rotation(2.1, Vec(1, -2, .4)), Vec(-.4, .2, .7))) {}
};
}  // namespace

TEST_F(OffsetShellCuda, DistinctInitialNodeFrameOffsetsPreservePhysicalForcesAndWork) {
    StressFree(ToEvaluation(Sample(initial)), "cuda_offset_rest");
    const auto physical = GeneralShellDeformation();
    for (auto frames : {physical, Transform(physical, Rotation(1.7, Vec(1, 2, -1)), Vec(2.1, -1.3, .7))}) {
        for (unsigned n = 0; n < 4; ++n) frames.q[n] = frames.q[n] * initial.q[n];
        const auto gpu = Sample(frames);
        ExpectShellAgreement(gpu, Evaluate(frames));
        Balance(ToEvaluation(gpu), frames, "cuda_offset_balance");
        CheckEnergyDerivatives(frames, 3);
    }
    ReferenceUnchanged();
}

TEST_F(RotatedShellCuda, RotatedInitialRectangleUsesActualChronoRestFields) {
    StressFree(ToEvaluation(Sample(initial)), "cuda_rotated_rest");
    const auto frames = Transform(GeneralShellDeformation(), initial.q[0], Vec(-.4, .2, .7));
    const auto gpu = Sample(frames);
    ExpectShellAgreement(gpu, Evaluate(frames));
    Balance(ToEvaluation(gpu), frames, "cuda_rotated_reference_balance");
    ReferenceUnchanged();
}

}  // namespace crash::qualification
