#include "ReissnerReferenceFixture.h"

namespace crash::qualification {
TEST_F(ReissnerReference, NeutralReferenceIsRetainedAcrossRepeatedCurrentConfigurations) {
    const auto zero = Evaluate(initial);
    StressFree(zero, "initial");
    EXPECT_NEAR(zero.area, kArea, 1e-13);
    const auto loaded = Evaluate(Axial(.002));
    ASSERT_TRUE(loaded.finite);
    ASSERT_GT(loaded.energy, 1e-3);
    const auto repeated = Evaluate(Axial(.002));
    Covariant(loaded, repeated, chrono::QUNIT, "repeated");
    StressFree(Evaluate(initial), "restored");
    ReferenceUnchanged();
}

TEST_F(ReissnerReference, FiniteStressFreeRigidMotionsPreserveZeroStrainForceAndEnergy) {
    const std::array<Quat, 4> rotations{{chrono::QUNIT, Rotation(.8, Vec(1, 0, 0)),
                                       Rotation(1.7, Vec(1, 2, -1)), Rotation(2.8, Vec(-2, 1, 3))}};
    for (std::size_t i = 0; i < rotations.size(); ++i) {
        SCOPED_TRACE(i);
        const auto moved = Transform(initial, rotations[i], Vec(2.1, -1.3, .7));
        const auto value = Evaluate(moved);
        StressFree(value, "rigid_" + std::to_string(i));
        ReferenceUnchanged();
    }
}

TEST_F(ReissnerReference, AxialMembraneMatchesIndependentBoundaryTractionAndElasticEnergy) {
    constexpr double strain = .002;
    const auto frames = Axial(strain);
    const auto value = Evaluate(frames);
    ASSERT_TRUE(value.finite);
    // Directors remain identity. Total Reissner axial stretch is lambda-1,
    // so exact section energy for this prescribed deformation is C e^2 A/2.
    const double nx = kC * strain, ny = kNu * nx;
    const double expected_energy = .5 * kArea * kC * strain * strain;
    double force_error = 0, strain_error = 0, resultant_error = 0;
    for (int n = 0; n < 4; ++n) {
        // Integrated traction of adjacent straight reference edges, shared
        // equally by their endpoints; no donor shape derivatives are used.
        const double bx = std::copysign(kWidth / 2, initial.x[n].x());
        const double by = std::copysign(kLength / 2, initial.x[n].y());
        const Vec expected(-nx * bx, -ny * by, 0);  // Restoring force.
        force_error = std::max(force_error, (value.force[n] - expected).Length());
        for (int c = 0; c < 12; ++c) {
            strain_error = std::max(strain_error, std::abs(value.strain[n][c] - (c == 0 ? strain : 0)));
            const double expected_stress = c == 0 ? nx : (c == 4 ? ny : 0);
            resultant_error = std::max(resultant_error, std::abs(value.stress[n][c] - expected_stress));
        }
    }
    Metric("analytic_membrane_force_error_N", force_error, kForceAbs);
    Metric("analytic_membrane_couple_Nm", MaxVector(value.couple), kMomentAbs);
    Metric("analytic_membrane_strain_error", strain_error, kStrainAbs);
    Metric("analytic_membrane_resultant_error", resultant_error, 2 * kC * kStrainAbs);
    Metric("analytic_membrane_energy_error_J", std::abs(value.energy - expected_energy), kRelative * expected_energy);
    Balance(value, frames, "membrane");
    ReferenceUnchanged();
}

TEST_F(ReissnerReference, SmallCurvatureMatchesIndependentBendingStiffnessAndEnergy) {
    // Infinitesimal constant curvature, not an exact finite cylinder. Nodal
    // w=-k*x^2/2 is constant at these four corners; theta_y=k*x. The analytical
    // Kirchhoff limit is Mxx=D*k, Myy=nu*D*k, U=A*D*k^2/2. Finite membrane
    // corrections are O(k^2 L^2), relative energy O(k^2 L^4/h^2). At k=1e-6/m
    // a fixed 2e-6 relative bending tolerance includes that asymptotic error.
    constexpr double curvature = 1e-6;
    constexpr double bending_relative = 2e-6;
    const auto frames = Bending(curvature);
    const auto value = Evaluate(frames);
    ASSERT_TRUE(value.finite);
    const double moment = kD * curvature;
    const double expected_energy = .5 * kArea * kD * curvature * curvature;
    double curvature_error = 0, moment_error = 0, couple_error = 0;
    for (int n = 0; n < 4; ++n) {
        for (int c = 6; c < 12; ++c) {
            const double expected_curvature = c == 7 ? curvature : 0;
            // Chrono's k_v.x is the negative of physical yy curvature.
            const double expected_moment = c == 7 ? moment : (c == 9 ? -kNu * moment : 0);
            curvature_error = std::max(curvature_error, std::abs(value.strain[n][c] - expected_curvature));
            moment_error = std::max(moment_error, std::abs(value.stress[n][c] - expected_moment));
        }
        const double bx = std::copysign(kWidth / 2, initial.x[n].x());
        const double by = std::copysign(kLength / 2, initial.x[n].y());
        const Vec expected_couple(kNu * moment * by, -moment * bx, 0);
        couple_error = std::max(couple_error, (value.couple[n] - expected_couple).Length());
    }
    Metric("analytic_bending_curvature_error_per_m", curvature_error, bending_relative * curvature);
    Metric("analytic_bending_resultant_error_N", moment_error, bending_relative * moment);
    Metric("analytic_bending_couple_error_Nm", couple_error, bending_relative * moment * kLength);
    Metric("analytic_bending_energy_error_J", std::abs(value.energy - expected_energy), bending_relative * expected_energy);
    EXPECT_GT(value.bending_energy, .999 * value.energy);
    ReferenceUnchanged();
}

TEST_F(ReissnerReference, FiniteMotionSuperposedOnMembranePrestressIsObjective) {
    const auto frames = Axial(.002);
    const auto before = Evaluate(frames);
    ASSERT_TRUE(before.finite);
    ASSERT_GT(before.energy, 1e-3);
    const auto q = Rotation(1.7, Vec(1, 2, -1));
    const auto moved = Transform(frames, q, Vec(2.1, -1.3, .7));
    const auto after = Evaluate(moved);
    ASSERT_TRUE(after.finite);
    Covariant(before, after, q, "prestressed_membrane");
    Balance(after, moved, "prestressed_membrane");
    ReferenceUnchanged();
}

TEST_F(ReissnerReference, FiniteMotionSuperposedOnDifferentialBendingIsObjective) {
    // Distinct nodal directors exercise orientation interpolation, unlike a
    // stress-free rigid tile or uniform-director membrane patch. This gate
    // must remain a failure if the linked formulation is not objective.
    const auto frames = Bending(.12, .002);
    const auto before = Evaluate(frames);
    ASSERT_TRUE(before.finite);
    ASSERT_GT(before.bending_energy, 1e-4);
    ASSERT_GT(MaxComponents(before.strain, 6, 12), .1);
    const auto q = Rotation(1.7, Vec(1, 2, -1));
    const auto moved = Transform(frames, q, Vec(2.1, -1.3, .7));
    const auto after = Evaluate(moved);
    ASSERT_TRUE(after.finite);
    Covariant(before, after, q, "prestressed_bending");
    Balance(before, frames, "bending_before");
    Balance(after, moved, "bending_after");
    ReferenceUnchanged();
}

TEST_F(ReissnerReference, MixedConfigurationForceAndCoupleAreEnergyWorkConjugates) {
    const auto frames = Bending(.08, .003);
    const auto base = Evaluate(frames);
    ASSERT_TRUE(base.finite);
    // Central difference of total elastic energy along an independent
    // prescribed translational/rotational variation. Rotations are perturbed
    // on the left, hence work uses the WORLD couples exported above.
    std::array<Vec, 4> dx{{Vec(.2, -.1, .15), Vec(-.1, .2, -.05),
                           Vec(.05, -.15, .12), Vec(-.15, .05, -.1)}};
    std::array<Vec, 4> dtheta{{Vec(.1, .2, -.05), Vec(-.2, .1, .08),
                               Vec(.08, -.15, .1), Vec(.12, -.08, -.1)}};
    double restoring_work = 0;
    for (int n = 0; n < 4; ++n)
        restoring_work += base.force[n].Dot(dx[n]) + base.couple[n].Dot(dtheta[n]);
    constexpr double step = 1e-6;
    double energies[2];
    for (int s = 0; s < 2; ++s) {
        const double delta = s == 0 ? -step : step;
        auto perturbed = frames;
        for (int n = 0; n < 4; ++n) {
            perturbed.x[n] += delta * dx[n];
            perturbed.q[n] = Rotation(delta * dtheta[n].Length(), dtheta[n]) * frames.q[n];
        }
        const auto value = Evaluate(perturbed);
        ASSERT_TRUE(value.finite);
        energies[s] = value.energy;
    }
    const double derivative = (energies[1] - energies[0]) / (2 * step);
    // 2e-6 allows central-difference O(step^2) and subtractive roundoff;
    // it is separate from the exact finite-transform covariance tolerance.
    const double limit = 1e-7 + 2e-6 * std::max(std::abs(derivative), std::abs(restoring_work));
    Metric("mixed_virtual_work_error_J", std::abs(derivative + restoring_work), limit);
    std::cout << std::setprecision(17) << "  energy_derivative_J = " << derivative
              << "; restoring_work_J = " << restoring_work << '\n';
    const auto restored = Evaluate(frames);
    Covariant(base, restored, chrono::QUNIT, "after_energy_probe");
    ReferenceUnchanged();
}
}  // namespace crash::qualification
