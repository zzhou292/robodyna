// Independent, opt-in total-configuration reference, not a production solver.
// Setup follows Chrono's src/demos/fea/demo_FEA_shellsReissner.cpp and the FEA
// manual. The linked ChElementShellReissner4 implements all element arithmetic.
// Only four nodes, one centered elastic layer, and prescribed current frames
// are used here: no integration, constraints, contact, damping, or plasticity.
#include "chrono/ChConfig.h"
#ifndef CHRONO_FEA
#error "The Reissner reference requires a Chrono core built with CH_ENABLE_MODULE_FEA=ON"
#endif

#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChMaterialShellReissner.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChSystemSMC.h"
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace {
using Vec = chrono::ChVector3d;
using Quat = chrono::ChQuaternion<>;
using Node = chrono::fea::ChNodeFEAxyzrot;
using Element = chrono::fea::ChElementShellReissner4;

constexpr double kE = 1.2e6;       // Pa, demo's elastic modulus
constexpr double kNu = 0.3;
constexpr double kThickness = 0.01;  // m
constexpr double kLength = 2.0;      // m, x direction
constexpr double kWidth = 1.0;       // m, y direction
constexpr double kArea = kLength * kWidth;
constexpr double kC = kE * kThickness / (1 - kNu * kNu);
constexpr double kD = kC * kThickness * kThickness / 12;
constexpr double kForceScale = kE * kThickness * kLength;
constexpr double kMomentScale = kForceScale * kLength;
constexpr double kEnergyScale = kMomentScale;

// Declared before any execution. Absolute floors account for binary64
// subtraction at O(1 m) coordinates; relative tolerance is 2e-8 for finite
// objectivity. These are physical gates, not fitted to observed residuals.
constexpr double kStrainAbs = 2e-11;
constexpr double kCurvatureAbs = kStrainAbs / kLength;
constexpr double kRelative = 2e-8;
constexpr double kForceAbs = 2e-11 * kForceScale;
constexpr double kMomentAbs = 2e-11 * kMomentScale;
constexpr double kEnergyAbs = 1e-20 * kEnergyScale;

Quat Rotation(double angle, Vec axis) {
    Quat result;
    result.SetFromAngleAxis(angle, axis.GetNormalized());
    return result;
}

struct Frames {
    std::array<Vec, 4> x;
    std::array<Quat, 4> q;
};

Frames Neutral() {
    // Counterclockwise, starting at (+x,+y). This matches the implementation's
    // natural node coordinates and gives initial material axes world x/y/z.
    return {{{Vec(1, .5, 0), Vec(-1, .5, 0), Vec(-1, -.5, 0), Vec(1, -.5, 0)}},
            {{chrono::QUNIT, chrono::QUNIT, chrono::QUNIT, chrono::QUNIT}}};
}

Frames Transform(const Frames& source, const Quat& q, const Vec& translation) {
    Frames result;
    for (int n = 0; n < 4; ++n) {
        result.x[n] = q.Rotate(source.x[n]) + translation;
        result.q[n] = q * source.q[n];  // Superposed world rotation, left action.
    }
    return result;
}

Frames Axial(double strain) {
    auto result = Neutral();
    for (auto& x : result.x)
        x.x() *= 1 + strain;
    return result;
}

Frames Bending(double curvature, double axial_strain = 0) {
    auto result = Axial(axial_strain);
    const auto neutral = Neutral();
    for (int n = 0; n < 4; ++n) {
        const double x = neutral.x[n].x();
        result.x[n].z() = -.5 * curvature * x * x;
        result.q[n] = Rotation(curvature * x, Vec(0, 1, 0));
    }
    return result;
}

struct Evaluation {
    std::array<Vec, 4> force{}, couple{};  // Both world coordinates.
    std::array<std::array<double, 12>, 4> strain{}, stress{};
    double energy = 0;
    double bending_energy = 0;
    double area = 0;
    bool finite = true;
};

void Metric(const std::string& name, double value, double limit) {
    std::ostringstream text;
    text << std::setprecision(17) << value;
    ::testing::Test::RecordProperty(name, text.str());
    std::ostringstream tolerance;
    tolerance << std::setprecision(17) << limit;
    ::testing::Test::RecordProperty(name + "_limit", tolerance.str());
    std::cout << "  " << name << " = " << text.str() << "; limit = " << tolerance.str() << '\n';
    EXPECT_TRUE(std::isfinite(value)) << name;
    EXPECT_LE(value, limit) << name;
}

double MaxVector(const std::array<Vec, 4>& values) {
    double result = 0;
    for (const auto& v : values)
        result = std::max(result, v.Length());
    return result;
}

double MaxComponents(const std::array<std::array<double, 12>, 4>& values, int first, int last) {
    double result = 0;
    for (const auto& point : values)
        for (int i = first; i < last; ++i)
            result = std::max(result, std::abs(point[i]));
    return result;
}

class ReissnerReference : public ::testing::Test {
  protected:
    chrono::ChSystemSMC system;
    std::shared_ptr<chrono::fea::ChMesh> mesh;
    std::shared_ptr<Element> element;
    std::array<std::shared_ptr<Node>, 4> nodes;
    const Frames initial = Neutral();

    void SetUp() override {
        system.SetNumThreads(1, 1, 1);
        mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
        mesh->SetAutomaticGravity(false);
        system.Add(mesh);
        auto elasticity = chrono_types::make_shared<chrono::fea::ChElasticityReissnerIsothropic>(
            kE, kNu, 5.0 / 6.0, .01);
        auto material = chrono_types::make_shared<chrono::fea::ChMaterialShellReissner>(elasticity);
        material->SetDensity(1000);
        for (int n = 0; n < 4; ++n) {
            nodes[n] = chrono_types::make_shared<Node>(chrono::ChFrame<>(initial.x[n], initial.q[n]));
            nodes[n]->SetMass(0);
            nodes[n]->GetInertia().fillDiagonal(0);
            mesh->AddNode(nodes[n]);
        }
        element = chrono_types::make_shared<Element>();
        element->SetNodes(nodes[0], nodes[1], nodes[2], nodes[3]);
        element->AddLayer(kThickness, 0, material);
        mesh->AddElement(element);
        // Public Setup dispatches private element SetupInitial. Exactly once:
        // another Setup/Relax/SetAsNeutral/ComputeMassProperties may recapture
        // the reference. No dynamics or timestep initialization is performed.
        system.Setup();
        EXPECT_DOUBLE_EQ(element->GetThickness(), kThickness);
        RecordProperty("reference_scope", "one_Q4_prescribed_configuration_no_dynamics");
        RecordProperty("relative_tolerance", "2e-8_fixed_before_execution");
    }

    Evaluation Evaluate(const Frames& frames) {
        for (int n = 0; n < 4; ++n) {
            nodes[n]->SetPos(frames.x[n]);
            nodes[n]->SetRot(frames.q[n]);
        }
        // This public method refreshes strain and forces from current frames.
        // Its Fi.setZero() does not resize. GetStateBlock is deliberately not
        // used: its 28 quaternion coordinates differ from Fi's 24 entries.
        chrono::ChVectorDynamic<> fi(24);
        element->ComputeInternalForces(fi);
        Evaluation result;
        result.finite = fi.allFinite();
        for (int n = 0; n < 4; ++n) {
            result.force[n] = Vec(fi(6*n), fi(6*n+1), fi(6*n+2));
            const Vec local_couple(fi(6*n+3), fi(6*n+4), fi(6*n+5));
            result.couple[n] = frames.q[n].Rotate(local_couple);
        }
        // These currently-public diagnostics are implementation-sensitive
        // reference-test access, not a stable production shell output API.
        for (int p = 0; p < 4; ++p) {
            const std::array<Vec, 4> generalized{{element->eps_tilde_1_i[p], element->eps_tilde_2_i[p],
                                                element->k_tilde_1_i[p], element->k_tilde_2_i[p]}};
            const double area_weight = element->alpha_i[p] * Element::w_i[p];
            result.finite = result.finite && std::isfinite(area_weight) && area_weight > 0;
            result.area += area_weight;
            for (int c = 0; c < 12; ++c) {
                const double strain = generalized[c/3][c%3];
                const double stress = element->stress_i[p](c);
                result.strain[p][c] = strain;
                result.stress[p][c] = stress;
                result.finite = result.finite && std::isfinite(strain) && std::isfinite(stress);
                // Centered, linear elastic layer: work-conjugate section
                // energy is 1/2 strain dot resultant, integrated on reference
                // area. This energy diagnostic is checked independently by
                // analytical membrane/bending and virtual-work tests below.
                const double energy = .5 * area_weight * strain * stress;
                result.energy += energy;
                if (c >= 6)
                    result.bending_energy += energy;
            }
        }
        result.finite = result.finite && std::isfinite(result.energy) && std::isfinite(result.bending_energy);
        return result;
    }

    void ReferenceUnchanged() {
        for (int n = 0; n < 4; ++n) {
            EXPECT_EQ(nodes[n]->GetX0().GetPos(), initial.x[n]);
            EXPECT_EQ(nodes[n]->GetX0().GetRot(), initial.q[n]);
            EXPECT_EQ(element->xa_0[n], initial.x[n]);
        }
    }

    void StressFree(const Evaluation& value, const std::string& prefix) {
        ASSERT_TRUE(value.finite);
        Metric(prefix + "_max_force_N", MaxVector(value.force), kForceAbs);
        Metric(prefix + "_max_couple_Nm", MaxVector(value.couple), kMomentAbs);
        Metric(prefix + "_max_strain", MaxComponents(value.strain, 0, 6), kStrainAbs);
        Metric(prefix + "_max_curvature_per_m", MaxComponents(value.strain, 6, 12), kCurvatureAbs);
        Metric(prefix + "_energy_J", std::abs(value.energy), kEnergyAbs);
    }

    void Balance(const Evaluation& value, const Frames& frames, const std::string& prefix) {
        Vec force(0, 0, 0), moment(0, 0, 0), center(0, 0, 0);
        for (const auto& x : frames.x)
            center += .25 * x;
        for (int n = 0; n < 4; ++n) {
            force += value.force[n];
            moment += chrono::Vcross(frames.x[n] - center, value.force[n]) + value.couple[n];
        }
        Metric(prefix + "_net_force_N", force.Length(), kForceAbs + kRelative * 4 * MaxVector(value.force));
        Metric(prefix + "_net_moment_Nm", moment.Length(),
               kMomentAbs + kRelative * 4 * (kLength * MaxVector(value.force) + MaxVector(value.couple)));
        // Independent rigid virtual displacement: 0.3m translation and a
        // dimensionless 0.4rad infinitesimal spin, including nodal couples.
        const Vec translation(.2, -.1, .2), spin(.1, -.2, .3);
        double work = 0;
        for (int n = 0; n < 4; ++n)
            work += value.force[n].Dot(translation + chrono::Vcross(spin, frames.x[n] - center)) +
                    value.couple[n].Dot(spin);
        Metric(prefix + "_rigid_virtual_work_J", std::abs(work),
               kMomentAbs + kRelative * 4 * (kLength * MaxVector(value.force) + MaxVector(value.couple)));
    }

    void Covariant(const Evaluation& before, const Evaluation& after, const Quat& rotation,
                   const std::string& prefix) {
        ASSERT_TRUE(before.finite);
        ASSERT_TRUE(after.finite);
        double force_error = 0, couple_error = 0, strain_error = 0, curvature_error = 0;
        double membrane_stress_error = 0, bending_stress_error = 0;
        for (int n = 0; n < 4; ++n) {
            force_error = std::max(force_error, (after.force[n] - rotation.Rotate(before.force[n])).Length());
            couple_error = std::max(couple_error, (after.couple[n] - rotation.Rotate(before.couple[n])).Length());
            for (int c = 0; c < 12; ++c) {
                auto& error = c < 6 ? strain_error : curvature_error;
                error = std::max(error, std::abs(after.strain[n][c] - before.strain[n][c]));
                auto& stress_error = c < 6 ? membrane_stress_error : bending_stress_error;
                stress_error = std::max(stress_error, std::abs(after.stress[n][c] - before.stress[n][c]));
            }
        }
        Metric(prefix + "_force_covariance_N", force_error, kForceAbs + kRelative * MaxVector(before.force));
        Metric(prefix + "_couple_covariance_Nm", couple_error, kMomentAbs + kRelative * MaxVector(before.couple));
        Metric(prefix + "_strain_invariance", strain_error,
               kStrainAbs + kRelative * MaxComponents(before.strain, 0, 6));
        Metric(prefix + "_curvature_invariance_per_m", curvature_error,
               kCurvatureAbs + kRelative * MaxComponents(before.strain, 6, 12));
        Metric(prefix + "_membrane_resultant_invariance_N_per_m", membrane_stress_error,
               2 * kC * kStrainAbs + kRelative * MaxComponents(before.stress, 0, 6));
        Metric(prefix + "_bending_resultant_invariance_N", bending_stress_error,
               2 * kD * kCurvatureAbs + kRelative * MaxComponents(before.stress, 6, 12));
        Metric(prefix + "_energy_invariance_J", std::abs(after.energy - before.energy),
               kEnergyAbs + kRelative * std::abs(before.energy));
    }
};

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
}  // namespace
