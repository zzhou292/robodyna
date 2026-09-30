#include "ElasticCouponModel.h"
#include "ElasticCouponModal.h"

#include "chrono/core/ChQuaternion.h"
#include "math/Quaternion.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace crash::qualification {
namespace {
using namespace reference;
namespace tlr = tl::fea::reissner;
using Status = ElasticCouponStatus;
namespace audit = patch_audit;
using PatchMatrix = audit::Matrix<kCouponFreeDofs>;
using PatchSpectrum = audit::ReferenceSpectrum<kCouponFreeDofs>;

void ExpectSameDoubles(const double* actual, const double* expected, std::size_t count) {
    EXPECT_EQ(std::memcmp(actual, expected, count * sizeof(double)), 0);
}

void ExpectVectorBits(tlr::Vec3 a, tlr::Vec3 b) {
    const double x[]{a.x, a.y, a.z}, y[]{b.x, b.y, b.z};
    ExpectSameDoubles(x, y, 3);
}

void ExpectConfigurationBits(const ElasticCouponConfiguration& a, const ElasticCouponConfiguration& b) {
    for (std::size_t n = 0; n < kCouponNodes; ++n) {
        for (unsigned c = 0; c < 3; ++c) {
            const double x = tlr::detail::Component(a.position[n], c);
            const double y = tlr::detail::Component(b.position[n], c);
            ExpectSameDoubles(&x, &y, 1);
        }
        const double qa[]{a.rotation[n].w, a.rotation[n].x, a.rotation[n].y, a.rotation[n].z};
        const double qb[]{b.rotation[n].w, b.rotation[n].x, b.rotation[n].y, b.rotation[n].z};
        ExpectSameDoubles(qa, qb, 4);
    }
}

void ExpectSpectrumBits(const PatchSpectrum& a, const PatchSpectrum& b) {
    ExpectSameDoubles(a.stiffness.data(), b.stiffness.data(), kCouponFreeDofs * kCouponFreeDofs);
    ExpectSameDoubles(a.mass_modes.data(), b.mass_modes.data(), kCouponFreeDofs * kCouponFreeDofs);
    ExpectSameDoubles(a.inverse_root_mass.data(), b.inverse_root_mass.data(), kCouponFreeDofs);
    ExpectSameDoubles(a.squared_frequency.data(), b.squared_frequency.data(), kCouponFreeDofs);
    const double da[]{a.symmetry_error, a.derivative_refinement_error, a.tl_derivative_error,
                      a.maximum_frequency_refinement_error, a.eigen_residual};
    const double db[]{b.symmetry_error, b.derivative_refinement_error, b.tl_derivative_error,
                      b.maximum_frequency_refinement_error, b.eigen_residual};
    ExpectSameDoubles(da, db, 5);
}

double Length(tlr::Vec3 vector) { return std::sqrt(tlr::detail::Dot(vector, vector)); }

std::string Precise(double value) {
    std::ostringstream text;
    text.precision(17);
    text << value;
    return text.str();
}

void ExpectVector(tlr::Vec3 actual, tlr::Vec3 expected, double absolute = 1e-9, double relative = 2e-8) {
    EXPECT_LE(Length(tlr::detail::Subtract(actual, expected)), absolute + relative * Length(expected));
}

void ExpectEvaluation(const ElasticCouponEvaluation& actual, const ElasticCouponEvaluation& expected,
                      double absolute = 1e-9, double relative = 2e-8) {
    for (std::size_t n = 0; n < kCouponNodes; ++n) {
        ExpectVector(actual.force[n], expected.force[n], absolute, relative);
        ExpectVector(actual.couple[n], expected.couple[n], absolute, relative);
    }
    EXPECT_NEAR(actual.energy, expected.energy, absolute + relative * std::abs(expected.energy));
    EXPECT_NEAR(actual.bending_energy, expected.bending_energy, absolute + relative * std::abs(expected.bending_energy));
    for (std::size_t e = 0; e < kCouponElements; ++e) {
        EXPECT_NEAR(actual.element[e].energy, expected.element[e].energy,
                    absolute + relative * std::abs(expected.element[e].energy));
        EXPECT_NEAR(actual.element[e].bending_energy, expected.element[e].bending_energy,
                    absolute + relative * std::abs(expected.element[e].bending_energy));
        for (std::size_t n = 0; n < 4; ++n) {
            ExpectVector(actual.element[e].force[n], expected.element[e].force[n], absolute, relative);
            ExpectVector(actual.element[e].couple[n], expected.element[e].couple[n], absolute, relative);
            for (std::size_t c = 0; c < 12; ++c) {
                EXPECT_NEAR(actual.element[e].strain[n][c], expected.element[e].strain[n][c],
                            absolute + relative * std::abs(expected.element[e].strain[n][c]));
                EXPECT_NEAR(actual.element[e].resultant[n][c], expected.element[e].resultant[n][c],
                            absolute + relative * std::abs(expected.element[e].resultant[n][c]));
            }
        }
    }
}

ElasticCouponConfiguration MixedDeformation(const ElasticCouponModel& model) {
    std::array<double, kCouponFreeDofs> increment{};
    for (std::size_t free = 0; free < kCouponFreeNodes.size(); ++free) {
        const double fraction = model.data().reference_configuration.position[kCouponFreeNodes[free]].x /
                                ElasticCouponData::length;
        increment[6 * free] = 1e-5 * fraction;
        increment[6 * free + 2] = .002 * fraction * fraction;
        increment[6 * free + 3] = .001 * fraction;
        increment[6 * free + 4] = -.02 * fraction;
        increment[6 * free + 5] = .0003 * fraction;
    }
    ElasticCouponConfiguration result;
    std::string diagnostic;
    EXPECT_EQ(ApplyElasticCouponIncrement(model.data().reference_configuration, increment, 1, result, diagnostic),
              Status::kSuccess) << diagnostic;
    return result;
}

TEST(ElasticCouponReference, PhysicalMassSharedNodesAndConstraints) {
    ElasticCouponModel model;
    const auto& data = model.data();
    const std::array<std::array<std::size_t, 4>, 2> connectivity{{{0, 1, 2, 3}, {4, 0, 3, 5}}};
    EXPECT_EQ(data.connectivity, connectivity);
    double total_mass = 0, total_inertia = 0;
    for (std::size_t n = 0; n < kCouponNodes; ++n) {
        const double mass = (n == 0 || n == 3) ? .1 : .05;
        EXPECT_NEAR(data.nodal_mass[n].mass, mass, 1e-15);
        EXPECT_NEAR(data.nodal_mass[n].physical_tangential_inertia, mass * .02 * .02 / 12, 1e-18);
        EXPECT_EQ(data.nodal_mass[n].physical_tangential_inertia, data.nodal_mass[n].artificial_drilling_inertia);
        EXPECT_EQ(data.fixed[n], n == 1 || n == 2);
        if (data.fixed[n]) {
            EXPECT_EQ(data.inverse_mass[n], 0);
            EXPECT_EQ(data.inverse_isotropic_inertia[n], 0);
        } else {
            EXPECT_NEAR(data.inverse_mass[n] * mass, 1, 1e-14);
            EXPECT_NEAR(data.inverse_isotropic_inertia[n] * mass * .02 * .02 / 12, 1, 1e-14);
        }
        total_mass += data.nodal_mass[n].mass;
        total_inertia += data.nodal_mass[n].physical_tangential_inertia;
    }
    EXPECT_NEAR(total_mass, 1000 * .2 * .1 * .02, 1e-15);
    EXPECT_NEAR(total_inertia, total_mass * .02 * .02 / 12, 1e-18);
    for (std::size_t e = 0; e < kCouponElements; ++e) {
        EXPECT_TRUE(data.reference[e].prepared);
        EXPECT_TRUE(data.section[e].prepared);
        EXPECT_EQ(data.element_mass[e].drilling_policy, tlr::ShellDrillingInertiaPolicy::kEqualPhysicalTangential);
        for (std::size_t local = 0; local < 4; ++local)
            ExpectVector(data.reference[e].initial_position[local],
                         data.reference_configuration.position[data.connectivity[e][local]], 0, 0);
    }
}

TEST(ElasticCouponReference, NeutralStateIsStressFreeInBothOperations) {
    ElasticCouponModel model;
    ElasticCouponEvaluation chrono, tl;
    std::string diagnostic;
    ASSERT_EQ(model.EvaluateChrono(model.data().reference_configuration, chrono, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(model.EvaluateTL(model.data().reference_configuration, tl, diagnostic), Status::kSuccess) << diagnostic;
    ExpectEvaluation(chrono, tl);
    EXPECT_NEAR(chrono.energy, 0, 1e-20);
    for (std::size_t n = 0; n < kCouponNodes; ++n) {
        ExpectVector(chrono.force[n], {}, 1e-9, 0);
        ExpectVector(chrono.couple[n], {}, 1e-10, 0);
    }
}

TEST(ElasticCouponReference, MixedDeformationWorldCouplesAndSharedAssembly) {
    ElasticCouponModel model;
    const auto configuration = MixedDeformation(model);
    ElasticCouponEvaluation chrono, tl;
    std::string diagnostic;
    ASSERT_EQ(model.EvaluateChrono(configuration, chrono, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(model.EvaluateTL(configuration, tl, diagnostic), Status::kSuccess) << diagnostic;
    ExpectEvaluation(chrono, tl);
    EXPECT_GT(tl.energy, 0);
    EXPECT_GT(tl.bending_energy, 0);
    ExpectVector(tl.force[0], tlr::detail::Add(tl.element[0].force[0], tl.element[1].force[1]), 0, 0);
    ExpectVector(tl.couple[3], tlr::detail::Add(tl.element[0].couple[3], tl.element[1].couple[2]), 0, 0);
    tlr::Vec3 net_force, net_moment;
    for (std::size_t n = 0; n < kCouponNodes; ++n) {
        net_force = tlr::detail::Add(net_force, tl.force[n]);
        net_moment = tlr::detail::Add(net_moment,
            tlr::detail::Add(tlr::detail::Cross(configuration.position[n], tl.force[n]), tl.couple[n]));
    }
    ExpectVector(net_force, {}, 1e-9, 0);
    ExpectVector(net_moment, {}, 1e-9, 0);
}

TEST(ElasticCouponReference, InvalidAndLateFailedEvaluationsPreserveOutputsAndReference) {
    ElasticCouponModel model, clean;
    const auto configuration = MixedDeformation(model);
    ElasticCouponEvaluation accepted;
    std::string diagnostic;
    ASSERT_EQ(model.EvaluateChrono(configuration, accepted, diagnostic), Status::kSuccess) << diagnostic;
    for (const bool use_chrono : {false, true}) {
        auto output = accepted;
        auto invalid = configuration;
        invalid.position[5].z = std::numeric_limits<double>::quiet_NaN();
        auto status = use_chrono ? model.EvaluateChrono(invalid, output, diagnostic)
                                 : model.EvaluateTL(invalid, output, diagnostic);
        EXPECT_EQ(status, Status::kInvalidConfiguration);
        EXPECT_FALSE(diagnostic.empty());
        ExpectEvaluation(output, accepted, 0, 0);
        invalid = configuration;
        invalid.rotation[5].w = .9;
        status = use_chrono ? model.EvaluateChrono(invalid, output, diagnostic)
                            : model.EvaluateTL(invalid, output, diagnostic);
        EXPECT_EQ(status, Status::kInvalidConfiguration);
        ExpectEvaluation(output, accepted, 0, 0);
        // Only the SECOND Q4 sees node 5: the first response has already been
        // produced before this otherwise-unit quaternion violates its chart.
        invalid = configuration;
        invalid.rotation[5] = {0, 1, 0, 0};
        status = use_chrono ? model.EvaluateChrono(invalid, output, diagnostic)
                            : model.EvaluateTL(invalid, output, diagnostic);
        EXPECT_EQ(status, Status::kForceFailure);
        ExpectEvaluation(output, accepted, 0, 0);
    }
    ElasticCouponEvaluation retry, baseline, neutral;
    ASSERT_EQ(model.EvaluateChrono(configuration, retry, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(clean.EvaluateChrono(configuration, baseline, diagnostic), Status::kSuccess) << diagnostic;
    ExpectEvaluation(retry, baseline, 0, 0);
    ASSERT_EQ(model.EvaluateChrono(model.data().reference_configuration, neutral, diagnostic), Status::kSuccess) << diagnostic;
    EXPECT_NEAR(neutral.energy, 0, 1e-20);
    EXPECT_GT(retry.energy, 0);  // No Setup or neutral recapture erased strain.
}

TEST(ElasticCouponReference, RigidMotionPreservesEnergyAndRotatesWorldForces) {
    ElasticCouponModel model;
    const auto original = MixedDeformation(model);
    auto transformed = original;
    chrono::ChQuaterniond common;
    common.SetFromRotVec(chrono::ChVector3d(.31, -.27, .19));
    const tlr::Quaternion rotation{common.e0(), common.e1(), common.e2(), common.e3()};
    for (std::size_t n = 0; n < kCouponNodes; ++n) {
        const auto x = original.position[n];
        const auto moved = common.Rotate(chrono::ChVector3d(x.x, x.y, x.z)) + chrono::ChVector3d(.2, -.3, .1);
        transformed.position[n] = {moved.x(), moved.y(), moved.z()};
        transformed.rotation[n] = tl::math::Product(rotation, original.rotation[n]);
    }
    ElasticCouponEvaluation before, after, tl;
    std::string diagnostic;
    ASSERT_EQ(model.EvaluateChrono(original, before, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(model.EvaluateChrono(transformed, after, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(model.EvaluateTL(transformed, tl, diagnostic), Status::kSuccess) << diagnostic;
    ExpectEvaluation(after, tl);
    EXPECT_NEAR(after.energy, before.energy, 1e-11 + 2e-8 * before.energy);
    for (std::size_t n = 0; n < kCouponNodes; ++n) {
        const auto f = before.force[n], c = before.couple[n];
        const auto force = common.Rotate(chrono::ChVector3d(f.x, f.y, f.z));
        const auto couple = common.Rotate(chrono::ChVector3d(c.x, c.y, c.z));
        ExpectVector(after.force[n], {force.x(), force.y(), force.z()});
        ExpectVector(after.couple[n], {couple.x(), couple.y(), couple.z()});
    }
}

TEST(ElasticCouponReference, FreeIncrementUsesWorldCompositionAndStagesLateFailure) {
    ElasticCouponModel model;
    auto base = model.data().reference_configuration;
    chrono::ChQuaterniond old, delta;
    old.SetFromRotVec(chrono::ChVector3d(.2, .1, -.3));
    delta.SetFromRotVec(chrono::ChVector3d(-.02, .03, .01));
    base.rotation[0] = {old.e0(), old.e1(), old.e2(), old.e3()};
    std::array<double, kCouponFreeDofs> increment{};
    increment[0] = .001;
    increment[3] = -.02; increment[4] = .03; increment[5] = .01;
    auto output = base;
    std::string diagnostic;
    ASSERT_EQ(ApplyElasticCouponIncrement(base, increment, 1, output, diagnostic), Status::kSuccess) << diagnostic;
    const auto expected = delta * old;
    EXPECT_NEAR(output.rotation[0].w, expected.e0(), 1e-15);
    EXPECT_NEAR(output.rotation[0].x, expected.e1(), 1e-15);
    EXPECT_NEAR(output.rotation[0].y, expected.e2(), 1e-15);
    EXPECT_NEAR(output.rotation[0].z, expected.e3(), 1e-15);
    EXPECT_DOUBLE_EQ(output.position[0].x, base.position[0].x + .001);
    for (const auto n : {1, 2}) {
        ExpectVector(output.position[n], base.position[n], 0, 0);
        EXPECT_EQ(output.rotation[n].w, base.rotation[n].w);
    }
    const auto saved = output;
    increment.back() = std::numeric_limits<double>::infinity();
    EXPECT_EQ(ApplyElasticCouponIncrement(base, increment, 1, output, diagnostic), Status::kInvalidConfiguration);
    for (std::size_t n = 0; n < kCouponNodes; ++n) {
        ExpectVector(output.position[n], saved.position[n], 0, 0);
        EXPECT_EQ(output.rotation[n].w, saved.rotation[n].w);
        EXPECT_EQ(output.rotation[n].x, saved.rotation[n].x);
        EXPECT_EQ(output.rotation[n].y, saved.rotation[n].y);
        EXPECT_EQ(output.rotation[n].z, saved.rotation[n].z);
    }
}

TEST(ElasticCouponReference, AllDofModalAuditPublishesBoundedBendingExperiment) {
    ElasticCouponModel model;
    ElasticCouponModalReport report;
    std::string diagnostic;
    ASSERT_EQ(AuditElasticCoupon(model, report, diagnostic), Status::kSuccess) << diagnostic;
    EXPECT_TRUE(diagnostic.empty());
    EXPECT_EQ(report.selected_mode, 0);
    EXPECT_GT(report.squared_frequency.front(), 0);
    EXPECT_TRUE(std::is_sorted(report.squared_frequency.begin(), report.squared_frequency.end()));
    EXPECT_GE(report.out_of_plane_mass_fraction, .9);
    EXPECT_GE(report.transverse_translation_mass_fraction, .5);
    EXPECT_LE(report.reference_symmetry_error, kCouponDerivativeTolerance);
    EXPECT_LE(report.mass_scaled_derivative_refinement_error, kCouponDerivativeTolerance);
    EXPECT_LE(report.tl_derivative_relative_error, kCouponDerivativeTolerance);
    EXPECT_LE(report.tl_directional_relative_error, kCouponDerivativeTolerance);
    EXPECT_LE(report.maximum_frequency_refinement_error, kCouponFrequencyRefinementTolerance);
    EXPECT_LE(report.eigen_residual, 1e-10);
    EXPECT_NEAR(.5 * (report.initial_configuration.position[4].z + report.initial_configuration.position[5].z), .002, 1e-15);
    EXPECT_NEAR(report.initial_configuration.position[4].z, report.initial_configuration.position[5].z, 1e-10);
    for (const auto n : {1, 2}) {
        ExpectVector(report.initial_configuration.position[n], model.data().reference_configuration.position[n], 0, 0);
        EXPECT_EQ(report.initial_configuration.rotation[n].w, 1);
    }
    EXPECT_GT(report.initial_energy, 0);
    EXPECT_GT(report.step_count, 100);
    EXPECT_LE(report.step_count, 1000000);
    EXPECT_GT(report.time_step, 0);
    EXPECT_LE(report.time_step, report.proposed_step_limit);
    EXPECT_LE(report.time_step, .1 / std::sqrt(report.monitored_norm_limit));
    EXPECT_LE(report.time_step, report.wave_step_limit);
    EXPECT_LE(report.time_step, report.rotary_step_limit);
    EXPECT_EQ(report.horizon, report.step_count * report.time_step);
    EXPECT_NEAR(report.horizon * report.first_mode_angular_frequency, std::acos(-1.0), 1e-14);
    EXPECT_EQ(report.monitored_norm_limit, 2 * report.sampled_norm_maximum);
    EXPECT_NEAR(report.shortest_reference_edge, .1, 1e-15);
    EXPECT_NEAR(report.membrane_wave_speed, std::sqrt(1.2e6 / (1000 * (1 - .3 * .3))), 1e-12);
    // Independent Euler-Bernoulli scale, used only as a broad dimensional
    // sanity check for this coarse thick-shell discretization, not accuracy.
    const double beam_scale = 1.875104068711961 * 1.875104068711961 *
                              std::sqrt(ElasticCouponData::young_modulus / ElasticCouponData::density * .02 * .02 / 12) /
                              (.2 * .2);
    EXPECT_GT(report.first_mode_angular_frequency, .5 * beam_scale);
    EXPECT_LT(report.first_mode_angular_frequency, 2 * beam_scale);
    double neutral_norm = 0, initial_norm = 0;
    ASSERT_EQ(MeasureElasticCouponOperatorNorm(model, model.data().reference_configuration, neutral_norm, diagnostic),
              Status::kSuccess) << diagnostic;
    ASSERT_EQ(MeasureElasticCouponOperatorNorm(model, report.initial_configuration, initial_norm, diagnostic),
              Status::kSuccess) << diagnostic;
    EXPECT_NEAR(neutral_norm, report.squared_frequency.back(), 1e-5 * neutral_norm);
    EXPECT_EQ(initial_norm, report.sampled_operator_norm[2]);
    EXPECT_LE(initial_norm, report.monitored_norm_limit);
    RecordProperty("first_mode_angular_frequency", Precise(report.first_mode_angular_frequency));
    RecordProperty("maximum_squared_frequency", Precise(report.squared_frequency.back()));
    RecordProperty("monitored_norm_limit", Precise(report.monitored_norm_limit));
    RecordProperty("reference_symmetry_error", Precise(report.reference_symmetry_error));
    RecordProperty("derivative_refinement_error", Precise(report.mass_scaled_derivative_refinement_error));
    RecordProperty("tl_derivative_error", Precise(report.tl_derivative_relative_error));
    RecordProperty("tl_directional_error", Precise(report.tl_directional_relative_error));
    RecordProperty("time_step", Precise(report.time_step));
    RecordProperty("step_count", std::to_string(report.step_count));
    RecordProperty("horizon", Precise(report.horizon));
    RecordProperty("initial_energy", Precise(report.initial_energy));
}

TEST(ElasticCouponReference, InvalidOperatorNormReadbackPreservesOutput) {
    ElasticCouponModel model;
    auto invalid = model.data().reference_configuration;
    invalid.rotation[5] = {0, 0, 0, 0};
    double norm = 123;
    std::string diagnostic;
    EXPECT_EQ(MeasureElasticCouponOperatorNorm(model, invalid, norm, diagnostic), Status::kInvalidConfiguration);
    EXPECT_EQ(norm, 123);
    EXPECT_FALSE(diagnostic.empty());
}

TEST(ElasticCouponReference, ExplicitDefaultParametersPreserveSetupAndResponseBits) {
    for (const auto pose : {ElasticCouponPose{}, ElasticCouponPose{{.5, .5, .5, .5}, {.01, -.02, .03}}}) {
        ElasticCouponModel legacy(pose);
        auto parameters = kElasticCouponDefaultParameters;
        ElasticCouponModel explicit_default(parameters, pose);
        parameters.length *= 2;  // The reference owns an immutable copy.
        EXPECT_TRUE(IsDefaultElasticCouponParameters(explicit_default.parameters()));
        const auto& a = legacy.data();
        const auto& b = explicit_default.data();
        ExpectConfigurationBits(a.reference_configuration, b.reference_configuration);
        EXPECT_EQ(a.connectivity, b.connectivity);
        EXPECT_EQ(a.fixed, b.fixed);
        ExpectSameDoubles(a.inverse_mass.data(), b.inverse_mass.data(), kCouponNodes);
        ExpectSameDoubles(a.inverse_isotropic_inertia.data(), b.inverse_isotropic_inertia.data(), kCouponNodes);
        for (std::size_t n = 0; n < kCouponNodes; ++n) {
            const double ma[]{a.nodal_mass[n].mass, a.nodal_mass[n].physical_tangential_inertia,
                              a.nodal_mass[n].artificial_drilling_inertia};
            const double mb[]{b.nodal_mass[n].mass, b.nodal_mass[n].physical_tangential_inertia,
                              b.nodal_mass[n].artificial_drilling_inertia};
            ExpectSameDoubles(ma, mb, 3);
        }
        for (std::size_t e = 0; e < kCouponElements; ++e) {
            ExpectSameDoubles(a.section[e].stiffness, b.section[e].stiffness, 144);
            ExpectSameDoubles(&a.section[e].thickness, &b.section[e].thickness, 1);
            ExpectSameDoubles(&a.section[e].density, &b.section[e].density, 1);
        }
        // Identical prescribed world increments exercise the copied setup,
        // including the original exact cyclic-pose path used by guided D.
        std::array<double, kCouponFreeDofs> increment{};
        increment[2] = .0001; increment[10] = -.002; increment[17] = .001;
        ElasticCouponConfiguration configuration;
        std::string diagnostic;
        ASSERT_EQ(ApplyElasticCouponIncrement(a.reference_configuration, increment, 1, configuration, diagnostic),
                  Status::kSuccess) << diagnostic;
        for (bool chrono : {false, true}) {
            ElasticCouponEvaluation first, second;
            ASSERT_EQ(chrono ? legacy.EvaluateChrono(configuration, first, diagnostic)
                             : legacy.EvaluateTL(configuration, first, diagnostic), Status::kSuccess) << diagnostic;
            ASSERT_EQ(chrono ? explicit_default.EvaluateChrono(configuration, second, diagnostic)
                             : explicit_default.EvaluateTL(configuration, second, diagnostic), Status::kSuccess) << diagnostic;
            ExpectEvaluation(first, second, 0, 0);
            // ShellResult contains only numeric arrays; compare each owning
            // array, avoiding bool/padding bytes in the reference structs.
            for (std::size_t n = 0; n < kCouponNodes; ++n) {
                ExpectVectorBits(first.force[n], second.force[n]);
                ExpectVectorBits(first.couple[n], second.couple[n]);
            }
            for (std::size_t e = 0; e < kCouponElements; ++e) {
                for (std::size_t n = 0; n < 4; ++n) {
                    ExpectVectorBits(first.element[e].force[n], second.element[e].force[n]);
                    ExpectVectorBits(first.element[e].couple[n], second.element[e].couple[n]);
                }
                ExpectSameDoubles(&first.element[e].strain[0][0], &second.element[e].strain[0][0], 48);
                ExpectSameDoubles(&first.element[e].resultant[0][0], &second.element[e].resultant[0][0], 48);
                ExpectSameDoubles(&first.element[e].energy, &second.element[e].energy, 1);
                ExpectSameDoubles(&first.element[e].bending_energy, &second.element[e].bending_energy, 1);
            }
            ExpectSameDoubles(&first.energy, &second.energy, 1);
            ExpectSameDoubles(&first.bending_energy, &second.bending_energy, 1);
        }
    }
}

TEST(ElasticCouponReference, ParameterizedSetupHasActualGeometrySectionMassAndNoDynamicsAdmission) {
    ElasticCouponParameters p;
    p.length = .04; p.width = .02; p.thickness = .001648;
    p.young_modulus = 200e9; p.poisson_ratio = .28; p.density = 7890;
    p.shear_factor = .75; p.torque_factor = .02;
    ElasticCouponModel model(p);
    const auto& data = model.data();
    const std::array<tlr::Vec3, kCouponNodes> expected{{{.02, .01, 0}, {0, .01, 0}, {0, -.01, 0},
                                                     {.02, -.01, 0}, {.04, .01, 0}, {.04, -.01, 0}}};
    const double total = p.density * p.thickness * p.length * p.width;
    double assembled = 0;
    for (std::size_t n = 0; n < kCouponNodes; ++n) {
        ExpectVector(data.reference_configuration.position[n], expected[n], 0, 0);
        const double mass = total * ((n == 0 || n == 3) ? .25 : .125);
        EXPECT_NEAR(data.nodal_mass[n].mass, mass, 2e-14 * mass);
        EXPECT_NEAR(data.nodal_mass[n].physical_tangential_inertia, mass * p.thickness * p.thickness / 12,
                    2e-14 * mass * p.thickness * p.thickness);
        EXPECT_EQ(data.nodal_mass[n].physical_tangential_inertia, data.nodal_mass[n].artificial_drilling_inertia);
        assembled += data.nodal_mass[n].mass;
    }
    EXPECT_NEAR(assembled, total, 2e-14 * total);
    // Independent centered isotropic section scalings (force/length and
    // force*length), not a second call through the setup adapter.
    const double g = p.young_modulus / (2 * (1 + p.poisson_ratio));
    for (const auto& section : data.section) {
        EXPECT_EQ(section.thickness, p.thickness);
        EXPECT_EQ(section.density, p.density);
        EXPECT_NEAR(section.stiffness[0], p.young_modulus * p.thickness / (1 - p.poisson_ratio * p.poisson_ratio),
                    2e-14 * p.young_modulus * p.thickness);
        EXPECT_NEAR(section.stiffness[2 * 12 + 2], p.shear_factor * g * p.thickness, 2e-14 * g * p.thickness);
        EXPECT_NEAR(section.stiffness[8 * 12 + 8], p.torque_factor * g * std::pow(p.thickness, 3) / 12,
                    2e-14 * g * std::pow(p.thickness, 3));
    }
    std::array<double, kCouponFreeDofs> increment{};
    increment[2] = 2e-7; increment[10] = -2e-5; increment[17] = 1e-5;
    ElasticCouponConfiguration configuration;
    ElasticCouponEvaluation chrono, tl;
    std::string diagnostic;
    ASSERT_EQ(ApplyElasticCouponIncrement(data.reference_configuration, increment, 1, configuration, diagnostic),
              Status::kSuccess) << diagnostic;
    ASSERT_EQ(model.EvaluateChrono(configuration, chrono, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(model.EvaluateTL(configuration, tl, diagnostic), Status::kSuccess) << diagnostic;
    ExpectEvaluation(chrono, tl, 1e-7, 2e-8);
    EXPECT_GT(tl.energy, 0);
    // Prescribed derivative qualification is independent of the B2 dynamics
    // admission. Its metre step is explicit for this smaller reference.
    const auto layout = audit::FullCouponLayout();
    const audit::PatchDifferenceSteps steps{2e-8, 1e-6};
    PatchMatrix chrono_jacobian, tl_jacobian;
    ASSERT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, data.reference_configuration, layout,
              audit::ForceSource::Chrono, steps, .5, chrono_jacobian, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, data.reference_configuration, layout,
              audit::ForceSource::TL, steps, .5, tl_jacobian, diagnostic), Status::kSuccess) << diagnostic;
    for (std::size_t c = 0; c < kCouponFreeDofs; ++c) {
        const double scale = chrono_jacobian.col(c).norm();
        ASSERT_GT(scale, 0);
        EXPECT_LE((chrono_jacobian.col(c) - tl_jacobian.col(c)).norm() / scale, audit::DerivativeTolerance);
    }
    ElasticCouponModalReport output;
    output.initial_energy = 123; output.step_count = 456;
    std::array<unsigned char, sizeof(output)> before{};
    std::memcpy(before.data(), &output, sizeof(output));
    EXPECT_EQ(AuditElasticCoupon(model, output, diagnostic), Status::kInvalidConfiguration);
    EXPECT_FALSE(diagnostic.empty());
    EXPECT_EQ(std::memcmp(before.data(), &output, sizeof(output)), 0);
}

TEST(ElasticCouponReference, InvalidExplicitParametersFailBeforeSetup) {
    const std::array<double ElasticCouponParameters::*, 7> positive{{&ElasticCouponParameters::length,
        &ElasticCouponParameters::width, &ElasticCouponParameters::thickness, &ElasticCouponParameters::young_modulus,
        &ElasticCouponParameters::density, &ElasticCouponParameters::shear_factor, &ElasticCouponParameters::torque_factor}};
    for (auto field : positive) for (double invalid : {0., -1., std::numeric_limits<double>::infinity(),
                                                      std::numeric_limits<double>::quiet_NaN()}) {
        auto p = kElasticCouponDefaultParameters;
        p.*field = invalid;
        EXPECT_THROW(ElasticCouponModel rejected(p), std::invalid_argument);
    }
    for (double invalid : {-1., .5, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        auto p = kElasticCouponDefaultParameters;
        p.poisson_ratio = invalid;
        EXPECT_THROW(ElasticCouponModel rejected(p), std::invalid_argument);
    }
    auto p = kElasticCouponDefaultParameters;
    p.length = std::numeric_limits<double>::denorm_min();
    EXPECT_THROW(ElasticCouponModel rejected(p), std::invalid_argument);
    const ElasticCouponPose invalid_pose{{0, 0, 0, 0}, {}};
    EXPECT_THROW(ElasticCouponModel rejected(kElasticCouponDefaultParameters, invalid_pose), std::invalid_argument);
    p = kElasticCouponDefaultParameters;
    p.density = 1e-300;  // Positive representable J, but 1/J overflows.
    EXPECT_THROW(ElasticCouponModel rejected(p), std::runtime_error);
    ElasticCouponModel valid;
    ElasticCouponEvaluation response;
    std::string diagnostic;
    EXPECT_EQ(valid.EvaluateTL(valid.data().reference_configuration, response, diagnostic), Status::kSuccess) << diagnostic;
}

TEST(ElasticCouponReference, ExplicitDefaultMassAndDifferenceOverloadsPreserveAllAuditBits) {
    ElasticCouponModel model;
    const auto layout = audit::FullCouponLayout();
    const auto mass = audit::DefaultPatchNodalMass(model.data());
    const audit::PatchDifferenceSteps steps;
    const auto legacy_mass = audit::InverseRootMass<kCouponFreeDofs>(model.data(), layout);
    const auto explicit_mass = audit::InverseRootMass<kCouponFreeDofs>(mass, layout);
    ExpectSameDoubles(legacy_mass.data(), explicit_mass.data(), kCouponFreeDofs);
    PatchMatrix legacy, explicit_steps;
    std::string diagnostic;
    for (auto source : {audit::ForceSource::TL, audit::ForceSource::Chrono}) {
        ASSERT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
                  source, .5, legacy, diagnostic), Status::kSuccess) << diagnostic;
        ASSERT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
                  source, steps, .5, explicit_steps, diagnostic), Status::kSuccess) << diagnostic;
        ExpectSameDoubles(legacy.data(), explicit_steps.data(), kCouponFreeDofs * kCouponFreeDofs);
    }
    double legacy_worst = 0, explicit_worst = 0;
    ASSERT_EQ(audit::DirectionalCrossCheck<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
              legacy, legacy_mass, legacy_worst, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(audit::DirectionalCrossCheck<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
              legacy, explicit_mass, steps, explicit_worst, diagnostic), Status::kSuccess) << diagnostic;
    ExpectSameDoubles(&legacy_worst, &explicit_worst, 1);
    PatchSpectrum first, second;
    ASSERT_EQ(audit::AuditReference<kCouponFreeDofs>(model, layout, first, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(audit::AuditReference<kCouponFreeDofs>(model, layout, mass, steps, second, diagnostic), Status::kSuccess) << diagnostic;
    ExpectSpectrumBits(first, second);
}

TEST(ElasticCouponReference, ExplicitTotalInertiaChangesOnlyTheMassMetric) {
    ElasticCouponModel model;
    const auto layout = audit::FullCouponLayout();
    const auto original = audit::DefaultPatchNodalMass(model.data());
    auto changed = original;
    for (auto& inertia : changed.total_isotropic_inertia) inertia *= 4;
    const auto before = audit::InverseRootMass<kCouponFreeDofs>(original, layout);
    const auto after = audit::InverseRootMass<kCouponFreeDofs>(changed, layout);
    for (std::size_t c = 0; c < kCouponFreeDofs; ++c)
        EXPECT_EQ(after(c), layout[c].component < 3 ? before(c) : .5 * before(c));
    PatchMatrix stiffness;
    std::string diagnostic;
    ASSERT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
              audit::ForceSource::Chrono, .5, stiffness, diagnostic), Status::kSuccess) << diagnostic;
    const auto base = audit::MassScale<kCouponFreeDofs>(stiffness, before);
    const auto scaled = audit::MassScale<kCouponFreeDofs>(stiffness, after);
    for (std::size_t row = 0; row < kCouponFreeDofs; ++row) for (std::size_t column = 0; column < kCouponFreeDofs; ++column) {
        const double factor = (layout[row].component < 3 ? 1 : .5) * (layout[column].component < 3 ? 1 : .5);
        EXPECT_EQ(scaled(row, column), factor * base(row, column));
    }
    const auto preserved = audit::DefaultPatchNodalMass(model.data());
    EXPECT_EQ(preserved.mass, original.mass);
    EXPECT_EQ(preserved.total_isotropic_inertia, original.total_isotropic_inertia);
}

TEST(ElasticCouponReference, InvalidExplicitAuditInputsPreserveCompleteOutputsAndAllowRetry) {
    ElasticCouponModel model;
    const auto layout = audit::FullCouponLayout();
    const auto mass = audit::DefaultPatchNodalMass(model.data());
    const auto root_mass = audit::InverseRootMass<kCouponFreeDofs>(mass, layout);
    PatchMatrix seeded = PatchMatrix::Constant(123), output = seeded;
    std::string diagnostic;
    const double tiny = std::numeric_limits<double>::denorm_min();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const auto steps : {audit::PatchDifferenceSteps{0, 1e-6}, audit::PatchDifferenceSteps{2e-7, nan},
                            audit::PatchDifferenceSteps{tiny, 1e-6}, audit::PatchDifferenceSteps{2e-7, tiny},
                            audit::PatchDifferenceSteps{std::numeric_limits<double>::infinity(), 1e-6}}) {
        EXPECT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
                  audit::ForceSource::Chrono, steps, .5, output, diagnostic), Status::kInvalidConfiguration);
        ExpectSameDoubles(output.data(), seeded.data(), kCouponFreeDofs * kCouponFreeDofs);
        EXPECT_FALSE(diagnostic.empty());
    }
    EXPECT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
              audit::ForceSource::Chrono, {std::numeric_limits<double>::max(), 1e-6}, 2, output, diagnostic),
              Status::kInvalidConfiguration);
    ExpectSameDoubles(output.data(), seeded.data(), kCouponFreeDofs * kCouponFreeDofs);
    PatchSpectrum spectrum;
    spectrum.stiffness.setConstant(11); spectrum.mass_modes.setConstant(12);
    spectrum.inverse_root_mass.setConstant(13); spectrum.squared_frequency.setConstant(14);
    spectrum.symmetry_error = 15; spectrum.derivative_refinement_error = 16; spectrum.tl_derivative_error = 17;
    spectrum.maximum_frequency_refinement_error = 18; spectrum.eigen_residual = 19;
    const auto preserved = spectrum;
    for (double invalid : {0., -1., nan, std::numeric_limits<double>::infinity()}) {
        auto bad_mass = mass;
        bad_mass.total_isotropic_inertia[5] = invalid;
        const auto roots = audit::InverseRootMass<kCouponFreeDofs>(bad_mass, layout);
        EXPECT_TRUE(roots.array().isNaN().all());
        EXPECT_EQ(audit::AuditReference<kCouponFreeDofs>(model, layout, bad_mass, {}, spectrum, diagnostic),
                  Status::kInvalidConfiguration);
        ExpectSpectrumBits(spectrum, preserved);
    }
    for (const auto last : {layout[0], audit::Coordinate{kCouponNodes, 0}, audit::Coordinate{5, 6}}) {
        auto bad_layout = layout;
        bad_layout.back() = last;
        EXPECT_TRUE(audit::InverseRootMass<kCouponFreeDofs>(mass, bad_layout).array().isNaN().all());
        EXPECT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, model.data().reference_configuration, bad_layout,
                  audit::ForceSource::TL, {}, .5, output, diagnostic), Status::kInvalidConfiguration);
        ExpectSameDoubles(output.data(), seeded.data(), kCouponFreeDofs * kCouponFreeDofs);
    }
    double worst = 7;
    EXPECT_EQ(audit::DirectionalCrossCheck<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
              seeded, root_mass, {0, 1e-6}, worst, diagnostic), Status::kInvalidConfiguration);
    EXPECT_EQ(worst, 7);
    PatchMatrix clean;
    ASSERT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
              audit::ForceSource::TL, {}, .5, output, diagnostic), Status::kSuccess) << diagnostic;
    ASSERT_EQ(audit::DifferenceJacobian<kCouponFreeDofs>(model, model.data().reference_configuration, layout,
              audit::ForceSource::TL, .5, clean, diagnostic), Status::kSuccess) << diagnostic;
    ExpectSameDoubles(output.data(), clean.data(), kCouponFreeDofs * kCouponFreeDofs);
}

}  // namespace
}  // namespace crash::qualification
