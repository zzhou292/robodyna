#include "ElasticCouponModel.h"
#include "ElasticCouponModal.h"

#include "chrono/core/ChQuaternion.h"
#include "math/Quaternion.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace crash::qualification {
namespace {
using namespace reference;
namespace tlr = tl::fea::reissner;
using Status = ElasticCouponStatus;

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

}  // namespace
}  // namespace crash::qualification
