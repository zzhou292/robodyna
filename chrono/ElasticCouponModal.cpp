#include "ElasticCouponModal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::reference {
namespace {
namespace audit = patch_audit;
using Matrix = audit::Matrix<kCouponFreeDofs>;
using Vector = audit::Vector<kCouponFreeDofs>;
using Status = ElasticCouponStatus;
using audit::Reject;

Status SelectInitialMode(const ElasticCouponModel& model,
                         const audit::ReferenceSpectrum<kCouponFreeDofs>& spectrum,
                         const Vector& inverse_root_mass, ElasticCouponModalReport& report,
                         std::string& diagnostic) {
    // This fixture is intended to release its lowest elastic bending mode.
    // A lower nonbending mode is diagnosed, not silently skipped or filtered.
    report.selected_mode = 0;
    const Vector mass_mode = spectrum.mass_modes.col(0);
    const Vector mode = inverse_root_mass.asDiagonal() * mass_mode;
    for (std::size_t free = 0; free < kCouponFreeNodes.size(); ++free) {
        report.transverse_translation_mass_fraction += std::pow(mass_mode(6 * free + 2), 2);
        for (const auto c : {2, 3, 4}) report.out_of_plane_mass_fraction += std::pow(mass_mode(6 * free + c), 2);
    }
    if (report.out_of_plane_mass_fraction < .9)
        return Reject("Lowest mode is not predominantly out-of-plane bending (required fraction >=0.9)",
                      report.out_of_plane_mass_fraction, .9, diagnostic);
    if (report.transverse_translation_mass_fraction < .5)
        return Reject("Lowest mode lacks transverse translational motion (required fraction >=0.5)",
                      report.transverse_translation_mass_fraction, .5, diagnostic);
    // Free entries 2 and 3 are the physical tip nodes 4 and 5. A twist mode has
    // opposite tip signs and cannot be normalized by the positive mean tip z.
    const double tip_a = mode(14), tip_b = mode(20);
    const double mean_tip = .5 * (tip_a + tip_b);
    if (!std::isfinite(mean_tip) || std::abs(mean_tip) <= .5 * std::max(std::abs(tip_a), std::abs(tip_b)))
        return Reject("Lowest mode lacks coherent nonzero transverse tip motion", mean_tip, 0, diagnostic);
    const double scale = ElasticCouponData::initial_tip_displacement / mean_tip;
    for (std::size_t c = 0; c < kCouponFreeDofs; ++c) report.initial_mode_increment[c] = scale * mode(c);
    return ApplyElasticCouponIncrement(model.data().reference_configuration, report.initial_mode_increment,
                                      1, report.initial_configuration, diagnostic);
}
}  // namespace

ElasticCouponStatus MeasureElasticCouponOperatorNorm(const ElasticCouponModel& model,
                                                      const ElasticCouponConfiguration& configuration,
                                                      double& output, std::string& diagnostic) {
    Matrix stiffness;
    const auto layout = audit::FullCouponLayout();
    const auto status = audit::DifferenceJacobian<kCouponFreeDofs>(model, configuration, layout, audit::ForceSource::Chrono, .5, stiffness, diagnostic);
    if (status != Status::kSuccess) return status;
    const double norm = audit::OperatorNorm<kCouponFreeDofs>(audit::MassScale<kCouponFreeDofs>(stiffness, audit::InverseRootMass<kCouponFreeDofs>(model.data(), layout)));
    if (!std::isfinite(norm) || norm <= 0) return Reject("Invalid coupon operator norm", norm, 0, diagnostic);
    output = norm;
    diagnostic.clear();
    return Status::kSuccess;
}

ElasticCouponStatus AuditElasticCoupon(const ElasticCouponModel& model, ElasticCouponModalReport& output,
                                       std::string& diagnostic) {
    if (!IsDefaultElasticCouponParameters(model.parameters())) {
        diagnostic="B2 dynamics audit requires its original fixture parameters; use explicit reference audits for parameterized screens";
        return Status::kInvalidConfiguration;
    }
    const auto& data = model.data();
    const auto& neutral = data.reference_configuration;
    const auto layout = audit::FullCouponLayout();
    audit::ReferenceSpectrum<kCouponFreeDofs> spectrum;
    auto status = audit::AuditReference<kCouponFreeDofs>(model, layout, spectrum, diagnostic);
    if (status != Status::kSuccess) return status;
    const Vector& inverse_root_mass = spectrum.inverse_root_mass;
    const Matrix& fine = spectrum.stiffness;
    const double smallest = spectrum.squared_frequency(0);
    ElasticCouponModalReport report;
    report.reference_symmetry_error = spectrum.symmetry_error;
    report.mass_scaled_derivative_refinement_error = spectrum.derivative_refinement_error;
    report.tl_derivative_relative_error = spectrum.tl_derivative_error;
    report.maximum_frequency_refinement_error = spectrum.maximum_frequency_refinement_error;
    report.eigen_residual = spectrum.eigen_residual;
    for (std::size_t c = 0; c < kCouponFreeDofs; ++c) report.squared_frequency[c] = spectrum.squared_frequency(c);
    status = SelectInitialMode(model, spectrum, inverse_root_mass, report, diagnostic);
    if (status != Status::kSuccess) return status;
    for (std::size_t sample = 0; sample < report.sampled_amplitude.size(); ++sample) {
        ElasticCouponConfiguration configuration;
        status = ApplyElasticCouponIncrement(neutral, report.initial_mode_increment, report.sampled_amplitude[sample],
                                            configuration, diagnostic);
        if (status != Status::kSuccess) return status;
        Matrix stiffness;
        if (sample == 0) stiffness = fine;
        else {
            status = audit::DifferenceJacobian<kCouponFreeDofs>(model, configuration, layout, audit::ForceSource::Chrono, .5, stiffness, diagnostic);
            if (status != Status::kSuccess) return status;
        }
        const double norm = audit::OperatorNorm<kCouponFreeDofs>(audit::MassScale<kCouponFreeDofs>(stiffness, inverse_root_mass));
        if (!std::isfinite(norm) || norm <= 0) return Reject("Sampled operator norm", norm, 0, diagnostic);
        report.sampled_operator_norm[sample] = norm;
        report.sampled_norm_maximum = std::max(report.sampled_norm_maximum, norm);
        status = audit::DirectionalCrossCheck<kCouponFreeDofs>(model, configuration, layout, stiffness, inverse_root_mass,
                                       report.tl_directional_relative_error, diagnostic);
        if (status != Status::kSuccess) return status;
    }
    report.monitored_norm_limit = 2 * report.sampled_norm_maximum;
    report.membrane_wave_speed = std::sqrt(data.young_modulus / (data.density * (1 - data.poisson_ratio * data.poisson_ratio)));
    report.shortest_reference_edge = std::numeric_limits<double>::infinity();
    for (const auto& connectivity : data.connectivity)
        for (std::size_t n = 0; n < 4; ++n) {
            const auto edge = tl::fea::reissner::detail::Subtract(neutral.position[connectivity[n]],
                                                                neutral.position[connectivity[(n + 1) % 4]]);
            report.shortest_reference_edge = std::min(report.shortest_reference_edge,
                                                       std::sqrt(tl::fea::reissner::detail::Dot(edge, edge)));
        }
    const double shear_modulus = data.young_modulus / (2 * (1 + data.poisson_ratio));
    report.spectral_step_limit = .1 / std::sqrt(report.monitored_norm_limit);
    report.wave_step_limit = .1 * report.shortest_reference_edge / report.membrane_wave_speed;
    report.rotary_step_limit = .1 * data.thickness / std::sqrt(12 * shear_modulus / data.density);
    report.proposed_step_limit = std::min({report.spectral_step_limit, report.wave_step_limit, report.rotary_step_limit});
    report.first_mode_angular_frequency = std::sqrt(smallest);
    const double half_period = std::acos(-1.0) / report.first_mode_angular_frequency;
    const double steps = std::ceil(half_period / report.proposed_step_limit);
    if (!std::isfinite(steps) || steps < 1 || steps > 1000000)
        return Reject("Coupon half-period exceeds one-million-step startup cap", steps, 1000000, diagnostic);
    report.step_count = static_cast<std::uint64_t>(steps);
    report.time_step = half_period / report.step_count;
    if (report.time_step > report.proposed_step_limit) report.time_step = half_period / ++report.step_count;
    report.horizon = report.step_count * report.time_step;
    ElasticCouponEvaluation initial;
    status = model.EvaluateTL(report.initial_configuration, initial, diagnostic);
    if (status != Status::kSuccess) return status;
    if (!std::isfinite(initial.energy) || initial.energy <= 0)
        return Reject("Initial bending strain energy must be positive", initial.energy, 0, diagnostic);
    report.initial_energy = initial.energy;
    output = report;
    diagnostic.clear();
    return Status::kSuccess;
}

}  // namespace crash::reference
