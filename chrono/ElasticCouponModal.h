#pragma once

#include "ElasticCouponModel.h"

#include <array>
#include <cstdint>

namespace crash::reference {

// These are fixed pre-run experiment gates, not user-adjustable tolerances or
// a general nonlinear stability certificate. FD amplitudes have different
// units for translation and world rotation.
inline constexpr double kCouponTranslationDifference = 1e-6 * ElasticCouponData::length;
inline constexpr double kCouponRotationDifference = 1e-6;
inline constexpr double kCouponDerivativeTolerance = 1e-5;
inline constexpr double kCouponFrequencyRefinementTolerance = .005;

struct ElasticCouponModalReport {
    std::array<double, kCouponFreeDofs> squared_frequency{};  // Sorted, rad^2/s^2.
    std::array<double, kCouponFreeDofs> initial_mode_increment{};  // m/rad, 2 mm mean tip z.
    ElasticCouponConfiguration initial_configuration;
    std::size_t selected_mode = 0;
    double out_of_plane_mass_fraction = 0;
    double transverse_translation_mass_fraction = 0;
    double reference_symmetry_error = 0;
    double maximum_frequency_refinement_error = 0;
    double mass_scaled_derivative_refinement_error = 0;
    double tl_derivative_relative_error = 0;
    double tl_directional_relative_error = 0;
    double eigen_residual = 0;
    // Full mass-scaled WORLD force Jacobian operator norms at 0,.5,1,2
    // times the initial mode. No off-reference skew part is discarded.
    std::array<double, 4> sampled_amplitude{{0, .5, 1, 2}};
    std::array<double, 4> sampled_operator_norm{};
    double sampled_norm_maximum = 0;
    double monitored_norm_limit = 0;  // Twice sampled_norm_maximum.
    double membrane_wave_speed = 0;
    double shortest_reference_edge = 0;
    double spectral_step_limit = 0;
    double wave_step_limit = 0;
    double rotary_step_limit = 0;
    double proposed_step_limit = 0;
    double first_mode_angular_frequency = 0;
    double initial_energy = 0;
    // h is rounded downward by selecting N=ceil((pi/omega_mode)/h_limit).
    // Horizon is exactly N*h; refinements use h/2,N*2 and h/4,N*4.
    double time_step = 0;
    double horizon = 0;
    std::uint64_t step_count = 0;
};

// Independent coherent-Chrono FD oracle over every one of the 24 free DOFs,
// FD halving, host TL derivative cross-check, mass spectrum and sampled norm.
// All report fields are staged until all gates pass; failures explain the
// actual measured quantity in diagnostic. No GPU work or trajectory is run.
ElasticCouponStatus AuditElasticCoupon(const ElasticCouponModel& model,
                                      ElasticCouponModalReport& output,
                                      std::string& diagnostic);

// Saved-frame recheck for the case coordinator. The caller compares the
// result with the startup monitored_norm_limit before accepting its envelope.
// The complete nonsymmetric Jacobian is used, not an analytic tangent.
ElasticCouponStatus MeasureElasticCouponOperatorNorm(
    const ElasticCouponModel& model, const ElasticCouponConfiguration& configuration,
    double& output, std::string& diagnostic);

}  // namespace crash::reference
