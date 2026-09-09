#pragma once

#include "GuidedPlateModel.h"

namespace crash::reference {
struct GuidedPlateModalReport {
    std::array<double,kGuidedPlateDofs> squared_frequency{};
    std::array<double,kGuidedPlateDofs> initial_mode_increment{};
    ElasticCouponConfiguration initial_configuration;
    std::size_t selected_mode = 0;
    double bending_mass_fraction = 0, normal_translation_mass_fraction = 0;
    double reference_symmetry_error = 0, mass_scaled_derivative_refinement_error = 0;
    double tl_derivative_relative_error = 0, tl_directional_relative_error = 0;
    double maximum_frequency_refinement_error = 0, eigen_residual = 0;
    // Structural-only samples include both release directions. Negative scales
    // can exceed contact's depth envelope; they never admit contact/dynamics.
    std::array<double,6> sampled_amplitude{{-1,-.5,0,.5,1,2}};
    std::array<double,6> sampled_structural_operator_norm{};
    double sampled_structural_norm_maximum = 0, monitored_structural_norm_limit = 0;
    double contact_rate_bound = 0, combined_rate_envelope = 0;
    double membrane_wave_speed = 0, shortest_reference_edge = 0;
    double spectral_step_limit = 0, wave_step_limit = 0, rotary_step_limit = 0, proposed_step_limit = 0;
    double first_mode_angular_frequency = 0, initial_elastic_energy = 0;
    double time_step = 0, horizon = 0;
    std::uint64_t step_count = 0;
};

// No B2 mode, qualification identity or one-contributor energy admission is
// reused. This computes the guided 16-coordinate reference spectrum, validates
// its lowest bending mode, samples the full nonsymmetric structural operator,
// adds the independent contact rate and proposes N/h for the declared 0.2s.
// A complete run, contact/work candidate checks and h refinements remain D2/D3.
// Caller output is unchanged on failure; the diagnostic names the rejected gate.
ElasticCouponStatus AuditGuidedPlate(const GuidedPlateModel&,GuidedPlateModalReport& output,std::string& diagnostic);
ElasticCouponStatus MeasureGuidedPlateStructuralNorm(const GuidedPlateModel&,const ElasticCouponConfiguration&,
                                                     double& output,std::string& diagnostic);
}  // namespace crash::reference
