#pragma once

#include "ShellModeComparison.h"
#include "ShellPatchInertia.h"

namespace crash::reference {

inline constexpr std::size_t kThinShellFixtureCount=6;
inline constexpr std::size_t kThinShellPolicyCount=2;
inline constexpr double kThinShellSourceThickness=.001648;
inline constexpr double kThinShellForecastHorizon=.2;
// A finite-amplitude diagnostic, not an extra acceptance or linearity gate.
// Normalize each physical mode by max(max_node|u|,edge*max_node|theta|),
// then prescribe amplitudes 1e-4*min(edge,t) and half that value.
inline constexpr double kThinShellShearAmplitudeFraction=1e-4;

using ThinShellMatrix=patch_audit::Matrix<kCouponFreeDofs>;
using ThinShellVector=patch_audit::Vector<kCouponFreeDofs>;
enum class ThinShellInertiaPolicy { PhysicalThickness, AreaCounterfactual };

struct ThinShellMatrixMeasurement {
    bool available=false;
    ElasticCouponStatus status=ElasticCouponStatus::kInvalidConfiguration;
    std::string diagnostic;
    ThinShellMatrix stiffness=ThinShellMatrix::Zero();
};

struct ThinShellModeDiagnostic {
    bool positive_frequency=false;
    double angular_frequency=0;
    double out_of_plane_mass_fraction=0,normal_translation_mass_fraction=0;
    double translation_mass_fraction=0,transverse_rotation_mass_fraction=0,drilling_mass_fraction=0;
    // Unit mass-normalized modal velocity, in physical world coordinates.
    // Baseline has zero area-added terms; its original numerical drilling
    // remains explicit. This is not an energy ledger from a trajectory.
    bool kinetic_available=false;
    ElasticCouponStatus kinetic_status=ElasticCouponStatus::kInvalidConfiguration;
    std::string kinetic_diagnostic;
    ShellPatchKineticEnergy kinetic;
    // Evaluated for fine modes only; coarse modes retain kinetic/participation
    // diagnostics without doubling these prescribed force observations.
    bool finite_amplitude_available=false;
    ElasticCouponStatus finite_amplitude_status=ElasticCouponStatus::kInvalidConfiguration;
    std::string finite_amplitude_diagnostic;
    std::array<double,2> amplitude_m{},total_energy{},bending_energy{},shear_energy{},shear_fraction{};
};

struct ThinShellClusterDiagnostic {
    std::uint32_t label=0;
    std::size_t count=0;
    std::array<std::size_t,kCouponFreeDofs> modes{};
    double mean_out_of_plane_fraction=0,mean_normal_translation_fraction=0;
    double tip_a_row_norm=0,tip_b_row_norm=0,mean_tip_row_norm=0;
    bool bending_eligible=false;
    std::string diagnostic;
};

// Raw symmetric eigendiagnostics are retained even if AuditReference rejects
// symmetry, finite-difference uncertainty or positivity. They are never an
// admitted spectrum unless the separate owning audit says so.
struct ThinShellSpectrumDiagnostic {
    bool available=false,positive_spectrum=false;
    std::string diagnostic;
    ThinShellVector squared_frequency=ThinShellVector::Zero();
    ThinShellMatrix mass_modes=ThinShellMatrix::Zero();
    double symmetry_error=0,eigen_residual=0;
    std::array<ThinShellModeDiagnostic,kCouponFreeDofs> modes{};
    std::size_t cluster_count=0;
    std::array<ThinShellClusterDiagnostic,kCouponFreeDofs> clusters{};
};

struct ThinShellModeMatchDiagnostic {
    bool attempted=false,matched=false,frequency_refined=false;
    double maximum_fd_relative_frequency_change=0; // coarse denominator; FD matches only
    ShellModeComparisonStatus status=ShellModeComparisonStatus::kInvalidInput;
    std::string diagnostic;
    // Indices in the retained full spectrum for each compact comparison pool.
    std::array<std::size_t,kCouponFreeDofs> reference_modes{},candidate_modes{};
    ShellModeSet reference_pool,candidate_pool;
    ShellModeComparison comparison;
};

struct ThinShellStepDiagnostic {
    bool available=false;
    std::string diagnostic;
    double neutral_operator_norm=0,neutral_spectral_limit=0,neutral_spectral_steps_200ms=0;
    double membrane_wave_speed=0,wave_limit=0,physical_rotary_limit=0;
    // Only the original thickness policy receives this three-way minimum.
    // It uses neutral norm alone, not B2's nonlinear sampled envelope, and is
    // an optimistic cost screen. No counterfactual rotary policy is invented.
    bool original_policy_estimate_available=false;
    double original_policy_limit=0,original_policy_steps_200ms=0;
};

struct ThinShellPolicyDiagnostic {
    ThinShellInertiaPolicy policy=ThinShellInertiaPolicy::PhysicalThickness;
    patch_audit::PatchNodalMass mass;
    ThinShellVector inverse_root_mass=ThinShellVector::Zero();
    std::array<ThinShellSpectrumDiagnostic,2> spectrum; // coarse, fine Chrono
    bool reference_attempted=false,reference_passed=false;
    ElasticCouponStatus reference_status=ElasticCouponStatus::kInvalidConfiguration;
    std::string reference_diagnostic;
    double reference_symmetry_error=0,derivative_refinement_error=0,tl_derivative_error=0;
    double maximum_frequency_refinement_error=0,reference_eigen_residual=0;
    bool directional_attempted=false;
    ElasticCouponStatus directional_status=ElasticCouponStatus::kInvalidConfiguration;
    double directional_error=0;
    std::string directional_diagnostic;
    // Shape reference=fine, candidate=coarse; FD relative frequency change
    // uses the coarse denominator as in the existing reference audit.
    ThinShellModeMatchDiagnostic coarse_fine_match;
    ThinShellStepDiagnostic step;
};

struct ThinShellFixtureDiagnostic {
    std::size_t fixture_index=0;
    double edge_m=0,thickness_multiplier=0;
    ElasticCouponParameters parameters;
    patch_audit::PatchDifferenceSteps difference_steps;
    bool setup_available=false,screen_passed=false,simulation_ready=false;
    std::string diagnostic;
    ElasticCouponConfiguration reference_configuration;
    std::array<std::array<std::size_t,4>,kCouponElements> connectivity{};
    ShellPatchInertia inertia;
    // All three raw matrices are measured and retained BEFORE AuditReference.
    std::array<ThinShellMatrixMeasurement,3> measurement; // Chrono coarse/fine, TL fine
    std::array<ThinShellPolicyDiagnostic,kThinShellPolicyCount> policies;
    ThinShellModeMatchDiagnostic hybrid_match;
};

struct ThinShellScreenDiagnostic {
    bool screen_passed=false,simulation_ready=false;
    std::array<ThinShellFixtureDiagnostic,kThinShellFixtureCount> fixtures;
};

// Fixed index order: edges .01/.02 m, each t/2,t,2t. E=200 GPa,
// rho=7890 kg/m^3, nu=.3, alpha=5/6, beta=.01 are explicit synthetic
// elastic overrides. Translation FD step=1e-6*edge, rotation=1e-6 rad.
// No MAT024 defaults, source geometry flattening, GPU or dynamics admission.
ElasticCouponParameters ThinShellFixtureParameters(std::size_t fixture_index);
// Invalid index throws. Expected setup/audit/mode rejection is a complete
// diagnostic result; all six cases remain observable in the batch operation.
ThinShellFixtureDiagnostic ScreenThinShellFixture(std::size_t fixture_index);
ThinShellScreenDiagnostic ScreenThinShellFixtures();

} // namespace crash::reference
