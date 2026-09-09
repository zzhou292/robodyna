#pragma once

#include "GuidedPlateModal.h"

namespace crash::reference {
struct GuidedPlatePenaltyDerivative {
    double step_m=0,derivative_N=0,restoring_force_N=0;
    double error_N=0,contact_uncertainty_N=0,allowance_N=0;
    bool backward=false;
};
struct GuidedPlatePenaltySample {
    double requested_penetration=0,actual_penetration=0,modal_scale=0;
    tlfea::contact::Q4IntegralInterval maximum_depth;
    unsigned inward_scale_adjustments=0;
    double shell_energy_tl=0,shell_energy_chrono=0,shell_energy_allowance=0;
    tlfea::contact::Q4CertifiedIntegral contact_potential,contact_resultant;
    double strip_potential=0,strip_resultant=0,strip_energy_allowance=0,strip_force_allowance=0;
    double total_potential_lower=0;
    std::array<GuidedPlatePenaltyDerivative,2> derivative{};
};
struct GuidedPlatePenaltyReport {
    GuidedPlateExperiment experiment=GuidedPlateExperiment::Original;
    std::uint64_t qualification_id=0;
    double initial_energy=0,admitted_energy_upper=0;
    std::array<GuidedPlatePenaltySample,2> sample{}; // target, then hard-cap boundary.
    bool target_sufficient=false,enforced=false;
};

// Startup-only reduced-mode diagnostic, NOT a global penetration theorem.
// Actual unsnapped modal x/q go through Chrono/TL shell forces and certified
// rectangular C2 contact with original absolute budgets and C3 area expansion.
// An independently integrated width-averaged affine strip is only an oracle.
// FD steps are 2e-7/1e-7 m; target uses central differences, the hard cap uses
// backward differences to stay inside the unchanged contact domain. Maximum
// tip, not rounded mean tip, defines amplitude. Any representational inward
// scale correction is recorded; initial state/reference geometry never changes.
// Failure preserves output. Original reports insufficient capacity without
// rejecting; PenaltyMarginV1 requires target lower energy > 1.01*initial energy.
ElasticCouponStatus AuditGuidedPlatePenalty(const GuidedPlateModel&,const GuidedPlateModalReport&,
                                          GuidedPlatePenaltyReport& output,std::string& diagnostic);
} // namespace crash::reference
