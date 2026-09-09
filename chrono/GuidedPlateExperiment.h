#pragma once

#include <cstdint>

namespace crash::reference {
enum class GuidedPlateExperiment { Original, PenaltyMarginV1 };

// Reviewed, named physical experiments, not an arbitrary penalty input API.
// Both keep the original geometry, mass, initial mode, depth stop and absolute
// integration budgets. Only PenaltyMarginV1 enforces the reduced-mode screen.
struct GuidedPlateExperimentSpec {
    GuidedPlateExperiment experiment;
    const char* name;
    std::uint64_t qualification_id;
    double stiffness_per_area;       // N/m^3, projected midsurface penalty.
    double target_penetration;      // m, prescribed screen; not a global bound.
    double maximum_penetration;     // m, unchanged runtime rejection boundary.
    double force_error;             // N, each original C2 absolute budget.
    double energy_error;            // J, including C3 area uncertainty.
};
inline constexpr double kGuidedOriginalForceError=1e-6*1e5*(.5*.2*.1)*.0005;
inline constexpr double kGuidedOriginalEnergyError=5e-9*1e5*(.5*.2*.1)*.0005*.0005;
inline constexpr GuidedPlateExperimentSpec kGuidedOriginalExperiment{
    GuidedPlateExperiment::Original,"original",0x4432475549444531ULL,1e5,.000375,.0005,
    kGuidedOriginalForceError,kGuidedOriginalEnergyError};
inline constexpr GuidedPlateExperimentSpec kGuidedPenaltyMarginV1Experiment{
    GuidedPlateExperiment::PenaltyMarginV1,"penalty-margin-v1",0x4432475549444532ULL,4e5,.000375,.0005,
    kGuidedOriginalForceError,kGuidedOriginalEnergyError};
inline constexpr const GuidedPlateExperimentSpec* FindGuidedPlateExperiment(GuidedPlateExperiment experiment) noexcept {
    switch (experiment) {
        case GuidedPlateExperiment::Original: return &kGuidedOriginalExperiment;
        case GuidedPlateExperiment::PenaltyMarginV1: return &kGuidedPenaltyMarginV1Experiment;
    }
    return nullptr;
}
} // namespace crash::reference
