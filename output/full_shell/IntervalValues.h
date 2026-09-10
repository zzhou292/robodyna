#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace crash::output::interval {
inline constexpr std::size_t IntegerCount = 4, RealCount = 35, ColumnCount = 39;
inline constexpr std::size_t RowBytes = (IntegerCount + RealCount) * 8;
// Single column authority for the existing CSV and the binary column arrays.
inline constexpr const char* CsvHeader =
    "owner_id,base_epoch,attempt,base_time_s,accepted_epoch,accepted_time_s,velocity_time_s,kick_dt_s,"
    "native_stored_kinetic_J,effective_stored_kinetic_J,native_internal_work_J,cumulative_plastic_work_J,"
    "native_kick_delta_J,effective_stored_delta_J,replacement_delta_J,applied_kick_work_J,reaction_kick_work_J,"
    "native_recurrence_residual_J,effective_bookkeeping_residual_J,roundoff_budget_J,"
    "wall_resultant_N,wall_resultant_error_N,wall_potential_J,wall_potential_error_J,"
    "wall_kick_work_J,wall_drift_work_J,wall_work_uncertainty_J,wall_quadratic_work_upper_J,"
    "wall_conservative_defect_J,wall_kick_impulse_N_s,wall_kick_impulse_error_N_s,"
    "maximum_penetration_m,active_contact_nodes,maximum_plastic_strain,yielded_points,yielded_parents,"
    "maximum_rotation_rad,maximum_area_ratio,maximum_thickness_ratio\n";
enum Integer : std::size_t { Owner, BaseEpoch, Attempt, Epoch };
enum Real : std::size_t {
    BaseTime, Time, VelocityTime, KickDt, NativeKinetic, EffectiveKinetic, NativeInternalWork,
    CumulativePlasticWork, NativeKickDelta, EffectiveDelta, ReplacementDelta, AppliedKickWork,
    ReactionKickWork, NativeResidual, EffectiveResidual, RoundoffBudget, WallResultant,
    WallResultantError, WallPotential, WallPotentialError, WallKickWork, WallDriftWork,
    WallWorkUncertainty, WallQuadraticWorkUpper, WallConservativeDefect, WallKickImpulse,
    WallKickImpulseError, MaximumPenetration, ActiveNodes, MaximumPlasticStrain,
    YieldedPoints, YieldedParents, MaximumRotation, MaximumAreaRatio, MaximumThicknessRatio
};
// Formatting-only values. No struct memory/padding is serialized. The first
// four integers are owner/base/attempt/accepted epoch; all other 35 fields,
// including recorded count diagnostics, preserve the original double semantics.
struct Values {
    std::array<std::uint64_t, IntegerCount> integers{};
    std::array<double, RealCount> reals{};
};
void CheckFinite(const Values&);
std::string CsvRow(const Values&);
std::vector<std::string> IntegerFields();
std::vector<std::string> RealFields();
} // namespace crash::output::interval
