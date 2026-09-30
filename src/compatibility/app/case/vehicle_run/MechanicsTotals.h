#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace crash::cases::vehicle_run {
struct WorkObservation {
    double last_increment_j = 0;
    double accepted_increment_sum_j = 0;
    double peak_absolute_increment_j = 0;
};
struct PlasticWorkObservation {
    WorkObservation work;
    std::uint64_t first_positive_epoch = 0;
    double first_positive_time_s = 0;
};
struct NativeStepObservation {
    double last_s = 0;
    double minimum_observed_s = 0;
};
struct SolidMechanicsTotals {
    // Same order as solids::BatchDiagnostics; absent families retain count zero.
    std::array<std::size_t, 5> parents{};
    std::array<WorkObservation, 5> native_work;
    std::array<WorkObservation, 5> hourglass_work;
    // Existing diagnostics provide one combined LAW36/LAW44 value. It is
    // already included in native work, not a separate energy contribution.
    PlasticWorkObservation metal_plastic_work;
    WorkObservation rhs_kick_work, rhs_drift_work;
    NativeStepObservation native_step;
};
struct BeamMechanicsTotals {
    std::size_t parents = 0;
    // Native EINT channels: membrane/shear, then flexural/torsional.
    std::array<WorkObservation, 2> native_work;
    PlasticWorkObservation plastic_work;
    WorkObservation rhs_kick_work, rhs_drift_work;
    NativeStepObservation native_step;
};
struct MotionMaximum {
    double last = 0, peak = 0;
};
struct MotionTotals {
    std::size_t nodes = 0;
    // Component maxima relative to original uniform translation, not strain,
    // fitted rigid-motion residuals, Euclidean norms or physical error limits.
    MotionMaximum translation_departure_m, velocity_departure_m_s;
    MotionMaximum orientation_component_departure, spin_component_rad_s;
};
struct MechanicsTotals {
    bool available = false, has_beam18 = false, has_type45 = false;
    std::uint64_t intervals = 0, owner_id = 0, source_instance_id = 0;
    std::uint64_t configuration_id = 0, qualification_id = 0, last_attempt = 0;
    double last_time_s = 0, last_velocity_time_s = 0, fixed_dt_s = 0;
    SolidMechanicsTotals solids;
    BeamMechanicsTotals beam18;
    MotionTotals motion;
};
// Fixed scalar storage, covered (including transient copies) by the existing
// four-MiB controller reservation; no material arrays or per-step allocation.
static_assert(sizeof(MechanicsTotals) <= 4096);
} // namespace crash::cases::vehicle_run
