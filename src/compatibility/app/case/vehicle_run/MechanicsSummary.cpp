#include "MechanicsSummary.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace crash::cases::vehicle_run {
namespace {
void Check(bool valid, const char* message) {
    if (!valid) throw std::invalid_argument(message);
}
bool Same(double a, double b) noexcept {
    return output::Bits(a) == output::Bits(b);
}
template<class Diagnostics>
void CheckParticipant(const Diagnostics& row, const vehicle_dynamics::StepObservation& step,
    const tl::fea::NodalStamp& accepted) {
    const auto& identity = step.mechanics.solids;
    Check(row.valid && row.has_completed_interval && row.phase == decltype(row.phase)::Prepared &&
        row.owner_id == accepted.owner_id && row.configuration_id == identity.configuration_id &&
        row.qualification_id == identity.qualification_id && row.epoch == accepted.epoch &&
        row.base_epoch == step.base.epoch && row.attempt == identity.attempt &&
        Same(row.time, accepted.time) && Same(row.base_time, step.base.time) &&
        Same(row.velocity_time, accepted.velocity_time) &&
        Same(row.kick_dt, accepted.reaction_kick_dt),
        "Mechanics summary participant is missing, stale or not commonly committed");
}
void CheckScope(const MechanicsTotals& totals, const vehicle_dynamics::StepObservation& step,
    const tl::fea::NodalStamp& accepted) {
    const auto& mechanics = step.mechanics;
    const auto& solids = mechanics.solids;
    Check(accepted.owner_id && accepted.epoch && accepted.reactions_valid &&
        accepted.node_count && accepted.epoch - 1 == totals.intervals &&
        accepted.reaction_base_epoch == accepted.epoch - 1 &&
        step.base.owner_id == accepted.owner_id && step.base.epoch == accepted.reaction_base_epoch &&
        Same(step.base.time, accepted.reaction_time) && Same(step.proposed_time, accepted.time) &&
        Same(step.base.fixed_dt, accepted.fixed_dt) &&
        tl::fea::trial_identity::SameStamp(mechanics.base_stamp, step.base) &&
        std::isfinite(accepted.time) && std::isfinite(step.base.time) && accepted.time > step.base.time &&
        std::isfinite(accepted.fixed_dt) && accepted.fixed_dt > 0 &&
        std::isfinite(accepted.velocity_time) && std::isfinite(accepted.reaction_kick_dt) &&
        accepted.reaction_kick_dt > 0 && mechanics.valid && !mechanics.kinetic_available &&
        mechanics.has_qeph && mechanics.has_t3 && mechanics.has_qbat && mechanics.has_type25 &&
        mechanics.has_type13 && mechanics.has_solids && solids.source_instance_id &&
        solids.configuration_id && solids.qualification_id && solids.attempt && solids.accepted_force_assembled &&
        Same(solids.base_velocity_time, step.base.velocity_time),
        "Mechanics summary requires the next complete actual accepted interval");
    CheckParticipant(mechanics.qeph, step, accepted);
    CheckParticipant(mechanics.t3, step, accepted);
    CheckParticipant(mechanics.qbat, step, accepted);
    CheckParticipant(mechanics.type25, step, accepted);
    CheckParticipant(mechanics.type13, step, accepted);
    CheckParticipant(solids, step, accepted);
    if (mechanics.has_type45) CheckParticipant(mechanics.type45, step, accepted);
    if (mechanics.has_beam18) {
        CheckParticipant(mechanics.beam18, step, accepted);
        Check(mechanics.beam18.source_instance_id == solids.source_instance_id &&
            mechanics.beam18.parent_count && mechanics.beam18.accepted_force_assembled &&
            Same(mechanics.beam18.base_velocity_time, step.base.velocity_time),
            "Mechanics summary structural beam source or accepted phase differs");
    }
    Check(totals.available == (totals.intervals != 0), "Mechanics summary availability differs");
    if (totals.available) {
        Check(totals.owner_id == accepted.owner_id && totals.source_instance_id == solids.source_instance_id &&
            totals.configuration_id == solids.configuration_id && totals.qualification_id == solids.qualification_id &&
            totals.has_beam18 == mechanics.has_beam18 && totals.has_type45 == mechanics.has_type45 &&
            totals.last_attempt < solids.attempt && Same(totals.last_time_s, step.base.time) &&
            Same(totals.last_velocity_time_s, step.base.velocity_time) &&
            Same(totals.fixed_dt_s, accepted.fixed_dt) && totals.motion.nodes == accepted.node_count,
            "Mechanics summary source, clock or participant population changed");
    }
}
void ObserveWork(WorkObservation& output, double increment) {
    Check(std::isfinite(increment), "Accepted mechanics work is nonfinite");
    const double sum = output.accepted_increment_sum_j + increment;
    Check(std::isfinite(sum), "Accepted mechanics work sum overflow");
    output.last_increment_j = increment;
    output.accepted_increment_sum_j = sum;
    output.peak_absolute_increment_j = std::max(output.peak_absolute_increment_j, std::abs(increment));
}
void ObservePlastic(PlasticWorkObservation& output, double increment, const tl::fea::NodalStamp& accepted) {
    Check(increment >= 0, "Accepted plastic work increment is negative");
    ObserveWork(output.work, increment);
    if (increment > 0 && !output.first_positive_epoch) {
        output.first_positive_epoch = accepted.epoch;
        output.first_positive_time_s = accepted.time;
    }
}
void ObserveNativeStep(NativeStepObservation& output, double value) {
    Check(std::isfinite(value) && value > 0, "Accepted native element timestep is unavailable or invalid");
    output.last_s = value;
    output.minimum_observed_s = output.minimum_observed_s > 0 ? std::min(output.minimum_observed_s, value) : value;
}
void ObserveMaximum(MotionMaximum& output, double value) {
    Check(std::isfinite(value) && value >= 0, "Accepted motion maximum is nonfinite or negative");
    output.last = value;
    output.peak = std::max(output.peak, value);
}
} // namespace

void ObserveAcceptedMechanics(MechanicsTotals& output, const vehicle_dynamics::StepObservation& step,
    const tl::fea::NodalStamp& accepted) {
    CheckScope(output, step, accepted);
    auto next = output;
    const auto& solids = step.mechanics.solids;
    bool any_solid = false;
    for (std::size_t family = 0; family < next.solids.parents.size(); ++family) {
        const auto count = solids.parent_count[family];
        Check(!output.available || count == output.solids.parents[family], "Accepted solid population changed");
        Check(count || (solids.native_internal_work_increment_j[family] == 0 &&
            solids.physical_hourglass_work_increment_j[family] == 0), "Absent solid family reports work");
        any_solid = any_solid || count != 0;
        next.solids.parents[family] = count;
        ObserveWork(next.solids.native_work[family], solids.native_internal_work_increment_j[family]);
        ObserveWork(next.solids.hourglass_work[family], solids.physical_hourglass_work_increment_j[family]);
    }
    Check(any_solid, "Accepted solid participant is empty");
    ObservePlastic(next.solids.metal_plastic_work, solids.plastic_work_increment_j, accepted);
    ObserveWork(next.solids.rhs_kick_work, solids.internal_kick_work_j);
    ObserveWork(next.solids.rhs_drift_work, solids.internal_drift_work_j);
    ObserveNativeStep(next.solids.native_step, solids.minimum_native_dt_s);
    if (step.mechanics.has_beam18) {
        const auto& beam = step.mechanics.beam18;
        Check(!output.available || beam.parent_count == output.beam18.parents, "Accepted structural beam population changed");
        next.beam18.parents = beam.parent_count;
        for (std::size_t channel = 0; channel < next.beam18.native_work.size(); ++channel)
            ObserveWork(next.beam18.native_work[channel], beam.native_internal_work_increment_j[channel]);
        ObservePlastic(next.beam18.plastic_work, beam.plastic_work_increment_j, accepted);
        ObserveWork(next.beam18.rhs_kick_work, beam.internal_kick_work_j);
        ObserveWork(next.beam18.rhs_drift_work, beam.internal_drift_work_j);
        ObserveNativeStep(next.beam18.native_step, beam.minimum_native_dt_s);
    }
    const auto& motion = step.uniform_motion;
    Check(motion.nodes == accepted.node_count, "Accepted motion summary omits physical domain nodes");
    next.motion.nodes = motion.nodes;
    ObserveMaximum(next.motion.translation_departure_m, motion.maximum_position_error);
    ObserveMaximum(next.motion.velocity_departure_m_s, motion.maximum_velocity_error);
    ObserveMaximum(next.motion.orientation_component_departure, motion.maximum_orientation_error);
    ObserveMaximum(next.motion.spin_component_rad_s, motion.maximum_spin);
    next.available = true;
    next.has_beam18 = step.mechanics.has_beam18;
    next.has_type45 = step.mechanics.has_type45;
    next.intervals = accepted.epoch;
    next.owner_id = accepted.owner_id;
    next.source_instance_id = solids.source_instance_id;
    next.configuration_id = solids.configuration_id;
    next.qualification_id = solids.qualification_id;
    next.last_attempt = solids.attempt;
    next.last_time_s = accepted.time;
    next.last_velocity_time_s = accepted.velocity_time;
    next.fixed_dt_s = accepted.fixed_dt;
    output = next;
}
} // namespace crash::cases::vehicle_run
