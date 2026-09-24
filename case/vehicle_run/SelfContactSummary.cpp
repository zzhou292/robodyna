#include "SelfContactSummary.h"

#include "lib_src/solvers/NodalTrialIdentity.h"
#include "output/ArtifactIO.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <ostream>
#include <stdexcept>

namespace crash::cases::vehicle_run {
namespace {

void Check(bool valid, const char* message) {
    if (!valid) throw std::invalid_argument(message);
}

bool Same(double a, double b) noexcept {
    return output::Bits(a) == output::Bits(b);
}

bool Partition(std::size_t total, std::initializer_list<std::size_t> counts) {
    for (const auto count : counts) {
        if (count > total) return false;
        total -= count;
    }
    return total == 0;
}

bool Finite(const tlfea::contact::Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

void CheckScope(const SelfContactTotals& totals,
    const vehicle_dynamics::StepObservation& step, const tl::fea::NodalStamp& accepted) {
    const auto& contact = step.self_contact;
    const auto& force = contact.accepted_force;
    const auto& solids = step.mechanics.solids;
    Check(contact.enabled && force.valid && accepted.owner_id && accepted.epoch &&
        accepted.reactions_valid && accepted.epoch - 1 == totals.intervals &&
        accepted.reaction_base_epoch == accepted.epoch - 1 &&
        step.base.owner_id == accepted.owner_id && step.base.epoch == accepted.epoch - 1 &&
        force.owner_id == accepted.owner_id && force.base_epoch == step.base.epoch &&
        force.attempt && force.configuration_id && force.qualification_id &&
        Same(force.position_time, step.base.time) &&
        Same(force.velocity_time, step.base.velocity_time) &&
        force.temporal_scheme == step.base.temporal_scheme &&
        force.velocity_phase == step.base.velocity_phase &&
        accepted.temporal_scheme == step.base.temporal_scheme &&
        Same(accepted.fixed_dt, step.base.fixed_dt) &&
        Same(accepted.reaction_time, step.base.time) &&
        Same(accepted.time, step.proposed_time) &&
        std::isfinite(step.base.time) && std::isfinite(step.base.velocity_time) &&
        std::isfinite(accepted.time) && std::isfinite(accepted.fixed_dt) &&
        accepted.fixed_dt > 0 && Same(accepted.time, step.base.time + accepted.fixed_dt) &&
        accepted.time > step.base.time,
        "Self-contact summary requires the next actual accepted-base force interval");

    Check(step.mechanics.valid && step.mechanics.has_solids &&
        tl::fea::trial_identity::SameStamp(step.mechanics.base_stamp, step.base) &&
        solids.valid && solids.has_completed_interval &&
        solids.phase == decltype(solids.phase)::Prepared &&
        solids.owner_id == accepted.owner_id && solids.epoch == accepted.epoch &&
        solids.base_epoch == step.base.epoch && solids.attempt == force.attempt &&
        Same(solids.time, accepted.time) && Same(solids.base_time, step.base.time) &&
        contact.regularity_generation && contact.policy_summary.complete,
        "Self-contact candidate observations are missing, stale or not commonly committed");

    Check(totals.available == (totals.intervals != 0),
        "Self-contact summary availability differs");
    if (totals.available) {
        Check(totals.owner_id == accepted.owner_id &&
            totals.configuration_id == force.configuration_id &&
            totals.qualification_id == force.qualification_id &&
            totals.last_attempt < force.attempt &&
            Same(totals.last_time_s, step.base.time) &&
            Same(totals.fixed_dt_s, accepted.fixed_dt),
            "Self-contact summary owner, configuration or clock changed");
    }
}

}  // namespace

void ObserveAcceptedSelfContact(SelfContactTotals& output,
    const vehicle_dynamics::StepObservation& step, const tl::fea::NodalStamp& accepted) {
    CheckScope(output, step, accepted);
    const auto& contact = step.self_contact;
    const auto& force = contact.accepted_force;
    const auto& policy = contact.policy_summary;
    Check(Partition(force.event_count,
              {force.vertex_face_event_count, force.edge_edge_event_count}) &&
        force.boundary_vertex_edge_event_count <= force.vertex_face_event_count &&
        force.active_count <= force.event_count,
        "Self-contact event counts do not form the declared VF/EE population");
    Check(policy.outcomes == contact.policy_outcomes &&
        policy.outcomes == contact.candidate_facet_pairs &&
        Partition(policy.outcomes, {policy.certified_separated,
            policy.excluded_same_rigid_group, policy.excluded_local_intersection,
            policy.represented_by_accepted_vf, policy.represented_by_accepted_ee}) &&
        (!policy.represented_by_accepted_vf || force.vertex_face_event_count) &&
        (!policy.represented_by_accepted_ee || force.edge_edge_event_count),
        "Self-contact policy partition or represented event family is invalid");
    for (const auto value : {force.potential_j, force.maximum_force_norm_n,
             force.maximum_sti_diagonal_n_m, force.maximum_represented_stiffness_n_m}) {
        Check(std::isfinite(value) && value >= 0,
            "Self-contact accepted-base scalar is nonfinite or negative");
    }
    Check(Finite(force.endpoint_a_resultant_n) && Finite(force.endpoint_b_resultant_n) &&
        Finite(force.equal_opposite_residual_n) && Finite(force.global_moment_n_m),
        "Self-contact resultant or moment diagnostic is nonfinite");

    auto next = output;
    next.available = true;
    next.intervals = accepted.epoch;
    next.owner_id = accepted.owner_id;
    next.configuration_id = force.configuration_id;
    next.qualification_id = force.qualification_id;
    next.last_attempt = force.attempt;
    next.fixed_dt_s = accepted.fixed_dt;
    next.last_base_time_s = step.base.time;
    next.last_base_velocity_time_s = step.base.velocity_time;
    next.last_time_s = accepted.time;
    next.last_event_count = force.event_count;
    next.peak_event_count = std::max(next.peak_event_count, next.last_event_count);
    next.last_active_count = force.active_count;
    next.last_vertex_face_events = force.vertex_face_event_count;
    next.last_boundary_vertex_edge_events = force.boundary_vertex_edge_event_count;
    next.last_edge_edge_events = force.edge_edge_event_count;
    next.last_accepted_parent_pairs = contact.accepted_broadphase_pairs;
    next.last_accepted_facet_pairs = contact.accepted_facet_pairs;
    next.last_discovered_features = contact.accepted_discovered_features;
    next.last_candidate_parent_pairs = contact.candidate_broadphase_pairs;
    next.last_candidate_facet_pairs = contact.candidate_facet_pairs;
    next.last_policy_outcomes = policy.outcomes;
    next.last_policy_digest = policy.digest;
    next.last_exact_crossing_pairs = policy.exact_crossing_pairs;
    next.last_exact_crossing_work = policy.exact_crossing_work;
    next.last_motion_certified_linear_separated = policy.motion_certified_linear_separated;
    next.last_linear_policy_coverage_pairs = policy.linear_policy_coverage_pairs;
    next.last_linear_policy_coverage_work = policy.linear_policy_coverage_work;
    next.last_nonlinear_subdivision_pairs = policy.nonlinear_subdivision_pairs;
    next.last_nonlinear_subdivision_work = policy.nonlinear_subdivision_work;
    next.last_certified_separated = policy.certified_separated;
    next.last_same_rigid_exclusions = policy.excluded_same_rigid_group;
    next.last_local_intersections = policy.excluded_local_intersection;
    next.last_represented_vf = policy.represented_by_accepted_vf;
    next.last_represented_ee = policy.represented_by_accepted_ee;
    next.last_regularity_generation = contact.regularity_generation;
    next.last_active_parents = contact.active_parents;
    next.last_removing_parents = contact.removing_parents;
    next.last_skipped_parents = contact.skipped_parents;
    next.last_accepted_base_potential_j = force.potential_j;
    next.peak_accepted_base_potential_j =
        std::max(next.peak_accepted_base_potential_j, force.potential_j);
    next.last_max_force_n = force.maximum_force_norm_n;
    next.peak_max_force_n = std::max(next.peak_max_force_n, force.maximum_force_norm_n);
    next.last_max_sti_n_m = force.maximum_sti_diagonal_n_m;
    next.peak_max_sti_n_m = std::max(next.peak_max_sti_n_m, force.maximum_sti_diagonal_n_m);
    next.last_max_represented_stiffness_n_m = force.maximum_represented_stiffness_n_m;
    next.last_equal_opposite_residual_n = {force.equal_opposite_residual_n.x,
        force.equal_opposite_residual_n.y, force.equal_opposite_residual_n.z};
    next.last_global_moment_n_m = {force.global_moment_n_m.x,
        force.global_moment_n_m.y, force.global_moment_n_m.z};
    output = next;
}

void detail::WriteSelfContactWorkProgress(std::ostream& output, const SelfContactTotals& totals) {
    if (!totals.available) return;
    output
        << " self_contact_exact_crossing_pairs=" << totals.last_exact_crossing_pairs
        << " self_contact_exact_crossing_work=" << totals.last_exact_crossing_work
        << " self_contact_motion_certified_linear_separated=" << totals.last_motion_certified_linear_separated
        << " self_contact_linear_policy_coverage_pairs=" << totals.last_linear_policy_coverage_pairs
        << " self_contact_linear_policy_coverage_work=" << totals.last_linear_policy_coverage_work
        << " self_contact_nonlinear_subdivision_pairs=" << totals.last_nonlinear_subdivision_pairs
        << " self_contact_nonlinear_subdivision_work=" << totals.last_nonlinear_subdivision_work;
}

}  // namespace crash::cases::vehicle_run
