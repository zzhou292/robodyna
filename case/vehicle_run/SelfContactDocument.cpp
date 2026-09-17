#include "SelfContactDocument.h"

#include "case/vehicle_self_contact/SelfContactStageError.h"

namespace crash::cases::vehicle_run::detail {

output::Document SelfContactDocument(const SelfContactTotals& totals) {
    using namespace output;
    Document document;
    document.SetObject();
    String(document, "schema", "robo_dyna.accepted_self_contact_summary.v1");
    Boolean(document, "available", totals.available);
    if (!totals.available) return document;
    String(document, "scope",
        "committed interval diagnostics: force and potential at accepted base; "
        "policy counts cover the sealed candidate interval; no endpoint energy or work ledger");
    String(document, "boundary_vertex_edge_scope", "subset of vertex-face events, not an additional event family");
    Integer(document, "accepted_intervals", totals.intervals);
    Integer(document, "owner_id", totals.owner_id);
    Integer(document, "configuration_id", totals.configuration_id);
    Integer(document, "qualification_id", totals.qualification_id);
    Integer(document, "last_attempt", totals.last_attempt);
    Number(document, "fixed_dt_s", totals.fixed_dt_s);
    Number(document, "last_base_time_s", totals.last_base_time_s);
    Number(document, "last_base_velocity_time_s", totals.last_base_velocity_time_s);
    Number(document, "last_time_s", totals.last_time_s);
    Integer(document, "last_event_count", totals.last_event_count);
    Integer(document, "peak_event_count", totals.peak_event_count);
    Integer(document, "last_active_count", totals.last_active_count);
    Integer(document, "last_vertex_face_events", totals.last_vertex_face_events);
    Integer(document, "last_boundary_vertex_edge_events", totals.last_boundary_vertex_edge_events);
    Integer(document, "last_edge_edge_events", totals.last_edge_edge_events);
    Integer(document, "last_accepted_parent_pairs", totals.last_accepted_parent_pairs);
    Integer(document, "last_accepted_facet_pairs", totals.last_accepted_facet_pairs);
    Integer(document, "last_discovered_features", totals.last_discovered_features);
    Integer(document, "last_candidate_parent_pairs", totals.last_candidate_parent_pairs);
    Integer(document, "last_candidate_facet_pairs", totals.last_candidate_facet_pairs);
    Integer(document, "last_policy_outcomes", totals.last_policy_outcomes);
    Integer(document, "last_policy_digest", totals.last_policy_digest);
    Integer(document, "last_certified_separated", totals.last_certified_separated);
    Integer(document, "last_same_rigid_exclusions", totals.last_same_rigid_exclusions);
    Integer(document, "last_local_intersections", totals.last_local_intersections);
    Integer(document, "last_represented_vf", totals.last_represented_vf);
    Integer(document, "last_represented_ee", totals.last_represented_ee);
    Integer(document, "last_regularity_generation", totals.last_regularity_generation);
    Integer(document, "last_active_parents", totals.last_active_parents);
    Integer(document, "last_removing_parents", totals.last_removing_parents);
    Integer(document, "last_skipped_parents", totals.last_skipped_parents);
    Number(document, "last_accepted_base_potential_j", totals.last_accepted_base_potential_j);
    Number(document, "peak_accepted_base_potential_j", totals.peak_accepted_base_potential_j);
    Number(document, "last_max_force_n", totals.last_max_force_n);
    Number(document, "peak_max_force_n", totals.peak_max_force_n);
    Number(document, "last_max_sti_n_m", totals.last_max_sti_n_m);
    Number(document, "peak_max_sti_n_m", totals.peak_max_sti_n_m);
    Number(document, "last_max_represented_stiffness_n_m", totals.last_max_represented_stiffness_n_m);
    FiniteArray(document, "last_equal_opposite_residual_n",
        totals.last_equal_opposite_residual_n.data(), totals.last_equal_opposite_residual_n.size());
    FiniteArray(document, "last_global_moment_n_m",
        totals.last_global_moment_n_m.data(), totals.last_global_moment_n_m.size());
    return document;
}

output::Document SelfContactErrorDocument(
    const vehicle_self_contact::SelfContactStageError& error) {
    using namespace output;
    using Count = tlfea::contact::SelfContactTransactionCountKind;
    const auto& report = error.report();
    Document document;
    document.SetObject();
    String(document, "schema", "robo_dyna.self_contact_stage_error.v1");
    String(document, "stage", vehicle_self_contact::SelfContactStageError::StageName(error.stage()));
    String(document, "message", report.message);
    Integer(document, "status_code", static_cast<unsigned>(report.status));
    String(document, "count_kind", report.count_kind == Count::ExactAcceptedEvents
        ? "exact_accepted_events" : report.count_kind == Count::AcceptedEventsLowerBound
        ? "accepted_events_lower_bound" : "none");
    if (report.candidate != SIZE_MAX) {
        const char* name = report.count_kind == Count::ExactAcceptedEvents
            ? "reported_exact_accepted_events" : report.count_kind == Count::AcceptedEventsLowerBound
            ? "reported_accepted_events_lower_bound" : "candidate_ordinal";
        Integer(document, name, report.candidate);
    }
    if (error.required_events()) Integer(document, "required_force_events", error.required_events());
    if (error.required_events_lower_bound())
        Integer(document, "required_events_lower_bound", error.required_events_lower_bound());
    if (report.pair != SIZE_MAX) Integer(document, "pair_ordinal", report.pair);
    if (report.discovery_task != SIZE_MAX) Integer(document, "discovery_task", report.discovery_task);
    Integer(document, "force_status_code", static_cast<unsigned>(report.force_status));
    Integer(document, "activity_status_code", static_cast<unsigned>(report.activity_status));
    Integer(document, "broadphase_status_code", static_cast<unsigned>(report.broadphase_status));
    Integer(document, "regularity_status_code", static_cast<unsigned>(report.regularity_status));
    Integer(document, "discovery_status_code", static_cast<unsigned>(report.discovery_status));
    Integer(document, "discovery_reason_code", static_cast<unsigned>(report.discovery_reason));
    Integer(document, "crossing_status_code", static_cast<unsigned>(report.crossing_status));
    Integer(document, "crossing_reason_code", static_cast<unsigned>(report.crossing_reason));
    Integer(document, "publication_status_code", static_cast<unsigned>(report.publication_status));
    Integer(document, "owner_status_code", static_cast<unsigned>(report.owner_status));
    Integer(document, "nonlinear_subdivision_work", report.nonlinear_subdivision_work);
    Integer(document, "nonlinear_subdivision_depth", report.nonlinear_subdivision_depth);
    Boolean(document, "nonlinear_subdivision_work_exhausted", report.nonlinear_subdivision_work_exhausted);
    Boolean(document, "nonlinear_subdivision_depth_exhausted", report.nonlinear_subdivision_depth_exhausted);
    return document;
}

}  // namespace crash::cases::vehicle_run::detail
