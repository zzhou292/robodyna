#include "SelfContactDocument.h"
#include "contact_diagnostics/Document.h"
#include "output/BoundedArrayJson.h"

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
    Integer(document, "last_exact_crossing_pairs", totals.last_exact_crossing_pairs);
    Integer(document, "last_exact_crossing_work", totals.last_exact_crossing_work);
    Integer(document, "last_motion_certified_linear_separated", totals.last_motion_certified_linear_separated);
    Integer(document, "last_linear_policy_coverage_pairs", totals.last_linear_policy_coverage_pairs);
    Integer(document, "last_linear_policy_coverage_work", totals.last_linear_policy_coverage_work);
    Integer(document, "last_nonlinear_subdivision_pairs", totals.last_nonlinear_subdivision_pairs);
    Integer(document, "last_nonlinear_subdivision_work", totals.last_nonlinear_subdivision_work);
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
    if(totals.performance.enabled)
        array_json::Child(document,"performance_diagnostics",contact_diagnostics::Document(totals.performance));
    return document;
}


}  // namespace crash::cases::vehicle_run::detail
