#include "PreparedCensusReplay.h"
#include "FailureBaselineScope.h"
#include "output/BoundedArrayJson.h"

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay {
namespace {
namespace contact = tlfea::contact;
namespace sct = contact::self_contact_transaction;
using namespace output;

template <class Key>
Document Facet(const Key& key) {
    Document d; d.SetObject();
    Integer(d, "source_instance_id", key.source_instance_id); Integer(d, "parent_eid", key.parent_eid);
    Integer(d, "level", key.level); Integer(d, "local_facet", key.local_facet); return d;
}
Document Vertex(const contact::FacetVertexKey& key) {
    Document d; d.SetObject();
    Integer(d, "source_instance_id", key.source_instance_id); Integer(d, "first", key.first);
    Integer(d, "second", key.second); Integer(d, "kind", static_cast<unsigned>(key.kind));
    Integer(d, "numerator", key.numerator); Integer(d, "denominator", key.denominator);
    Integer(d, "level", key.level); Integer(d, "grid_i", key.grid_i); Integer(d, "grid_j", key.grid_j); return d;
}
Document Edge(const contact::FacetEdgeKey& key) {
    Document d; d.SetObject();
    array_json::Child(d, "first", Vertex(key.endpoints[0])); array_json::Child(d, "second", Vertex(key.endpoints[1]));
    Integer(d, "parent_eid", key.parent_eid); Boolean(d, "parent_boundary", key.parent_boundary); return d;
}
Document Feature(const contact::RepresentedFeaturePathKey& key) {
    Document d; d.SetObject(); Integer(d, "kind", static_cast<unsigned>(key.kind));
    if (key.kind == contact::RepresentedFeatureKind::VertexFace) {
        array_json::Child(d, "vertex", Vertex(key.vertex)); array_json::Child(d, "face", Facet(key.face));
    } else if (key.kind == contact::RepresentedFeatureKind::EdgeEdge) {
        array_json::Child(d, "first_edge", Edge(key.edges[0])); array_json::Child(d, "second_edge", Edge(key.edges[1]));
    }
    return d;
}
Document Coverage(const sct::NonlinearSeparationResult& r) {
    Document d; d.SetObject();
    String(d, "status", StatusName(r.status)); Integer(d, "status_code", static_cast<unsigned>(r.status));
    Boolean(d, "certified", Certified(r.status)); Integer(d, "work", r.work); Integer(d, "deepest", r.deepest);
    Integer(d, "separated_cells", r.separated_cells); Integer(d, "covered_cells", r.covered_cells);
    Integer(d, "closed_covered_cells", r.closed_covered_cells); Integer(d, "proof_digest", r.proof_digest);
    Boolean(d, "work_exhausted", r.work_exhausted); Boolean(d, "depth_exhausted", r.depth_exhausted);
    if (r.accepted_certificate != SIZE_MAX) Integer(d, "frozen_certificate_ordinal", r.accepted_certificate);
    if (r.accepted_source_order != UINT64_MAX) Integer(d, "accepted_source_order", r.accepted_source_order);
    array_json::Child(d, "covered_feature", Feature(r.feature));
    if (r.excluded_rigid_group != UINT32_MAX) Integer(d, "excluded_rigid_group", r.excluded_rigid_group);
    Boolean(d, "has_unresolved_cell", r.has_unresolved_cell);
    if (r.has_unresolved_cell) {
        Integer(d, "unresolved_path", r.unresolved_path); Integer(d, "unresolved_depth", r.unresolved_depth);
    }
    Boolean(d, "has_intersection", r.has_intersection);
    if (r.has_intersection) {
        array_json::Child(d, "intersection_feature", Feature(r.intersection_feature));
        Integer(d, "intersection_time_numerator", r.intersection_time_numerator);
        Integer(d, "intersection_time_depth", r.intersection_time_depth);
    }
    Boolean(d, "has_contact_transition", r.has_contact_transition);
    if (r.has_contact_transition) {
        array_json::Child(d, "transition_feature", Feature(r.transition_feature));
        Integer(d, "transition_time_lower_numerator", r.transition_time_lower_numerator);
        Integer(d, "transition_time_depth", r.transition_time_depth);
        Boolean(d, "transition_time_exact", r.transition_time_exact);
        Boolean(d, "transition_zero_geometry_separated", r.transition_zero_geometry_separated);
    }
    return d;
}
}

output::Document PairDocument(const PairResult& r) {
    using namespace output;
    Document d; d.SetObject(); String(d, "schema", "robo_dyna.prepared_census_pair_replay.v1");
    String(d, "file", r.file); String(d, "family", r.family); Integer(d, "shard_pair_ordinal", r.ordinal);
    array_json::Child(d, "first_facet", Facet(r.facets[0])); array_json::Child(d, "second_facet", Facet(r.facets[1]));
    String(d, "baseline_status", StatusName(r.baseline_status)); Integer(d, "baseline_work", r.baseline_work);
    Integer(d, "baseline_depth", r.baseline_depth); Boolean(d, "root_affine", r.affine);
    String(d, "baseline_depth_scope", r.family == "failure"
        ? (r.observed_failure_baseline ? failure_detail::ObservedBaselineDepthScope
                                       : failure_detail::UnreportedBaselineDepthScope)
        : r.family == "linear" ? "unknown; zero is not a measured depth" : "captured subdivision depth");
    Integer(d, "accepted_owner_count", r.owners); Integer(d, "same_rigid_exclusion_count", r.exclusions);
    array_json::Child(d, "ledger", Coverage(r.ledger)); array_json::Child(d, "policy", Coverage(r.policy)); return d;
}
output::Document SummaryDocument(const Summary& s, const std::string& manifest_sha256) {
    using namespace output;
    Document d; d.SetObject(); String(d, "schema", "robo_dyna.prepared_census_replay_summary.v1");
    String(d, "manifest_sha256", manifest_sha256); Boolean(d, "complete", s.complete);
    String(d, "scope", "host diagnostic replay of every frozen candidate; no physical acceptance or commit");
    String(d, "witness_scope", "covered/intersection witnesses can name earlier or local cells; possible crossing is not proved penetration");
    Integer(d, "files", s.files); Integer(d, "input_bytes", s.bytes); Integer(d, "pairs", s.pairs);
    Integer(d, "linear_pairs", s.linear); Integer(d, "nonlinear_pairs", s.nonlinear);
    Integer(d, "noncertified_pairs", s.noncertified); Integer(d, "policy_work", s.total_work);
    Integer(d, "omitted_nonlinear_persistent_pairs", s.omitted_nonlinear_persistent);
    Integer(d, "omitted_linear_persistent_pairs", s.omitted_linear_persistent);
    String(d, "omission_scope", "thickness-persistent rows omitted by the source census have no continuous geometry qualification from this replay");
    Integer(d, "result_digest", s.result_digest);
    Document counts; counts.SetObject();
    for (unsigned status = 0; status < s.status_counts.size(); ++status)
        Integer(counts, StatusName(static_cast<sct::NonlinearSeparationStatus>(status)), s.status_counts[status]);
    array_json::Child(d, "status_counts", counts); return d;
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay
