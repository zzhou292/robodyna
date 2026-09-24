#include "SelfContactDocument.h"
#include "SelfContactErrorGeometry.h"
#include "output/BoundedArrayJson.h"
#include "case/vehicle_self_contact/SelfContactStageError.h"

namespace crash::cases::vehicle_run::detail {

namespace {

void AppendCrossingWork(output::Document& document,
                        const tlfea::contact::SelfContactCrossingDiagnostics& value) {
    if (!value.available) return;
    using namespace output;
    Document native;
    native.SetObject();
    String(native, "schema", "robo_dyna.native_crossing_failure.v1");
    String(native, "scope", "Failed native call; work is the admitted canonical prefix, not total chunk work or a geometry certificate");
    Integer(native, "batch_pair_offset", value.batch_pair_offset);
    Integer(native, "prior_batch_work", value.prior_batch_work);
    if (value.input_path != SIZE_MAX) Integer(native, "input_path", value.input_path);
    if (value.input_pair != SIZE_MAX) Integer(native, "input_pair", value.input_pair);
    Integer(native, "input_paths", value.input_paths);
    Integer(native, "input_pairs", value.input_pairs);
    Integer(native, "unique_pairs", value.unique_pairs);
    Integer(native, "certified_separated", value.certified_separated);
    Integer(native, "certified_crossing_contact", value.certified_crossing_contact);
    Integer(native, "unresolved", value.unresolved);
    Integer(native, "admitted_work", value.admitted_work);
    if (value.total_work_limit) {
        Integer(native, "total_work_limit", value.total_work_limit);
        Integer(native, "rejected_pair_work", value.rejected_pair_work);
    }
    array_json::Child(document, "native_crossing_failure", native);
}

}  // namespace

output::Document SelfContactErrorDocument(
    const vehicle_self_contact::SelfContactStageError& error) {
    using namespace output;
    using Count = tlfea::contact::SelfContactTransactionCountKind;
    const auto& report = error.report();
    Document document;
    document.SetObject();
    String(document, "schema", "robo_dyna.self_contact_stage_error.v2");
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
    if (report.filter_status != tlfea::contact::self_contact_filters::Status::Ok)
        Integer(document, "filter_status_code", static_cast<unsigned>(report.filter_status));
    if (report.filter_scope != tlfea::contact::SelfContactFacetFilterFailureScope::None) {
        String(document, "filter_failure_scope",
            report.filter_scope == tlfea::contact::SelfContactFacetFilterFailureScope::CandidateChunkBeforeFold
                ? "candidate_chunk_before_serial_fold" : "unknown");
        if (report.filter_chunk_begin != SIZE_MAX)
            Integer(document, "filter_chunk_begin", report.filter_chunk_begin);
        Integer(document, "filter_chunk_pairs", report.filter_chunk_pairs);
    }
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
    AppendCrossingWork(document, report.crossing_diagnostics);
    AppendSelfContactErrorGeometry(document, report);
    return document;
}


}  // namespace crash::cases::vehicle_run::detail
