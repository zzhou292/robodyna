#include "Internal.h"
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
namespace crash::cases::vehicle_self_contact::native::initial_surfaces {
namespace {
const char* Name(Status status) {
    switch (status) {
    case Status::Ready: return "ready_initial_source";
    case Status::InvalidInput: return "invalid_input";
    case Status::UnsupportedSource: return "unsupported_source";
    case Status::ResourceLimit: return "resource_limit";
    case Status::NeedsNativeReaderOrder: return "needs_native_reader_order";
    }
    return "unknown";
}
const char* Name(NumericalStage stage) {
    switch (stage) {
    case NumericalStage::None: return "none";
    case NumericalStage::ProbePreflight: return "solid_probe_preflight";
    case NumericalStage::PartPreflight: return "part_preflight";
    case NumericalStage::ProbeBuild: return "solid_probe_build";
    case NumericalStage::PartBuild: return "part_build";
    }
    return "unknown";
}
const char* Name(values::Status status) {
    switch (status) {
    case values::Status::Ok: return "ok";
    case values::Status::InvalidInput: return "invalid_input";
    case values::Status::UnsupportedProfile: return "unsupported_profile";
    case values::Status::ResourceLimit: return "resource_limit";
    case values::Status::UnsupportedArithmetic: return "unsupported_arithmetic";
    }
    return "unknown";
}
void Bound(const output::Document& document, std::size_t cap) {
    output::Require(cap && cap <= 1u<<20, "Invalid initial source report cap");
    rapidjson::StringBuffer bytes;
    rapidjson::Writer<rapidjson::StringBuffer> writer(bytes);
    output::Require(document.Accept(writer) && bytes.GetSize() <= cap, "Initial source report exceeds metadata cap");
}
}
output::Document ForecastDocument(const Forecast& f) {
    output::Document d;
    d.SetObject();
    output::String(d, "schema", "robo_dyna.initial_surface_forecast.v1");
    output::String(d, "scope", "inclusive source and public arena reservations; not measured RSS");
    output::Integer(d, "upstream_input_reservation", f.upstream_geometry_reservation);
    output::Integer(d, "corrected_context_reservation", f.context_reservation);
    output::Integer(d, "packed_inputs", f.packed_inputs);
    output::Integer(d, "certificate_workspace", f.certificate_workspace);
    output::Integer(d, "extraction_output_ceiling", f.extraction_output);
    output::Integer(d, "extraction_scratch_ceiling", f.extraction_scratch);
    output::Integer(d, "retained_faces", f.retained_faces);
    output::Integer(d, "origin_group_bytes", f.origin_group_bytes);
    output::Integer(d, "digest_workspace", f.digest_workspace);
    output::Integer(d, "maximum_faces", f.maximum_faces);
    output::Integer(d, "peak_reservation_bytes", f.peak_bytes);
    return d;
}
output::Document ResultDocument(const Preparation& result, std::size_t cap) {
    output::Document d;
    d.SetObject();
    output::String(d, "schema", "robo_dyna.initial_source_surfaces.v1");
    output::String(d, "status", Name(result.report.status));
    output::String(d, "reason", result.report.reason);
    output::String(d, "scope", "declared V5 initial clause surfaces before I25SURFI classification/filter/SH2; no runtime admission");
    output::Integer(d, "diagnostic_solid_eid", result.report.solid_element);
    output::Integer(d, "diagnostic_solid_face", result.report.solid_face);
    output::Integer(d, "diagnostic_first_eid", result.report.first_candidate_element);
    output::Integer(d, "diagnostic_conflicting_eid", result.report.conflicting_candidate_element);
    if (result.report.numerical_stage != NumericalStage::None) {
        const auto& numerical = result.report.numerical;
        output::String(d, "numerical_stage", Name(result.report.numerical_stage));
        output::String(d, "numerical_status", Name(numerical.status));
        output::String(d, "numerical_row_meaning", "representative table index; family and native reader order not inferred");
        if (numerical.row != SIZE_MAX) output::Integer(d, "representative_row", numerical.row);
        if (numerical.node != SIZE_MAX) output::Integer(d, "physical_node", numerical.node);
        output::Integer(d, "required_faces", numerical.required_faces);
        output::Boolean(d, "count_complete", numerical.count_complete);
    }
    output::Require(bool(result.source) == (result.report.status == Status::Ready), "Initial source report/handle disagree");
    if (result.source) {
        const auto& value = *result.source;
        const auto& c = value.census();
        const auto& certificate = value.certificate();
        output::Integer(d, "physical_nodes", c.nodes);
        output::Integer(d, "physical_shells", c.physical_shells);
        output::Integer(d, "physical_solids", c.physical_solids);
        output::Integer(d, "reader_bricks", c.reader_bricks);
        output::Integer(d, "native_raw8_bricks", c.native_raw8_bricks);
        output::Integer(d, "declared_penta", c.declared_penta);
        output::Integer(d, "original_selected_solids", c.original_selected_solids);
        output::Integer(d, "retained_selected_solids", c.extraction.selected_solids);
        output::Integer(d, "omitted_selected_solids", c.omitted_selected_solids);
        output::Integer(d, "selected_quads", c.extraction.selected_quads);
        output::Integer(d, "selected_triangles", c.extraction.selected_triangles);
        output::Integer(d, "solid_faces", c.extraction.solid_faces);
        output::Integer(d, "shell_faces", c.extraction.shell_faces);
        output::Integer(d, "faces", c.faces);
        output::Integer(d, "quad_faces", c.quad_faces);
        output::Integer(d, "triangle_faces", c.triangle_faces);
        output::Integer(d, "internal_faces", c.extraction.internal_faces);
        output::Integer(d, "degenerate_faces", c.extraction.degenerate_faces);
        output::Integer(d, "shell_suppressed_faces", c.extraction.shell_suppressed_faces);
        output::Integer(d, "queried_solid_faces", certificate.queried_solid_faces);
        output::Integer(d, "matching_physical_shells", certificate.matching_physical_shells);
        output::Integer(d, "equal_node_key_groups", certificate.equal_node_key_groups);
        output::Integer(d, "differing_origin_groups", certificate.differing_origin_groups);
        output::Integer(d, "retained_origin_groups", value.origin_groups().size());
        output::String(d, "source_digest", value.provenance().source_digest);
        output::String(d, "selection_digest", value.provenance().selection_digest);
        output::String(d, "input_digest", value.provenance().input_digest);
        output::String(d, "output_digest", value.provenance().output_digest);
        output::String(d, "control_rule", value.provenance().control_rule);
        output::String(d, "published_identity", "external_kind_parent_eid_pid_rawface_and_source_location");
        output::String(d, "native_reader_or_storage_ordinals", "unavailable_not_published");
        output::String(d, "membership_order", "complete_matching_family_membership_invariant");
        output::String(d, "CREATE_order", "consumed_node_role_words_invariant_all_origins_retained");
        output::String(d, "raw_origin_order", "unavailable_within_typed_equivalence_groups");
        output::String(d, "output_digest_scope", "complete_representative_origin_roster_not_native_ELEM_order");
        output::Integer(d, "peak_reservation_bytes", value.forecast().peak_bytes);
    }
    Bound(d, cap);
    return d;
}
}
