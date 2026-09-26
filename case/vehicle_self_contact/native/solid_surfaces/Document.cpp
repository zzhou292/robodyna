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
    output::Require(bool(result.source) == (result.report.status == Status::Ready), "Initial source report/handle disagree");
    if (result.source) {
        const auto& value = *result.source;
        const auto& c = value.census();
        const auto& certificate = value.certificate();
        output::Integer(d, "physical_nodes", c.nodes);
        output::Integer(d, "physical_shells", c.physical_shells);
        output::Integer(d, "physical_solids", c.physical_solids);
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
        output::String(d, "source_digest", value.provenance().source_digest);
        output::String(d, "selection_digest", value.provenance().selection_digest);
        output::String(d, "input_digest", value.provenance().input_digest);
        output::String(d, "output_digest", value.provenance().output_digest);
        output::String(d, "control_rule", value.provenance().control_rule);
        output::String(d, "published_identity", "external_kind_parent_eid_pid_rawface_and_source_location");
        output::String(d, "native_reader_or_storage_ordinals", "unavailable_not_published");
        output::String(d, "membership_order", "complete_matching_family_membership_invariant");
        output::String(d, "CREATE_order", "no_consumed_element_ordinal_tie");
        output::Integer(d, "peak_reservation_bytes", value.forecast().peak_bytes);
    }
    Bound(d, cap);
    return d;
}
}
