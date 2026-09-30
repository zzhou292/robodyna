#include "Internal.h"
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
namespace crash::cases::vehicle_self_contact::native::coated {
namespace {
using output::Boolean;
using output::Integer;
using output::String;
using output::array_json::Child;
output::Document Object() { output::Document value; value.SetObject(); return value; }
void Bound(const output::Document& value, std::size_t cap) {
    output::Require(cap && cap <= 1u<<20, "Invalid V5 coated report byte cap");
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    output::Require(value.Accept(writer) && buffer.GetSize() < cap, "V5 coated report exceeds byte cap");
}
output::Document LocationDocument(const Location& location) {
    auto result = Object();
    Boolean(result, "parent_available", location.source_eid != 0);
    Integer(result, "eid", location.source_eid); Integer(result, "pid", location.source_pid);
    Integer(result, "source_line", location.source_line); Integer(result, "node_id", location.source_node_id);
    if (location.canonical_row != SIZE_MAX) Integer(result, "canonical_row", location.canonical_row);
    if (location.expanded_main != SIZE_MAX) Integer(result, "expanded_main", location.expanded_main);
    if (location.edge != SIZE_MAX) Integer(result, "edge", location.edge);
    return result;
}
output::Document Counts(const RoleCounts& counts) {
    auto result = Object();
    Integer(result, "shells", counts.shells); Integer(result, "membership_pairs", counts.matches);
    Integer(result, "unmatched", counts.unmatched); Integer(result, "unique_match", counts.unique);
    Integer(result, "multiple_match", counts.multiple); Integer(result, "forward_coating", counts.forward);
    Integer(result, "reversed_coating", counts.reversed); Integer(result, "arithmetic_failures", counts.arithmetic_failures);
    return result;
}
output::Document Record(const output::full_shell::RecordFile& record) {
    output::arrays::CheckHash(record.sha256);
    output::Require(record.file.size() <= 4096, "Oversized coated source path");
    auto result = Object(); String(result, "file", record.file); String(result, "sha256", record.sha256);
    Integer(result, "bytes", record.bytes); return result;
}
output::Document Digests(const Digest& digest) {
    output::arrays::CheckHash(digest.sha256);
    output::Require(digest.fields.size() <= 32, "Too many coated digest fields");
    auto result = Object(); String(result, "sha256", digest.sha256);
    String(result, "encoding", "native_topology_fields.v1; typed little-endian scalars; 65536-word chunks; named field/type/shape framing");
    output::Value rows(rapidjson::kArrayType);
    for (const auto& field : digest.fields) {
        output::arrays::CheckHash(field.sha256);
        output::Require(field.name.size() <= 128 && field.type.size() <= 8, "Oversized coated digest label");
        auto row = Object(); String(row, "name", field.name); String(row, "type", field.type);
        String(row, "sha256", field.sha256); Integer(row, "rows", field.rows);
        Integer(row, "columns", field.columns); Integer(row, "chunks", field.chunks);
        output::Value child; child.CopyFrom(row, result.GetAllocator()); rows.PushBack(child, result.GetAllocator());
    }
    result.AddMember("fields", rows, result.GetAllocator()); return result;
}
}
output::Document ForecastDocument(const Forecast& f, std::size_t cap) {
    auto result = Object(); String(result, "schema", "robo_dyna.v5_coated_source_forecast.v1");
    Boolean(result, "admitted", f.admitted);
    String(result, "scope", "inclusive retained model reservation plus new bounded workspace; not measured RSS or physical admission");
    Integer(result, "retained_model_reservation", f.retained_model_reservation);
    Integer(result, "selection_reservation", f.selection_reservation);
    Integer(result, "source_member_reservation", f.member_reservation);
    Integer(result, "input_bytes", f.input_bytes); Integer(result, "decode_peak", f.decode_peak);
    Integer(result, "membership_scratch", f.membership_scratch); Integer(result, "ordering_scratch", f.ordering_scratch);
    Integer(result, "role_bytes", f.role_bytes); Integer(result, "result_bytes", f.result_bytes);
    Integer(result, "peak_bytes", f.peak_bytes); Integer(result, "tl_output_bytes", f.topology.output_bytes);
    Integer(result, "tl_scratch_bytes", f.topology.scratch_bytes);
    Integer(result, "tl_status", unsigned(f.topology.status));
    Integer(result, "sizeof_node", f.node_size); Integer(result, "sizeof_shell", f.shell_size);
    Integer(result, "sizeof_solid", f.solid_size); Integer(result, "sizeof_role", f.role_size);
    Bound(result, cap); return result;
}
output::Document ResultDocument(const Result& r, std::size_t cap) {
    output::Require(r.contact_parts.size() <= selection::Limits{}.parts &&
        (r.config.scope == Scope::RetainedV5PhysicalShellsAndOriginalContact ||
         r.config.scope == Scope::RetainedV6NativePhysicalShellsAndOriginalContact) &&
        r.config.coordinates == SourceCoordinates::OriginalNativeNodeCards &&
        r.config.order == NativeOrder::CaseDeclaredAscendingPhysicalNidItab &&
        r.config.membership == SurfaceMembership::SingleSurfaceImbinZero &&
        (!r.topology_complete || (r.selected_roles_complete && r.topology_attempted && r.topology_report.status == s::Status::Ok)),
        "Inconsistent V5 coated result scope or completion");
    auto result = Object();
    const bool native=r.config.scope==Scope::RetainedV6NativePhysicalShellsAndOriginalContact;
    String(result, "schema", native?"robo_dyna.v6_coated_source_assessment.v1":"robo_dyna.v5_coated_source_assessment.v1");
    String(result, "scope", native?"retained V6 raw8 native physical shells/solids; source assessment only; no runtime admission":
        "all retained V5 physical shells/solids; original-contact shell topology only; no runtime, solid-face or full-original case admission");
    String(result, "reader_packet", native?"declared V6 raw8 H8 before INITIA; original source-native coordinates":
        "declared V5 H8/PENTA6 before INITIA; original source-native coordinates");
    String(result, "native_order", "case-declared ascending physical-NID ITAB; unsigned six-word surface order; IMBIN=0");
    String(result, "limitation", "not proof of original full-case internal numbering, converter choices, or omitted original families");
    Boolean(result, "all_physical_roles_complete", r.physical_roles_complete);
    Boolean(result, "selected_contact_roles_complete", r.selected_roles_complete); Boolean(result, "topology_attempted", r.topology_attempted);
    Boolean(result, "topology_complete", r.topology_complete);
    Boolean(result, "deletion_retry", false); Boolean(result, "runtime_admitted", false);
    Integer(result, "first_physical_role_status", unsigned(r.role_status));
    Integer(result, "first_selected_role_status", unsigned(r.selected_role_status));
    if (r.topology_attempted) Integer(result, "topology_status", unsigned(r.topology_report.status));
    else result.AddMember("topology_status", output::Value(), result.GetAllocator());
    Child(result, "physical_shells", Counts(r.physical)); Child(result, "contact_shells", Counts(r.contact));
    Integer(result, "physical_nodes", r.physical_nodes); Integer(result, "physical_solids", r.physical_solids);
    Integer(result, "original_canonical_solids", r.original_solids);
    output::Require(r.original_solids >= r.physical_solids, "Invalid retained solid disposition");
    Integer(result, "omitted_original_solids_not_used", r.original_solids-r.physical_solids);
    auto families = Object();
    const char* names[]{"solid18_law36", "solid24_law42", "solid6z_law42", "solid18_law44", "solid18_law90"};
    for (unsigned i = 0; i < 5; ++i) Integer(families, names[i], r.solid_families[i]);
    Child(result, "retained_solid_families", families);
    Integer(result, "candidate_visits", r.candidate_visits);
    Integer(result, "source_coordinate_roundtrip_changed_components", r.coordinate_roundtrip_changes);
    Integer(result, "output_mains", r.output_mains); Integer(result, "output_references", r.output_references);
    Integer(result, "output_incidences", r.output_incidences);
    Child(result, "first_physical_unready", LocationDocument(r.first_unready));
    Child(result, "first_selected_unready", LocationDocument(r.first_selected_unready)); Child(result, "failure", LocationDocument(r.failure));
    Integer(result, "native_neighbor_warning_count", r.topology_report.neighbor_warnings.count);
    Child(result, "first_native_warning", LocationDocument(r.first_warning));
    auto provenance = Object(); Child(provenance, "canonical", Record(r.provenance.canonical_manifest));
    Child(provenance, "scope", Record(r.provenance.scope_report)); Child(provenance, "member", Record(r.provenance.source_member));
    String(provenance, "auxiliary_sha256", r.provenance.auxiliary_sha256);
    String(provenance, "combine_sha256", r.provenance.combine_sha256); Child(result, "provenance", provenance);
    output::Value parts(rapidjson::kArrayType);
    for (const auto& part : r.contact_parts) {
        auto row = Object(); Integer(row, "pid", part.part_id); Integer(row, "mid", part.material_id); Integer(row, "sid", part.section_id);
        Boolean(row, "retained_shell_part", part.retained_shell_part); Boolean(row, "excluded_shell_part", part.excluded_shell_part);
        Integer(row, "shells", part.shells); Integer(row, "solid_primitives_not_assessed", part.solids); Integer(row, "beam_primitives_not_assessed", part.beams);
        output::Value child; child.CopyFrom(row, result.GetAllocator()); parts.PushBack(child, result.GetAllocator());
    }
    result.AddMember("original_contact_part_dispositions", parts, result.GetAllocator());
    Child(result, "forecast", ForecastDocument(r.forecast, cap)); Child(result, "input_digest", Digests(r.input_digest));
    if (r.topology_complete) Child(result, "output_digest", Digests(r.output_digest));
    else result.AddMember("output_digest", output::Value(), result.GetAllocator());
    Bound(result, cap); return result;
}
} // namespace crash::cases::vehicle_self_contact::native::coated
