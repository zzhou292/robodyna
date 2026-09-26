#include "Internal.h"
#include "output/BoundedArrayJson.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::mixed_interface {
namespace {
const char* Name(Status status) {
    switch (status) {
    case Status::Ready: return "ready_classified_sides_before_support";
    case Status::InvalidInput: return "invalid_input";
    case Status::UnsupportedSource: return "unsupported_source";
    case Status::ResourceLimit: return "resource_limit";
    case Status::NeedsNativeReaderOrder: return "needs_native_reader_order";
    }
    return "unknown";
}
void Bound(const output::Document& document, std::size_t cap) {
    output::Require(cap && cap <= 1u<<20, "Invalid mixed source report cap");
    rapidjson::StringBuffer bytes;
    rapidjson::Writer<rapidjson::StringBuffer> writer(bytes);
    output::Require(document.Accept(writer) && bytes.GetSize() <= cap, "Mixed source report exceeds metadata cap");
}
}
output::Document ForecastDocument(const Forecast& f) {
    output::Document doc;
    doc.SetObject();
    output::String(doc, "schema", "robo_dyna.mixed_interface_forecast.v1");
    output::String(doc, "scope", "inclusive source and numerical public ceiling reservations; not measured RSS");
    output::Integer(doc, "initial_source_reservation", f.initial_source_reservation);
    output::Integer(doc, "packed_inputs", f.packed_inputs);
    output::Integer(doc, "source_role_workspace", f.source_role_workspace);
    output::Integer(doc, "interface_output_ceiling", f.interface_output);
    output::Integer(doc, "shared_scratch_ceiling", f.shared_scratch);
    output::Integer(doc, "sides_output_upper_bound", f.sides_output);
    output::Integer(doc, "digest_and_report", f.digest_and_report);
    output::Integer(doc, "peak_reservation_bytes", f.peak_bytes);
    return doc;
}
output::Document ResultDocument(const Preparation& result, std::size_t cap) {
    output::Document doc;
    doc.SetObject();
    output::String(doc, "schema", "robo_dyna.mixed_interface_source.v1");
    output::String(doc, "status", Name(result.report.status));
    output::String(doc, "reason", result.report.reason);
    output::String(doc, "scope", "IN24 classification, I25SURFI filter and SH2 sides only; no support, gap, normals or runtime admission");
    if (result.report.raw_face != SIZE_MAX) output::Integer(doc, "diagnostic_raw_face", result.report.raw_face);
    output::Integer(doc, "diagnostic_source_eid", result.report.source_element);
    output::Integer(doc, "numerical_stage", std::uint64_t(result.report.numerical_stage));
    if (result.report.numerical_stage == NumericalStage::InterfacePreflight ||
        result.report.numerical_stage == NumericalStage::InterfaceBuild) {
        const auto& r = result.report.interface_report;
        output::Integer(doc, "interface_status", std::uint64_t(r.status));
        if (r.physical_solid != SIZE_MAX) output::Integer(doc, "representative_physical_solid_row", r.physical_solid);
        if (r.context_row != SIZE_MAX) output::Integer(doc, "representative_context_row", r.context_row);
        if (r.node != SIZE_MAX) output::Integer(doc, "physical_node", r.node);
    } else if (result.report.numerical_stage != NumericalStage::None) {
        const auto& r = result.report.sides_report;
        output::Integer(doc, "sides_status", std::uint64_t(r.status));
        if (r.primary != SIZE_MAX) output::Integer(doc, "primary_ordinal", r.primary);
        if (r.node != SIZE_MAX) output::Integer(doc, "physical_node", r.node);
    }
    output::Require(bool(result.source) == (result.report.status == Status::Ready), "Mixed source report/handle disagree");
    if (result.source) {
        const auto& value = *result.source;
        const auto& sides = value.sides();
        const auto& c = value.certificate();
        output::String(doc, "source_digest", value.provenance().source_digest);
        output::String(doc, "initial_digest", value.provenance().initial_digest);
        output::String(doc, "output_digest", value.provenance().output_digest);
        output::String(doc, "raw_origin_order", "unavailable within initial typed equivalence groups; every origin retained");
        output::String(doc, "support_owner", "unavailable; IN24 witness is not IELEM_M");
        output::String(doc, "normal_fields", "unavailable; sides Main neighbor fields are API zero only");
        output::Integer(doc, "nodes", sides.node_count);
        output::Integer(doc, "raw_origins", sides.raw_origin_count);
        output::Integer(doc, "raw_shell_faces", c.raw_shells);
        output::Integer(doc, "raw_solid_faces", c.raw_solids);
        output::Integer(doc, "unique_coatings", c.unique_coatings);
        output::Integer(doc, "ordinary_shells", c.ordinary_shells);
        output::Integer(doc, "primary_count", sides.primary_count);
        output::Integer(doc, "shell_primary_count", sides.shell_primary_count);
        output::Integer(doc, "solid_primary_count", c.solid_primaries);
        output::Integer(doc, "main_count", sides.main_count);
        output::Integer(doc, "multi_origin_primaries", c.multi_origin_primaries);
        output::Integer(doc, "coalesced_origins", c.coalesced_origins);
        output::Boolean(doc, "complete_origins", c.complete_origins);
        output::Boolean(doc, "complete_solid_flags", c.complete_solid_flags);
        output::Integer(doc, "peak_reservation_bytes", value.forecast().peak_bytes);
        const auto& a = value.admission_census();
        output::String(doc, "census_scope", "same retained physical context; source policies and raw physical ledger, not runtime admission");
        output::Integer(doc, "positive_nodal_mass", a.positive_mass);
        output::Integer(doc, "zero_nodal_mass", a.zero_mass);
        output::Integer(doc, "negative_nodal_mass", a.negative_mass);
        output::Integer(doc, "nonfinite_nodal_mass", a.nonfinite_mass);
        output::Boolean(doc, "actual_rigid_binding_available", a.actual_rigid_binding_available);
        output::Integer(doc, "actual_rigid_groups", a.rigid_groups);
        output::Integer(doc, "actual_rigid_members", a.rigid_members);
        output::Integer(doc, "nonpositive_mass_rigid_members", a.nonpositive_rigid_members);
        output::Integer(doc, "tied_source_candidate_nodes", a.tied_source_candidates);
        output::Integer(doc, "nonpositive_mass_tied_candidates", a.nonpositive_tied_candidates);
        output::Boolean(doc, "finalized_cin_available", a.finalized_cin_available);
        output::String(doc, "tied_candidate_meaning", "retained source slave membership only, not PRE_I2 classification or finalized CIN");
        rapidjson::Value examples(rapidjson::kArrayType);
        for (std::size_t i = 0; i < a.mass_example_count; ++i) {
            const auto& v = a.mass_examples[i];
            output::Document row;
            row.SetObject();
            output::Integer(row, "source_node_id", v.source_node_id);
            output::Integer(row, "domain_node", v.domain_node);
            output::Integer(row, "raw_mass_bits", output::Bits(v.raw_mass_kg));
            if (std::isfinite(v.raw_mass_kg)) output::Number(row, "raw_mass_kg", v.raw_mass_kg);
            output::Boolean(row, "rigid_member", v.rigid_member);
            output::Boolean(row, "tied_source_candidate", v.tied_source_candidate);
            rapidjson::Value copy;
            copy.CopyFrom(row, doc.GetAllocator());
            examples.PushBack(copy, doc.GetAllocator());
        }
        doc.AddMember("nonpositive_mass_examples", examples, doc.GetAllocator());
        const char* families[]{"qeph", "t3", "qbat"};
        const char* policies[]{"none", "constant_all_points", "tab1_any_point"};
        for (unsigned family = 0; family < 3; ++family)
            for (unsigned policy = 0; policy < 3; ++policy)
                output::Integer(doc, (std::string("source_failure_") + families[family] + "_" + policies[policy]).c_str(),
                    a.shell_failure_policies[family][policy]);
    }
    Bound(doc, cap);
    return doc;
}
}
