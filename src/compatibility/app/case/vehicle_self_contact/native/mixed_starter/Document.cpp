#include "Internal.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
namespace crash::cases::vehicle_self_contact::native::mixed_starter {
namespace {
const char* Name(Status status) {
    switch (status) {
    case Status::Ready: return "ready_starter_topology_before_initial_history";
    case Status::InvalidInput: return "invalid_input";
    case Status::ResourceLimit: return "resource_limit";
    case Status::UnsupportedSource: return "unsupported_source";
    }
    return "unknown";
}
void Bound(const output::Document& doc, std::size_t cap) {
    output::Require(cap && cap <= 1u<<20, "Invalid mixed Starter metadata cap");
    rapidjson::StringBuffer bytes;
    rapidjson::Writer<rapidjson::StringBuffer> writer(bytes);
    output::Require(doc.Accept(writer) && bytes.GetSize() <= cap, "Mixed Starter report exceeds metadata cap");
}
}
output::Document ForecastDocument(const Forecast& value) {
    output::Document doc;
    doc.SetObject();
    output::String(doc, "schema", "robo_dyna.mixed_starter_forecast.v1");
    output::String(doc, "scope", "peak reservation across actual sequential source phases; not measured RSS");
    output::Integer(doc, "prior_construction_peak", value.prior_construction_peak);
    output::Integer(doc, "upstream_retained", value.upstream_retained);
    output::Integer(doc, "embedding_additional", value.embedding_additional);
    output::Integer(doc, "embedding_prior_peak", value.embedding_prior_peak);
    output::Integer(doc, "combined_inputs", value.combined_inputs);
    output::Integer(doc, "starter_output", value.starter_output);
    output::Integer(doc, "starter_scratch", value.starter_scratch);
    output::Integer(doc, "normal_source_validation", value.normal_source_validation);
    output::Integer(doc, "retained_metadata", value.retained_metadata);
    output::Integer(doc, "digest_and_report", value.digest_and_report);
    output::Integer(doc, "current_phase", value.current_phase);
    output::Integer(doc, "retained_bytes", value.retained_bytes);
    output::Integer(doc, "peak_bytes", value.peak_bytes);
    return doc;
}
output::Document ResultDocument(const Preparation& result, std::size_t cap) {
    output::Document doc;
    doc.SetObject();
    output::String(doc, "schema", "robo_dyna.mixed_starter_source.v1");
    output::String(doc, "status", Name(result.report.status));
    output::String(doc, "reason", result.report.reason);
    output::String(doc, "scope", "native mixed topology and Starter I25NORM cache; no initial history or runtime admission");
    output::Integer(doc, "numerical_stage", std::uint64_t(result.report.numerical_stage));
    if (result.report.numerical_stage == NumericalStage::Forecast || result.report.numerical_stage == NumericalStage::Build) {
        const auto& r = result.report.startup_report;
        output::Integer(doc, "startup_status", std::uint64_t(r.status));
        if (r.primary != SIZE_MAX) output::Integer(doc, "primary_ordinal", r.primary);
        if (r.node != SIZE_MAX) output::Integer(doc, "domain_node", r.node);
    } else if (result.report.numerical_stage == NumericalStage::NormalSourceAdmission) {
        const auto& r = result.report.normal_source_report;
        output::Integer(doc, "normal_source_status", std::uint64_t(r.status));
        if (r.main != SIZE_MAX) output::Integer(doc, "main_ordinal", r.main);
        if (r.reference != SIZE_MAX) output::Integer(doc, "reference_ordinal", r.reference);
        if (r.node != SIZE_MAX) output::Integer(doc, "domain_node", r.node);
    }
    output::Require(bool(result.source) == (result.report.status == Status::Ready), "Mixed Starter report/source disagree");
    if (result.source) {
        const auto& value = *result.source;
        const auto& snapshot = value.snapshot();
        output::String(doc, "source_digest", value.provenance().source_digest);
        output::String(doc, "post_gapm_digest", value.provenance().post_gapm_digest);
        output::String(doc, "output_digest", value.provenance().output_digest);
        output::Integer(doc, "nodes", snapshot.node_count);
        output::Integer(doc, "primary_count", snapshot.primary_count);
        output::Integer(doc, "shell_primary_count", snapshot.shell_primary_count);
        output::Integer(doc, "main_count", snapshot.main_count);
        output::Integer(doc, "raw_origins", snapshot.raw_origin_count);
        output::Integer(doc, "normal_references", snapshot.starter.reference_count);
        output::Integer(doc, "normal_incidence", snapshot.normal_incidence_count);
        output::Integer(doc, "pre_shell_internal_count", snapshot.post_gapm->pre_shell_internal_count);
        output::Integer(doc, "native_neighbor_warnings", value.neighbor_warnings().count);
        output::Boolean(doc, "solid_erosion_enabled", snapshot.post_gapm->final_solid_erosion == s::SolidErosion::Enabled);
        output::Boolean(doc, "engine_activation_applied", false);
        output::Boolean(doc, "combined_domain_embedding", value.embedding() != nullptr);
        output::Integer(doc, "retained_bytes", value.forecast().retained_bytes);
        output::Integer(doc, "peak_bytes", value.forecast().peak_bytes);
    }
    Bound(doc, cap);
    return doc;
}
}
