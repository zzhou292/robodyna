#include "Report.h"

#include "output/BoundedArrayJson.h"
#include <string_view>

namespace crash::analysis::impact_response {
namespace {

using output::Require;
using output::Value;

const Value& Member(const Value& object, const char* name) {
    Require(object.IsObject(), "Expected canonical source object");
    const Value* found = nullptr;
    for (auto it = object.MemberBegin(); it != object.MemberEnd(); ++it) {
        if (std::string_view(it->name.GetString(), it->name.GetStringLength()) == name) {
            Require(!found, "Duplicate canonical source field");
            found = &it->value;
        }
    }
    Require(found, "Missing canonical source field");
    return *found;
}

std::string Text(const Value& object, const char* name) {
    return output::array_json::Text(Member(object, name));
}

std::uint64_t Unsigned(const Value& object, const char* name) {
    return output::array_json::UInt(Member(object, name));
}

std::map<std::uint64_t, std::string> PartTitles(
    const output::full_shell::source::CanonicalData& source) {
    const auto document = output::array_json::Parse(
        source.canonical_bytes, source.limits.file_bytes);
    const auto& parts = Member(document, "parts");
    Require(parts.IsArray() && parts.Size() == source.parts.size(),
        "Impact analysis source title table differs from catalog");
    std::map<std::uint64_t, std::string> result;
    for (const auto& row : parts.GetArray()) {
        const auto id = Unsigned(row, "source_part_id");
        const auto& declaration =
            output::full_shell::source::FindPart(source, id);
        const auto title = Text(row, "title");
        Require(title.size() <= 4096 &&
                Unsigned(row, "source_material_id") == declaration.material &&
                Unsigned(row, "source_section_id") == declaration.section &&
                result.emplace(id, title).second,
            "Impact analysis source part title/declaration differs");
    }
    return result;
}

}  // namespace

RunMetadata MakeRunMetadata(const output::physical_run::Replay& replay,
    const output::physical_run::ViewerInput& viewer,
    output::full_shell::RecordFile viewer_input,
    output::full_shell::RecordFile run_summary,
    output::full_shell::RecordFile connectivity_report) {
    namespace arrays = output::arrays;
    const auto& context = replay.context();
    const auto& canonical = replay.mapping().source().data();
    const auto& source = canonical.inputs;
    const auto& index = replay.index();
    Require(viewer.mapping_sha256 == replay.mapping().digest() &&
            viewer.manifest.sha256.size() == 64 &&
            viewer_input.bytes && run_summary.bytes && connectivity_report.bytes,
        "Impact analysis run authorities are incomplete");
    const output::full_shell::RecordFile* files[] = {
        &viewer_input, &viewer.manifest, &source.canonical_manifest,
        &source.scope_report, &source.source_member, &run_summary,
        &connectivity_report};
    for (const auto* file : files)
        arrays::CheckHash(file->sha256);
    Require(viewer.mapping_sha256 == context.identity().source_mapping_sha256 &&
            source.scope_report.bytes == context.identity().source_inventory_bytes &&
            source.scope_report.sha256 == context.identity().source_inventory_sha256 &&
            !index.frames.empty() &&
            output::full_shell::SameStamp(index.frames.back().stamp, index.final),
        "Impact analysis replay/source/index identities differ");

    RunMetadata result;
    result.identity = context.identity();
    result.source_units = source.units;
    result.viewer_input = std::move(viewer_input);
    result.archive_manifest = viewer.manifest;
    result.source_canonical_manifest = source.canonical_manifest;
    result.source_scope_report = source.scope_report;
    result.source_member = source.source_member;
    result.run_summary = std::move(run_summary);
    result.connectivity_report = std::move(connectivity_report);
    result.tire_policy = source.tire_policy;
    result.mapping_sha256 = viewer.mapping_sha256;
    result.source_part_titles = PartTitles(canonical);
    result.replay_peak_host_bytes = replay.peak_host_bytes();
    result.mapped_nodes = context.nodes();
    result.mapped_parents = context.parents().size();
    result.mapped_triangles = replay.mapping().triangles();
    result.stored_native_points = context.points();
    result.planned_intervals = index.planned_intervals;
    result.accepted_intervals = index.accepted_intervals;
    result.horizon_complete = index.horizon_complete;
    result.stop_reason = index.stop_reason;
    result.fixed_dt_s = context.fixed_dt();
    result.profile = replay.configuration().profile;
    return result;
}

}  // namespace crash::analysis::impact_response
