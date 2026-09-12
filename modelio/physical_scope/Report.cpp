#include "PhysicalScope.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::modelio::physical_scope {
std::string ReportJson(const PhysicalScope& scope) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    const auto number = [&](const char* key, std::size_t value) { writer.Key(key); writer.Uint64(value); };
    const auto text = [&](const char* key, const std::string& value) {
        writer.Key(key); writer.String(value.data(), value.size());
    };
    const auto state = [](Coverage value) {
        return value == Coverage::Complete ? "complete" : value == Coverage::Partial ? "partial" : "none";
    };
    const auto& data = scope.data();
    const auto& canonical = scope.tied_source().canonical().data();
    writer.StartObject();
    text("schema", scope.structural_beam_source() ? "robo_dyna.original_physical_source_coverage.v2" :
        "robo_dyna.original_physical_source_coverage.v1");
    text("scope", "Source membership only; no complete mass ledger, DOF or runtime admission");
    text("archive_sha256", canonical.archive_sha256);
    text("member_sha256", canonical.inputs.source_member.sha256);
    text("canonical_sha256", canonical.inputs.canonical_manifest.sha256);
    text("tire_policy", canonical.inputs.tire_policy);
    text("type25_disposition", "All literal endpoints are provisional source coverage; no coefficient producer admitted");
    writer.Key("role_bits"); writer.StartObject();
    number("shell", Shell); number("type13_endpoint", Type13Endpoint); number("solid", Solid);
    number("retained_point_mass", RetainedPointMass); number("provisional_type25", ProvisionalType25);
    number("excluded_tire_shell", ExcludedTireShell); number("beam_orientation", BeamOrientation);
    if(scope.structural_beam_source()) number("beam18_endpoint",Beam18Endpoint);
    writer.EndObject();
    writer.Key("excluded_tire_part_ids"); writer.StartArray();
    for (auto id : canonical.excluded_parts) writer.Uint64(id);
    writer.EndArray();
    writer.Key("counts"); writer.StartObject();
    const auto& count = data.counts;
    number("baseline_nodes", count.baseline_nodes); number("with_type25_nodes", count.with_type25_nodes);
    number("type25_added_nodes", count.type25_added_nodes); number("type25_connections", data.spotwelds.size());
    number("type25_optional_cards", count.optional_spotwelds);
    number("plain_groups", data.plain_groups.size()); number("part_roots", data.part_roots.size());
    number("plain_complete_before", count.plain_complete_before); number("plain_complete_after", count.plain_complete_after);
    number("roots_complete_before", count.roots_complete_before); number("roots_complete_after", count.roots_complete_after);
    number("all_point_mass_records", data.point_masses.size());
    number("rigid_skin_parents", data.rigid_skin.size());
    number("retained_point_mass_records", scope.point_mass_source().data().consumed.size());
    number("outside_mass_in_baseline", count.outside_mass_in_baseline);
    number("outside_mass_with_type25", count.outside_mass_with_type25);
    writer.Key("role_node_counts"); writer.StartArray();
    for (unsigned bit=0;bit<(scope.structural_beam_source()?8u:7u);++bit) writer.Uint64(count.role_nodes[bit]);
    writer.EndArray(); writer.EndObject();
    writer.Key("budget"); writer.StartObject();
    number("inclusive_rigid_source_reservation", scope.forecast().source_reservation);
    number("additional_retained_payload", scope.forecast().additional_retained);
    number("workspace_reservation", scope.forecast().workspace);
    number("result_reservation", scope.forecast().result_reservation);
    number("complete_startup_reservation", scope.forecast().total_bytes);
    number("owned_result_payload", data.owned_payload_bytes);
    writer.EndObject();
    const auto groups = [&](const char* key, const auto& rows) {
        writer.Key(key); writer.StartArray();
        for (const auto& group : rows) {
            writer.StartObject();
            number("id", group.id); number("node_set_id", group.node_set_id);
            number("child_part_id", group.child_part_id); number("source_index", group.source_index);
            text("before", state(group.before)); text("after", state(group.after));
            number("covered_before", group.covered_before); number("covered_after", group.covered_after);
            number("tire_members", group.tire_members);
            writer.Key("members"); writer.StartArray();
            for (const auto& member : group.members) {
                writer.StartArray(); writer.Uint64(member.node); writer.Uint(member.roles); writer.EndArray();
            }
            writer.EndArray(); writer.EndObject();
        }
        writer.EndArray();
    };
    groups("plain_groups", data.plain_groups);
    groups("part_roots", data.part_roots);
    writer.Key("rigid_skin"); writer.StartArray();
    for (const auto& row : data.rigid_skin) {
        writer.StartArray(); writer.Uint64(row.element); writer.Uint64(row.part);
        writer.Uint64(row.canonical_row); writer.Uint64(row.root_index); writer.EndArray();
    }
    writer.EndArray();
    writer.Key("spotwelds"); writer.StartArray();
    for (const auto& row : data.spotwelds) {
        writer.StartObject(); number("id", row.id); number("source_index", row.source_index);
        number("first_card", row.first_card);
        writer.Key("default_only"); writer.Bool(row.default_only);
        writer.Key("nodes"); writer.StartArray();
        for (auto id : row.nodes) writer.Uint64(id);
        writer.EndArray(); writer.EndObject();
    }
    writer.EndArray();
    writer.Key("point_masses"); writer.StartArray();
    for (const auto& mass : data.point_masses) {
        writer.StartArray(); writer.Uint64(mass.element); writer.Uint64(mass.node);
        writer.Uint64(mass.source_record);
        if (mass.root_index == SIZE_MAX) writer.Null(); else writer.Uint64(mass.root_index);
        writer.Uint(mass.roles); writer.EndArray();
    }
    writer.EndArray();
    text("incidence_columns", "family,source_EID,source_PID,canonical_row,local_slot_mask,selected,orientation_only");
    writer.Key("node_evidence"); writer.StartArray();
    for (const auto& node : data.evidence) {
        writer.StartObject(); number("node", node.node); number("roles", node.roles);
        writer.Key("incidence"); writer.StartArray();
        for (const auto& entry : node.incidence) {
            writer.StartArray(); writer.String(entry.family == Family::Shell ? "shell" :
                entry.family == Family::Beam ? "beam" : "solid");
            writer.Uint64(entry.element); writer.Uint64(entry.part); writer.Uint(entry.canonical_row);
            writer.Uint(entry.local_slots); writer.Bool(entry.selected); writer.Bool(entry.orientation_only);
            writer.EndArray();
        }
        writer.EndArray(); writer.EndObject();
    }
    writer.EndArray(); writer.EndObject();
    output::Require(buffer.GetSize() <= 32 * 1024 * 1024, "Physical census report exceeds separate serialization cap");
    return {buffer.GetString(), buffer.GetSize()};
}
} // namespace crash::modelio::physical_scope
