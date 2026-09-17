#include "Report.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace crash::analysis::impact_response {
namespace {

using Allocator = output::Document::AllocatorType;
using output::Require;
using output::Value;

Value Object() {
    return Value(rapidjson::kObjectType);
}

Value Array() {
    return Value(rapidjson::kArrayType);
}

void Add(Value& object, Allocator& allocator, const char* name, Value value) {
    object.AddMember(Value(name, allocator), value, allocator);
}

void String(Value& object, Allocator& allocator, const char* name,
    const std::string& text) {
    Add(object, allocator, name,
        Value(text.c_str(), static_cast<rapidjson::SizeType>(text.size()), allocator));
}

void Number(Value& object, Allocator& allocator, const char* name, double value) {
    Require(std::isfinite(value), "Nonfinite impact analysis report value");
    Add(object, allocator, name, Value(value));
}

void Integer(Value& object, Allocator& allocator, const char* name,
    std::uint64_t value) {
    Value number;
    number.SetUint64(value);
    Add(object, allocator, name, std::move(number));
}

void Boolean(Value& object, Allocator& allocator, const char* name, bool value) {
    Add(object, allocator, name, Value(value));
}

Value File(const output::full_shell::RecordFile& file, Allocator& allocator) {
    Value value = Object();
    String(value, allocator, "file", file.file);
    String(value, allocator, "sha256", file.sha256);
    Integer(value, allocator, "bytes", file.bytes);
    return value;
}

Value Triple(const std::array<double, 3>& values, Allocator& allocator) {
    Value array = Array();
    for (const auto value : values) {
        Require(std::isfinite(value), "Nonfinite impact analysis coordinate");
        array.PushBack(value, allocator);
    }
    return array;
}

Value UnsignedArray(const std::vector<std::uint64_t>& values,
    Allocator& allocator) {
    Value array = Array();
    for (const auto value : values) {
        Value number;
        number.SetUint64(value);
        array.PushBack(number, allocator);
    }
    return array;
}

Value Stamp(const SampleStamp& stamp, Allocator& allocator) {
    Value value = Object();
    Integer(value, allocator, "epoch", stamp.epoch);
    Integer(value, allocator, "attempt", stamp.attempt);
    Number(value, allocator, "time_s", stamp.time_s);
    return value;
}

Value OccurrenceDocument(const Occurrence& occurrence, Allocator& allocator) {
    Require(occurrence.available, "Missing impact analysis occurrence");
    Value value = Object();
    Add(value, allocator, "sample", Stamp(occurrence.stamp, allocator));
    Boolean(value, allocator, "parent_active", occurrence.parent_active);
    Add(value, allocator, "parent_centroid_m",
        Triple(occurrence.centroid_m, allocator));
    return value;
}

Value WitnessDocument(const Witness& witness,
    const PlasticityResult& result, Allocator& allocator) {
    Require(witness.available && witness.parent_index < result.catalog.size(),
        "Invalid impact analysis witness");
    const auto& source = result.catalog[witness.parent_index];
    Value value = OccurrenceDocument(witness.occurrence, allocator);
    Integer(value, allocator, "source_element_id", source.source_element);
    Integer(value, allocator, "source_part_id", source.source_part);
    Number(value, allocator, "maximum_equivalent_plastic_strain",
        witness.value);
    return value;
}

Value BoundsDocument(const SpatialBounds& bounds, Allocator& allocator) {
    Require(bounds.available, "Missing impact analysis bounds");
    Value value = Object();
    Add(value, allocator, "low_m", Triple(bounds.low_m, allocator));
    Add(value, allocator, "high_m", Triple(bounds.high_m, allocator));
    return value;
}

Value KindCounts(const std::vector<std::string>& names,
    const std::vector<std::uint64_t>& counts, Allocator& allocator) {
    Require(names.size() == counts.size(), "Connectivity kind count shape differs");
    Value rows = Array();
    for (std::size_t i = 0; i < names.size(); ++i) {
        Value row = Object();
        String(row, allocator, "kind", names[i]);
        Integer(row, allocator, "relations", counts[i]);
        rows.PushBack(row, allocator);
    }
    return rows;
}

Value RoleCounts(const std::array<std::uint64_t, 4>& counts,
    Allocator& allocator) {
    static constexpr const char* Names[] = {
        "constitutive", "rigid_skin", "constraint", "coefficient_only"};
    Value rows = Array();
    for (unsigned i = 0; i < 4; ++i) {
        Value row = Object();
        String(row, allocator, "role", Names[i]);
        Integer(row, allocator, "relations", counts[i]);
        rows.PushBack(row, allocator);
    }
    return rows;
}

}  // namespace

output::Document BuildReport(const RunMetadata& metadata,
    const SummaryEvidence& summary, const PlasticityResult& plasticity,
    const ConnectivityEvidence& connectivity) {
    Require(!plasticity.samples.empty() &&
            plasticity.catalog.size() == plasticity.parents.size() &&
            connectivity.parents.size() == plasticity.ever_positive_parents,
        "Impact analysis report inputs are incomplete");
    output::Document document;
    document.SetObject();
    auto& allocator = document.GetAllocator();
    output::String(document, "schema", "robo_dyna.impact_response_analysis.v1");
    output::String(document, "scope",
        "Authenticated saved shell native equivalent plastic strain and static weak potential-transfer context. "
        "Samples are discrete accepted archive states; no interpolation, residual-shape, stress, total-energy, "
        "reaction-path, self-contact, or constrained-rank inference.");

    Value authority = Object();
    Add(authority, allocator, "viewer_input", File(metadata.viewer_input, allocator));
    Add(authority, allocator, "archive_manifest",
        File(metadata.archive_manifest, allocator));
    Add(authority, allocator, "run_summary",
        File(metadata.run_summary, allocator));
    Add(authority, allocator, "connectivity_report",
        File(metadata.connectivity_report, allocator));
    Add(authority, allocator, "source_canonical_manifest",
        File(metadata.source_canonical_manifest, allocator));
    Add(authority, allocator, "source_scope_report",
        File(metadata.source_scope_report, allocator));
    Add(authority, allocator, "source_member",
        File(metadata.source_member, allocator));
    String(authority, allocator, "source_mapping_sha256",
        metadata.mapping_sha256);
    String(authority, allocator, "tire_policy", metadata.tire_policy);
    Add(document, allocator, "authority", std::move(authority));

    Value units = Object();
    String(units, allocator, "source_mass", metadata.source_units.mass);
    String(units, allocator, "source_length", metadata.source_units.length);
    String(units, allocator, "source_time", metadata.source_units.time);
    Number(units, allocator, "source_mass_to_kg", metadata.source_units.mass_to_kg);
    Number(units, allocator, "source_length_to_m",
        metadata.source_units.length_to_m);
    Number(units, allocator, "source_time_to_s", metadata.source_units.time_to_s);
    String(units, allocator, "reported_position", "m");
    String(units, allocator, "reported_time", "s");
    String(units, allocator, "reported_force", "N");
    String(units, allocator, "reported_energy", "J");
    String(units, allocator, "equivalent_plastic_strain", "dimensionless");
    Add(document, allocator, "units", std::move(units));

    Value run = Object();
    Integer(run, allocator, "owner_id", metadata.identity.owner);
    Integer(run, allocator, "run_id", metadata.identity.run);
    Integer(run, allocator, "source_instance_id",
        metadata.identity.source_instance);
    Integer(run, allocator, "configuration_id",
        metadata.identity.configuration);
    Integer(run, allocator, "qualification_id",
        metadata.identity.qualification);
    Integer(run, allocator, "planned_intervals", metadata.planned_intervals);
    Integer(run, allocator, "accepted_intervals", metadata.accepted_intervals);
    Boolean(run, allocator, "horizon_complete", metadata.horizon_complete);
    String(run, allocator, "stop_reason", metadata.stop_reason);
    Number(run, allocator, "fixed_dt_s", metadata.fixed_dt_s);
    Integer(run, allocator, "saved_samples", plasticity.samples.size());
    Integer(run, allocator, "mapped_nodes", metadata.mapped_nodes);
    Integer(run, allocator, "mapped_shell_parents", metadata.mapped_parents);
    Integer(run, allocator, "mapped_triangles", metadata.mapped_triangles);
    Integer(run, allocator, "stored_native_points",
        metadata.stored_native_points);
    Integer(run, allocator, "replay_peak_host_bytes",
        metadata.replay_peak_host_bytes);
    Boolean(run, allocator, "type45_participant", metadata.profile.type45);
    Boolean(run, allocator, "structural_limit_column",
        metadata.profile.structural_limit);
    Boolean(run, allocator, "structural_beam_participant",
        metadata.profile.beam18);
    Add(document, allocator, "run", std::move(run));

    Value layout = Object();
    Integer(layout, allocator, "available_parents",
        plasticity.available_parents);
    Integer(layout, allocator, "not_applicable_parents",
        plasticity.not_applicable_parents);
    Integer(layout, allocator, "unavailable_parents",
        plasticity.unavailable_parents);
    Integer(layout, allocator, "stored_native_points", plasticity.native_points);
    static constexpr const char* LayoutNames[] = {
        "zero_points", "one_point", "three_points", "four_points", "other"};
    for (unsigned i = 0; i < plasticity.native_point_layouts.size(); ++i)
        Integer(layout, allocator, LayoutNames[i],
            plasticity.native_point_layouts[i]);
    String(layout, allocator, "point_indexing",
        "Context::point_offsets variable native ranges; no fixed points-per-parent assumption");
    Add(document, allocator, "native_field_layout", std::move(layout));

    Value global = Object();
    Integer(global, allocator, "ever_positive_native_points",
        plasticity.ever_positive_points);
    Integer(global, allocator, "ever_positive_parents",
        plasticity.ever_positive_parents);
    Integer(global, allocator, "ever_positive_parts",
        plasticity.ever_positive_parts);
    Boolean(global, allocator, "inactive_positive_history_observed",
        plasticity.inactive_positive_history_observed);
    if (plasticity.first_positive.available)
        Add(global, allocator, "first_positive_saved_sample",
            WitnessDocument(plasticity.first_positive, plasticity, allocator));
    if (plasticity.peak.available)
        Add(global, allocator, "peak_saved_value",
            WitnessDocument(plasticity.peak, plasticity, allocator));
    const auto& final = plasticity.samples.back();
    Value final_value = Object();
    Add(final_value, allocator, "sample", Stamp(final.stamp, allocator));
    Integer(final_value, allocator, "active_parents", final.active_parents);
    Integer(final_value, allocator, "positive_native_points",
        final.positive_points);
    Integer(final_value, allocator, "positive_parents",
        final.positive_parents);
    Integer(final_value, allocator, "positive_parts", final.positive_parts);
    if (final.maximum.available)
        Add(final_value, allocator, "maximum",
            WitnessDocument(final.maximum, plasticity, allocator));
    Add(global, allocator, "final_saved_sample", std::move(final_value));
    Add(document, allocator, "shell_plasticity", std::move(global));

    Value samples = Array();
    for (const auto& sample : plasticity.samples) {
        Value row = Object();
        Add(row, allocator, "sample", Stamp(sample.stamp, allocator));
        Integer(row, allocator, "active_parents", sample.active_parents);
        Integer(row, allocator, "positive_native_points",
            sample.positive_points);
        Integer(row, allocator, "active_parent_positive_native_points",
            sample.active_positive_points);
        Integer(row, allocator, "inactive_parent_positive_native_points",
            sample.inactive_positive_points);
        Integer(row, allocator, "positive_parents", sample.positive_parents);
        Integer(row, allocator, "positive_parts", sample.positive_parts);
        if (sample.maximum.available)
            Add(row, allocator, "maximum",
                WitnessDocument(sample.maximum, plasticity, allocator));
        samples.PushBack(row, allocator);
    }
    Add(document, allocator, "saved_sample_history", std::move(samples));

    Value parts = Array();
    for (const auto& part : plasticity.parts) {
        Value row = Object();
        const auto title = metadata.source_part_titles.find(part.source_part);
        Require(title != metadata.source_part_titles.end(),
            "Impact analysis report is missing a source part title");
        Integer(row, allocator, "source_part_id", part.source_part);
        String(row, allocator, "source_part_title", title->second);
        Integer(row, allocator, "source_material_id", part.source_material);
        Integer(row, allocator, "source_section_id", part.source_section);
        Integer(row, allocator, "source_elform", part.source_elform);
        Value families = Array();
        for (const auto family : part.native_families)
            families.PushBack(family, allocator);
        Add(row, allocator, "native_families", std::move(families));
        Integer(row, allocator, "parents", part.parents);
        Integer(row, allocator, "native_field_parents", part.field_parents);
        Integer(row, allocator, "stored_native_points", part.native_points);
        Integer(row, allocator, "ever_positive_parents",
            part.ever_positive_parents);
        Integer(row, allocator, "ever_positive_native_points",
            part.ever_positive_points);
        Integer(row, allocator, "final_positive_parents",
            part.final_positive_parents);
        Integer(row, allocator, "final_positive_native_points",
            part.final_positive_points);
        Integer(row, allocator, "ever_inactive_parents",
            part.ever_inactive_parents);
        Number(row, allocator, "final_maximum_equivalent_plastic_strain",
            part.final_max_strain);
        Number(row, allocator, "peak_equivalent_plastic_strain",
            part.peak_strain);
        if (part.first_positive.available)
            Add(row, allocator, "first_positive_saved_sample",
                WitnessDocument(part.first_positive, plasticity, allocator));
        if (part.peak.available)
            Add(row, allocator, "peak_saved_value",
                WitnessDocument(part.peak, plasticity, allocator));
        if (part.yielded_reference_centroid_bounds.available)
            Add(row, allocator, "yielded_parent_reference_centroid_bounds_m",
                BoundsDocument(part.yielded_reference_centroid_bounds,
                    allocator));
        if (part.yielded_final_centroid_bounds.available)
            Add(row, allocator, "yielded_parent_final_centroid_bounds_m",
                BoundsDocument(part.yielded_final_centroid_bounds,
                    allocator));
        parts.PushBack(row, allocator);
    }
    Add(document, allocator, "parts", std::move(parts));

    std::vector<std::size_t> yielded;
    yielded.reserve(plasticity.ever_positive_parents);
    for (std::size_t i = 0; i < plasticity.parents.size(); ++i)
        if (plasticity.parents[i].ever_positive) yielded.push_back(i);
    std::sort(yielded.begin(), yielded.end(), [&](std::size_t a, std::size_t b) {
        const auto& left = plasticity.catalog[a];
        const auto& right = plasticity.catalog[b];
        return left.source_part != right.source_part
            ? left.source_part < right.source_part
            : left.source_element < right.source_element;
    });

    Value parents = Array();
    for (const auto index : yielded) {
        const auto& source = plasticity.catalog[index];
        const auto& state = plasticity.parents[index];
        const auto title = metadata.source_part_titles.find(source.source_part);
        Require(title != metadata.source_part_titles.end(),
            "Impact analysis yielded parent is missing a source part title");
        Value row = Object();
        Integer(row, allocator, "source_element_id", source.source_element);
        Integer(row, allocator, "source_part_id", source.source_part);
        String(row, allocator, "source_part_title", title->second);
        Integer(row, allocator, "source_material_id", source.source_material);
        Integer(row, allocator, "source_section_id", source.source_section);
        Integer(row, allocator, "source_elform", source.source_elform);
        Integer(row, allocator, "native_family", source.native_family);
        Integer(row, allocator, "family_index", source.family_index);
        Integer(row, allocator, "canonical_parent_index",
            source.canonical_parent);
        Integer(row, allocator, "native_points", source.native_points);
        Value nodes = Array();
        for (std::size_t slot = 0; slot < source.node_count(); ++slot) {
            Value node;
            node.SetUint64(source.source_nodes[slot]);
            nodes.PushBack(node, allocator);
        }
        Add(row, allocator, "source_node_ids", std::move(nodes));
        Integer(row, allocator, "ever_positive_native_points",
            state.ever_positive_points);
        Integer(row, allocator, "peak_positive_native_points",
            state.peak_positive_points);
        Integer(row, allocator, "final_positive_native_points",
            state.final_positive_points);
        Number(row, allocator, "first_positive_saved_maximum",
            state.first_positive_strain);
        Number(row, allocator, "peak_equivalent_plastic_strain",
            state.peak_strain);
        Number(row, allocator, "final_maximum_equivalent_plastic_strain",
            state.final_max_strain);
        Add(row, allocator, "first_positive_saved_sample",
            OccurrenceDocument(state.first_positive, allocator));
        Add(row, allocator, "peak_saved_value",
            OccurrenceDocument(state.peak, allocator));
        Boolean(row, allocator, "ever_inactive", state.ever_inactive);
        Boolean(row, allocator, "active_at_final_sample",
            state.active_at_final_sample);
        Add(row, allocator, "reference_centroid_m",
            Triple(state.reference_centroid_m, allocator));
        Add(row, allocator, "final_centroid_m",
            Triple(state.final_centroid_m, allocator));
        std::array<double, 3> translation{};
        for (unsigned axis = 0; axis < 3; ++axis)
            translation[axis] =
                state.final_centroid_m[axis] - state.reference_centroid_m[axis];
        Add(row, allocator, "final_centroid_translation_m",
            Triple(translation, allocator));
        parents.PushBack(row, allocator);
    }
    Add(document, allocator, "yielded_parents", std::move(parents));

    Value companion = Object();
    String(companion, allocator, "schema", summary.schema);
    String(companion, allocator, "status", summary.status);
    String(companion, allocator, "reason", summary.reason);
    String(companion, allocator, "physical_profile", summary.physical_profile);
    Number(companion, allocator, "actual_completed_time_s",
        summary.actual_completed_time_s);
    Value wall = Object();
    Number(wall, allocator, "peak_force_n", summary.peak_wall_force_n);
    Number(wall, allocator, "peak_penetration_m",
        summary.peak_wall_penetration_m);
    Number(wall, allocator, "peak_same_mask_potential_j",
        summary.peak_same_mask_potential_j);
    Number(wall, allocator, "last_same_mask_potential_j",
        summary.last_same_mask_potential_j);
    Number(wall, allocator, "last_removed_potential_j",
        summary.last_removed_potential_j);
    Add(companion, allocator, "wall_observations", std::move(wall));
    Value reported_plastic = Object();
    Number(reported_plastic, allocator, "solid_law36_law44_sum_j",
        summary.solid_reported_plastic_work_sum_j);
    Number(reported_plastic, allocator, "structural_beam_sum_j",
        summary.beam_reported_plastic_work_sum_j);
    String(reported_plastic, allocator, "scope",
        "Included native reported work components; not point strain and not additional total energy");
    Add(companion, allocator, "other_reported_plastic_work",
        std::move(reported_plastic));
    Value motion = Object();
    Number(motion, allocator, "translation_departure_last_m",
        summary.translation_departure_last_m);
    Number(motion, allocator, "translation_departure_peak_m",
        summary.translation_departure_peak_m);
    Number(motion, allocator, "velocity_departure_last_m_s",
        summary.velocity_departure_last_m_s);
    Number(motion, allocator, "velocity_departure_peak_m_s",
        summary.velocity_departure_peak_m_s);
    Number(motion, allocator, "orientation_component_departure_last",
        summary.orientation_component_departure_last);
    Number(motion, allocator, "orientation_component_departure_peak",
        summary.orientation_component_departure_peak);
    Number(motion, allocator, "spin_component_last_rad_s",
        summary.spin_component_last_rad_s);
    Number(motion, allocator, "spin_component_peak_rad_s",
        summary.spin_component_peak_rad_s);
    String(motion, allocator, "scope",
        "Departure from original uniform translation; component maxima, not strain or fitted rigid-motion residual");
    Add(companion, allocator, "motion", std::move(motion));
    Add(document, allocator, "authenticated_run_summary_companion",
        std::move(companion));

    Value connectivity_value = Object();
    String(connectivity_value, allocator, "schema", connectivity.schema);
    String(connectivity_value, allocator, "scope", connectivity.scope);
    String(connectivity_value, allocator, "source_archive_sha256",
        connectivity.archive_sha256);
    Integer(connectivity_value, allocator, "nodes", connectivity.nodes);
    Integer(connectivity_value, allocator, "relations",
        connectivity.relations);
    Integer(connectivity_value, allocator, "ordered_slots",
        connectivity.ordered_slots);
    Integer(connectivity_value, allocator, "element_components",
        connectivity.element_components);
    Integer(connectivity_value, allocator, "potential_transfer_components",
        connectivity.potential_transfer_components);
    Integer(connectivity_value, allocator, "yielded_source_nodes",
        connectivity.yielded_source_nodes);
    Integer(connectivity_value, allocator, "incident_relations",
        connectivity.incident_relations);
    Add(connectivity_value, allocator, "complete_relations_by_kind",
        KindCounts(connectivity.kind_codes, connectivity.relations_by_kind,
            allocator));
    Add(connectivity_value, allocator, "yielded_incident_relations_by_kind",
        KindCounts(connectivity.kind_codes,
            connectivity.incident_relations_by_kind, allocator));
    Add(connectivity_value, allocator, "yielded_incident_relations_by_role",
        RoleCounts(connectivity.incident_relations_by_role, allocator));
    Add(connectivity_value, allocator,
        "yielded_transfer_component_min_source_node_ids",
        UnsignedArray(connectivity.yielded_transfer_component_labels,
            allocator));
    String(connectivity_value, allocator, "interpretation",
        "Direct shared-node incidence and weak potential-transfer membership only; no current force, moment, reaction, released-DOF rank, or post-damage transfer claim");

    std::map<std::size_t, const ConnectivityParentEvidence*> connectivity_by_parent;
    for (const auto& evidence : connectivity.parents)
        connectivity_by_parent.emplace(evidence.parent_index, &evidence);
    Value connectivity_parents = Array();
    for (const auto index : yielded) {
        const auto found = connectivity_by_parent.find(index);
        Require(found != connectivity_by_parent.end(),
            "Missing yielded parent connectivity evidence");
        const auto& evidence = *found->second;
        const auto& source = plasticity.catalog[index];
        Value row = Object();
        Integer(row, allocator, "source_element_id", source.source_element);
        Integer(row, allocator, "source_part_id", source.source_part);
        Integer(row, allocator, "source_nodes", evidence.source_nodes);
        Integer(row, allocator, "matched_constitutive_relations",
            evidence.matched_constitutive_relations);
        Integer(row, allocator, "combined_node_role_bits",
            evidence.combined_node_role_bits);
        Add(row, allocator, "transfer_component_min_source_node_ids",
            UnsignedArray(evidence.transfer_component_labels, allocator));
        Add(row, allocator, "incident_relations_by_kind",
            KindCounts(connectivity.kind_codes,
                evidence.incident_relations_by_kind, allocator));
        Add(row, allocator, "incident_relations_by_role",
            RoleCounts(evidence.incident_relations_by_role, allocator));
        Value incident_parts = Array();
        for (std::size_t kind = 0; kind < connectivity.kind_codes.size(); ++kind) {
            if (evidence.incident_part_ids_by_kind[kind].empty()) continue;
            Value item = Object();
            String(item, allocator, "kind", connectivity.kind_codes[kind]);
            Add(item, allocator, "source_part_ids",
                UnsignedArray(evidence.incident_part_ids_by_kind[kind],
                    allocator));
            incident_parts.PushBack(item, allocator);
        }
        Add(row, allocator, "direct_incident_source_parts_by_kind",
            std::move(incident_parts));
        connectivity_parents.PushBack(row, allocator);
    }
    Add(connectivity_value, allocator, "yielded_parents",
        std::move(connectivity_parents));
    Add(document, allocator, "static_connectivity",
        std::move(connectivity_value));

    Value unavailable = Array();
    static constexpr const char* Missing[] = {
        "continuous first-yield time between saved samples",
        "shell stress/resultant fields and through-thickness stress histories",
        "settled residual or permanent deformation",
        "TIME0 kinetic/internal/total energy and a closed whole-system energy balance",
        "per-feature wall reaction and dynamic connector/constraint reaction histories",
        "current constrained-DOF rank and verified force/moment transfer path",
        "vehicle self-contact force, crossing, friction, or folding diagnostics",
        "solid/beam spatial plastic-strain fields and calibrated rupture evidence"};
    for (const auto* text : Missing)
        unavailable.PushBack(Value(text, allocator), allocator);
    Add(document, allocator, "unavailable_quantities",
        std::move(unavailable));
    return document;
}

}  // namespace crash::analysis::impact_response
