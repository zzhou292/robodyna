#include "MappingDraft.h"
#include "InputChecks.h"
#include "MappingRecords.h"
#include <algorithm>

namespace crash::output::full_shell::source::detail {
namespace {
template<class T> std::vector<T> Decode(const CanonicalData& d, const char* name) {
    const auto& a = FindArray(d, name);
    return arrays::Decode<T>(a.descriptor, a.bytes);
}
} // namespace
std::size_t MappingWorkingBytes(const CanonicalData& d, MappingInput in) {
    Require(in.node_count == d.retained_nodes && in.parent_count == d.retained_shells &&
        in.canonical_nodes && in.parents, "Missing/incomplete runtime source ordering");
    std::size_t bytes = 0;
    for (const auto& a : d.arrays) AddBytes(bytes, 2 * a.descriptor.bytes, d.limits.host_bytes);
    for (std::size_t i = 0; i < MappingSpecs().size(); ++i) {
        const auto layout = MappingLayout(i, in.node_count, in.parent_count, 2 * d.retained_q4 + d.retained_t3);
        const auto n = arrays::ByteCount(layout, {d.limits.file_bytes, UINT32_MAX, 64});
        AddBytes(bytes, 4 * n, d.limits.host_bytes);
    }
    AddBytes(bytes, 2 * in.parent_count * sizeof(full_shell::ParentPoints), d.limits.host_bytes);
    AddBytes(bytes, 8 * d.canonical_nodes, d.limits.host_bytes);
    AddBytes(bytes, 2 * d.canonical_bytes.size(), d.limits.host_bytes);
    AddBytes(bytes, 2 * d.scope_bytes.size(), d.limits.host_bytes);
    if (in.execution) {
        CheckMappingExecution(*in.execution);
        // Complete bounded descriptor clone, per-PID counters and digest/JSON
        // staging. Metadata codec stays capped at the existing16KiB record cap.
        AddBytes(bytes, 2 * sizeof(MappingExecution), d.limits.host_bytes);
        AddBytes(bytes, 2 * in.execution->parts.size() * sizeof(MappingExecutionPart), d.limits.host_bytes);
        AddBytes(bytes, in.execution->parts.size() * sizeof(std::array<std::uint64_t, 2>), d.limits.host_bytes);
        AddBytes(bytes, 16 * MappingMetadataByteCap, d.limits.host_bytes);
    }
    return bytes;
}
namespace {
void CheckNative(const NativeParent& p, std::size_t count) {
    Require(p.native_family && p.family_index < count && p.native_points <= 64,
        "Invalid caller native family/index/point declaration");
    switch (p.plastic) {
        case PlasticField::NativeEquivalentPlasticStrain:
            Require(p.native_points, "Available native plastic field has no points");
            break;
        case PlasticField::NotApplicable:
        case PlasticField::Unavailable:
            break;
        default: throw std::runtime_error("Unknown native plastic field applicability");
    }
}
} // namespace
MappingDraft BuildMapping(const CanonicalData& d, MappingInput in) {
    (void)MappingWorkingBytes(d, in); // Count/byte checks precede borrowed arrays and allocations.
    std::vector<std::array<std::uint64_t, 2>> execution_counts;
    if (in.execution) {
        CheckMappingExecution(*in.execution);
        Require(Bits(in.execution->projection_working_length_m) == Bits(d.inputs.units.length_to_m) &&
            Bits(in.execution->coefficient_working_length_m) == Bits(d.inputs.units.length_to_m),
            "Mapping execution metrics differ from authenticated source units");
        execution_counts.resize(in.execution->parts.size());
        for (const auto& p : in.execution->parts) {
            const auto& source = FindPart(d, p.part);
            Require(source.material == p.material && source.section == p.section && source.shell_section &&
                source.material_role == SourceMaterialRole::Elastic &&
                (source.source_elform == 2 || source.source_elform == 16),
                "Global LAW1 mapping PID/MID/SID/material role differs from source");
        }
    }
    const auto node_ids = Decode<std::uint64_t>(d, "node_ids");
    const auto records = Decode<std::uint64_t>(d, "shells_records");
    const auto connectivity = Decode<std::uint32_t>(d, "shells_node_indices");
    MappingDraft next;
    next.node_ids.reserve(in.node_count);
    next.node_canonical.reserve(in.node_count);
    std::vector<std::uint32_t> local(d.canonical_nodes, UINT32_MAX);
    for (std::size_t i = 0; i < in.node_count; ++i) {
        const auto canonical = in.canonical_nodes[i];
        Require(canonical < local.size() && local[canonical] == UINT32_MAX,
            "Duplicate/out-of-range runtime source node");
        local[canonical] = static_cast<std::uint32_t>(i);
        next.node_ids.push_back(node_ids[canonical]);
        next.node_canonical.push_back(canonical);
    }
    next.parent_ids.reserve(4 * in.parent_count);
    next.parent_reference.reserve(3 * in.parent_count);
    next.parent_points.reserve(3 * in.parent_count);
    next.parent_nodes.reserve(4 * in.parent_count);
    next.triangles.reserve(3 * (2 * d.retained_q4 + d.retained_t3));
    next.triangle_parents.reserve(2 * d.retained_q4 + d.retained_t3);
    next.points.reserve(in.parent_count);
    std::vector<unsigned char> seen(d.canonical_shells, 0), used(in.node_count, 0);
    std::vector<std::pair<std::uint32_t, std::uint32_t>> family_indices;
    family_indices.reserve(in.parent_count);
    std::size_t plastic_points = 0;
    for (std::size_t i = 0; i < in.parent_count; ++i) {
        const auto& p = in.parents[i];
        CheckNative(p, in.parent_count);
        Require(p.canonical_parent < seen.size() && !seen[p.canonical_parent],
            "Duplicate/out-of-range runtime source parent");
        seen[p.canonical_parent] = 1;
        family_indices.emplace_back(p.native_family, p.family_index);
        const auto* raw = records.data() + 6 * p.canonical_parent;
        const auto& part = FindPart(d, raw[1]);
        Require(std::binary_search(d.selected_parts.begin(), d.selected_parts.end(), raw[1]) && part.source_elform,
            "Runtime parent is omitted or lacks a literal source ELFORM");
        const MappingExecutionPart* resolved = nullptr;
        std::size_t resolved_index = 0;
        if (in.execution) {
            const auto& parts = in.execution->parts;
            const auto at = std::lower_bound(parts.begin(), parts.end(), raw[1],
                [](const auto& entry, std::uint64_t id) { return entry.part < id; });
            if (at != parts.end() && at->part == raw[1]) {
                resolved = &*at;
                resolved_index = at - parts.begin();
            }
        }
        // A known elastic zero-point role is the additive resolved execution
        // profile. It cannot lose its descriptor by downgrading to mapping-v1.
        if (part.material_role == SourceMaterialRole::Elastic &&
            p.plastic == PlasticField::NotApplicable && p.native_points == 0) {
            Require(resolved, "Global elastic zero-point mapping lacks execution provenance");
        }
        if (resolved) {
            const bool triangle = connectivity[4 * p.canonical_parent + 2] == connectivity[4 * p.canonical_parent + 3];
            Require(p.native_points == 0 && p.plastic == PlasticField::NotApplicable &&
                p.native_family == (triangle ? 2u : 1u), "Global LAW1 mapping has forged family/point availability");
            ++execution_counts[resolved_index][triangle ? 1 : 0];
        }
        next.parent_ids.insert(next.parent_ids.end(), {raw[0], raw[1], part.material, part.section});
        next.parent_reference.insert(next.parent_reference.end(), {p.canonical_parent, p.native_family, p.family_index});
        next.parent_points.insert(next.parent_points.end(), {part.source_elform, p.native_points, static_cast<unsigned>(p.plastic)});
        next.points.push_back({raw[0], raw[1], part.source_elform, p.native_family, p.native_points, p.plastic});
        if (p.plastic == PlasticField::NativeEquivalentPlasticStrain) {
            Require(p.native_points <= 4194304 - plastic_points, "Native plastic point pool exceeds capacity");
            plastic_points += p.native_points;
        }
        std::uint32_t n[4];
        for (unsigned j = 0; j < 4; ++j) {
            n[j] = local[connectivity[4 * p.canonical_parent + j]];
            Require(n[j] != UINT32_MAX, "Runtime source parent has a missing node");
            used[n[j]] = 1;
            next.parent_nodes.push_back(n[j]);
        }
        next.triangles.insert(next.triangles.end(), {n[0], n[1], n[2]});
        next.triangle_parents.push_back(static_cast<std::uint32_t>(i));
        if (n[2] != n[3]) {
            next.triangles.insert(next.triangles.end(), {n[0], n[2], n[3]});
            next.triangle_parents.push_back(static_cast<std::uint32_t>(i));
        }
    }
    if (in.execution) {
        for (std::size_t i = 0; i < execution_counts.size(); ++i) {
            Require(execution_counts[i][0] == in.execution->parts[i].qeph &&
                execution_counts[i][1] == in.execution->parts[i].t3,
                "Global LAW1 per-PID mapping coverage is incomplete");
        }
    }
    std::sort(family_indices.begin(), family_indices.end());
    Require(std::adjacent_find(family_indices.begin(), family_indices.end()) == family_indices.end() &&
        std::all_of(used.begin(), used.end(), [](unsigned char x) { return x != 0; }) &&
        next.triangle_parents.size() == 2 * d.retained_q4 + d.retained_t3,
        "Runtime mapping has duplicate family indices or incomplete source incidence");
    return next;
}
} // namespace crash::output::full_shell::source::detail
