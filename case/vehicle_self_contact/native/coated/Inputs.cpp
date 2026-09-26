#include "Internal.h"
#include "modelio/source_assembly/NativeCoordinates.h"
#include "modelio/solid_source/SourcePolicy.h"
#include "lib_src/elements/solid6z/CollapsedBrickTopology.h"
#include "lib_src/collision/self_contact_filters/Environment.h"
#include <algorithm>
#include <climits>
#include <cstring>
namespace crash::cases::vehicle_self_contact::native::coated::detail {
namespace {
bool Bits(double a, double b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }
template<class T> std::vector<T> Decode(const source::CanonicalData& c, const char* name) {
    const auto& array = source::FindArray(c, name);
    return output::arrays::Decode<T>(array.descriptor, array.bytes,
        {c.limits.file_bytes, std::max(c.limits.nodes, c.limits.parents), 64});
}
bool Selected(const selection::Data& source, std::uint64_t pid) {
    const auto row = std::lower_bound(source.parts.begin(), source.parts.end(), pid,
        [](const auto& part, auto key) { return part.part_id < key; });
    return row != source.parts.end() && row->part_id == pid && row->retained_shell_part;
}
template<class Parent>
Solid BindSolid(const modelio::solid_source::Row& row, const Parent& parent,
        const Inputs& inputs, const std::vector<std::uint32_t>& canonical_to_domain,
        const std::vector<std::uint64_t>& canonical_ids, unsigned count) {
    const auto& reference = parent.reference;
    const auto& original = reference.input();
    output::Require(reference.prepared() && original.source_element_id == row.element_id &&
        original.source_part_id == row.part_id, "Physical solid reference identity differs from selected source");
    Solid result;
    result.source_id = row.element_id; result.part_id = row.part_id;
    result.canonical_row = row.canonical_row; result.source_line = row.source_line;
    result.family = std::uint32_t(row.family); result.reference_index = std::uint32_t(row.reference_index);
    result.kind = count == 6 ? ReaderKind::DeclaredPenta6 : ReaderKind::Hex8;
    std::array<std::uint32_t, 8> declared{};
    std::uint64_t raw_ids[8];
    std::copy(row.raw_node_ids.begin(), row.raw_node_ids.end(), raw_ids);
    if (count == 6) {
        tl::fea::solid6z::CollapsedBrickTopology packed;
        output::Require(tl::fea::solid6z::MapCollapsedTopEdges(raw_ids, packed) == tl::fea::solid6z::Status::Success,
                        "V5 PENTA declaration lost its explicit original collapsed-brick mapping");
        for (unsigned slot = 0; slot < 6; ++slot)
            output::Require(packed.six_to_raw[slot] == row.six_to_raw[slot], "V5 PENTA source packing changed");
    }
    for (unsigned slot = 0; slot < 8; ++slot)
        output::Require(row.canonical_nodes[slot] < canonical_ids.size() &&
            canonical_ids[row.canonical_nodes[slot]] == row.raw_node_ids[slot],
            "Raw solid source/canonical node identity differs");
    unsigned mapped = 0;
    for (unsigned slot = 0; slot < count; ++slot) {
        const unsigned raw = count == 6 ? row.six_to_raw[slot] : slot;
        output::Require(raw < 8 && row.canonical_nodes[raw] < canonical_to_domain.size(), "Solid source slot outside canonical domain");
        const auto domain = canonical_to_domain[row.canonical_nodes[raw]];
        output::Require(domain != UINT32_MAX && domain < inputs.nodes.size() &&
            canonical_ids[row.canonical_nodes[raw]] == row.raw_node_ids[raw] &&
            original.source_node_id[slot] == row.raw_node_ids[raw] && parent.domain_nodes[slot] == domain,
            "Solid canonical/reference/physical node mappings differ");
        const auto native = inputs.nodes[domain].native_position;
        const auto represented = original.position_m[slot];
        output::Require(Bits(native.x * inputs.units.length_m, represented.x) &&
            Bits(native.y * inputs.units.length_m, represented.y) && Bits(native.z * inputs.units.length_m, represented.z),
            "Solid reference coordinates differ from source-native node cards");
        declared[slot] = domain;
        const auto source_slot = reference.source_slot(slot);
        output::Require(source_slot < count && !(mapped & (1u << source_slot)), "Invalid retained mechanics source-slot permutation");
        mapped |= 1u << source_slot;
    }
    // The checked mechanics permutation is retained authority, but deliberately
    // not applied to the earlier contact reader packet.
    try {
        result.nodes = ReaderSlots(result.kind, declared, inputs.nodes);
    } catch (const std::exception& error) {
        throw std::runtime_error("V5 contact reader EID " + std::to_string(row.element_id) +
            " PID " + std::to_string(row.part_id) + " source line " + std::to_string(row.source_line) + ": " + error.what());
    }
    return result;
}
}
const source::CanonicalData& CheckSource(const PhysicalModel& model,
        const selection::OriginalSelection& selection, Config config, Limits limits) {
    using namespace modelio;
    output::Require(tlfea::contact::self_contact_filters::CompatibleHostArithmetic(),
                    "V5 native source assessment requires RN, gradual underflow and masked floating traps");
    const auto& source_domain = model.source_domain();
    const auto& solid_input = source_domain.source().solid_source();
    const auto& canonical = selection.canonical().data();
    const auto& domain = source_domain.domain();
    const auto& shells = model.shell_source();
    const Limits hard;
    const s::Limits topology_hard;
    output::Require(limits.topology.max_nodes && limits.topology.max_nodes <= topology_hard.max_nodes &&
        limits.topology.max_primary_faces && limits.topology.max_primary_faces <= topology_hard.max_primary_faces &&
        limits.topology.max_output_bytes && limits.topology.max_output_bytes <= topology_hard.max_output_bytes &&
        limits.topology.max_scratch_bytes && limits.topology.max_scratch_bytes <= topology_hard.max_scratch_bytes,
        "Invalid coated topology limits");
    output::Require(config.scope == Scope::RetainedV5PhysicalShellsAndOriginalContact &&
        config.coordinates == SourceCoordinates::OriginalNativeNodeCards &&
        config.order == NativeOrder::CaseDeclaredAscendingPhysicalNidItab &&
        config.membership == SurfaceMembership::SingleSurfaceImbinZero,
        "Unsupported V5 coating source declaration");
    output::Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes &&
        limits.nodes && limits.nodes <= hard.nodes && limits.shells && limits.shells <= hard.shells &&
        limits.solids && limits.solids <= hard.solids && limits.metadata_bytes && limits.metadata_bytes <= hard.metadata_bytes,
        "Invalid V5 coating source limits");
    output::Require(source_domain.policy() == physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5 &&
        solid_input.data().policy == solid_source::Policy::OriginalVehicleSupportsV5 &&
        &canonical == &solid_input.canonical().data() && &canonical == &shells.references().source().canonical().data(),
        "V5 coating source handles do not share the required retained canonical authority");
    output::Require(domain.prepared() && model.solids().prepared() && model.solids().domain() &&
        model.solids().domain()->SharesStorage(domain) && model.solids().source_instance_id() == domain.source_instance_id(),
        "V5 coating physical solid domain differs");
    output::Require(domain.node_count() <= limits.nodes && shells.references().rows().size() <= limits.shells &&
        solid_input.data().rows.size() <= limits.solids && selection.data().counts.retained_shells <= limits.shells,
        "V5 coating source count exceeds capacity");
    const auto& solids = model.solids();
    output::Require(solids.solid18().size() == 908 && solids.solid24().size() == 1991 && solids.solid6z().size() == 350 &&
        solids.solid18_law44().size() == 386 && solids.solid18_law90().size() == 1345 &&
        solid_input.data().rows.size() == 4980 && shells.references().rows().size() == 349645 &&
        selection.data().counts.retained_shells == 337092, "Complete retained V5 source census changed");
    output::Require(canonical.inputs.units.length == "mm" && canonical.inputs.units.length_to_m == .001,
        "V5 reader packet requires original declared millimetres");
    return canonical;
}
Inputs PrepareInputs(const PhysicalModel& model, const selection::OriginalSelection& selection,
        const std::string& member, Config config, Limits limits) {
    const auto& canonical = CheckSource(model, selection, config, limits);
    output::Require(member.size() == canonical.inputs.source_member.bytes &&
        member.capacity() <= 2*canonical.inputs.source_member.bytes &&
        output::Sha256(member) == canonical.inputs.source_member.sha256, "V5 coating original member hash or extent differs");
    const auto ids = Decode<std::uint64_t>(canonical, "node_ids");
    output::Require(ids.size() == canonical.canonical_nodes && std::is_sorted(ids.begin(), ids.end()),
        "V5 canonical node identity order changed");
    const auto physical = model.source_domain().domain().nodes();
    std::vector<std::uint32_t> requested, canonical_to_domain(ids.size(), UINT32_MAX);
    requested.reserve(physical.size());
    std::uint64_t previous = 0;
    for (std::size_t i = 0; i < physical.size(); ++i) {
        const auto id = physical[i].source_id;
        output::Require(id > previous && id <= INT_MAX, "V5 ITAB source NID is not strictly increasing and native-int representable");
        previous = id;
        const auto found = std::lower_bound(ids.begin(), ids.end(), id);
        output::Require(found != ids.end() && *found == id, "Physical node is absent from canonical source");
        const auto row = std::size_t(found - ids.begin());
        requested.push_back(std::uint32_t(row)); canonical_to_domain[row] = std::uint32_t(i);
    }
    auto native = modelio::source_nodes::ReadNativeCoordinates(canonical, ids, requested, member);
    Inputs result;
    result.units = {canonical.inputs.units.length_to_m, canonical.inputs.units.mass_to_kg, canonical.inputs.units.time_to_s};
    result.coordinate_roundtrip_changes = native.roundtrip_changed_components;
    result.nodes.reserve(physical.size());
    for (std::size_t i = 0; i < physical.size(); ++i) {
        const auto x = native.positions[i]; const auto actual = physical[i].position;
        output::Require(Bits(x[0]*result.units.length_m, actual.x) && Bits(x[1]*result.units.length_m, actual.y) &&
            Bits(x[2]*result.units.length_m, actual.z), "Native node card differs from actual physical-domain SI bits");
        result.nodes.push_back({physical[i].source_id, requested[i], {x[0], x[1], x[2]}});
    }
    // Retire the returned coordinate vector before shell/solid packet staging.
    decltype(native.positions){}.swap(native.positions);
    const auto records = Decode<std::uint64_t>(canonical, "shells_records");
    const auto connectivity = Decode<std::uint32_t>(canonical, "shells_node_indices");
    const auto lines = Decode<std::uint32_t>(canonical, "shells_source_lines");
    output::Require(records.size() == 6*canonical.canonical_shells && connectivity.size() == 4*canonical.canonical_shells &&
        lines.size() == canonical.canonical_shells, "Physical shell canonical extents differ");
    std::vector<unsigned char> seen(canonical.canonical_shells, 0);
    const auto& rows = model.shell_source().references().rows();
    result.shells.reserve(rows.size()); std::size_t selected_count = 0;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto& row = rows[i]; const auto k = row.canonical_parent;
        output::Require(k < canonical.canonical_shells && !seen[k] && records[6*k] == row.element_id &&
            records[6*k+1] == row.part_id && lines[k] == row.source_line &&
            row.status == vehicle_startup::ReferenceStatus::Success, "Physical shell reference/source identity differs");
        seen[k] = 1;
        Shell shell;
        shell.primary.source_id = row.element_id; shell.part_id = row.part_id;
        shell.canonical_row = k; shell.source_line = row.source_line; shell.physical_parent = std::uint32_t(i);
        shell.contact_selected = Selected(selection.data(), row.part_id); selected_count += shell.contact_selected;
        shell.primary.layout = connectivity[4*k+2] == connectivity[4*k+3] ? n::ShellLayout::Triangle3 : n::ShellLayout::Quad4;
        for (unsigned slot = 0; slot < 4; ++slot) {
            const auto node = connectivity[4*k+slot];
            output::Require(node < ids.size() && ids[node] == records[6*k+2+slot] && canonical_to_domain[node] != UINT32_MAX,
                "Physical shell source node is missing or changed");
            shell.primary.nodes[slot] = canonical_to_domain[node];
        }
        result.shells.push_back(shell);
    }
    output::Require(selected_count == selection.data().counts.retained_shells, "Complete original-contact shell coverage differs");
    const auto& solid_rows = model.source_domain().source().solid_source().data().rows;
    const auto& solids = model.solids(); result.solids.reserve(solid_rows.size());
    std::array<std::size_t, 5> counts{};
    const std::size_t family_counts[]{solids.solid18().size(), solids.solid24().size(), solids.solid6z().size(),
        solids.solid18_law44().size(), solids.solid18_law90().size()};
    for (const auto& row : solid_rows) {
        const auto family = std::size_t(row.family);
        output::Require(family < counts.size() && row.reference_index < family_counts[family] && row.reference_index == counts[family]++, "Selected solid source/family order differs");
        using F = modelio::solid_source::Family;
        switch (row.family) {
        case F::Solid18: result.solids.push_back(BindSolid(row, solids.solid18()[row.reference_index], result, canonical_to_domain, ids, 8)); break;
        case F::Solid24: result.solids.push_back(BindSolid(row, solids.solid24()[row.reference_index], result, canonical_to_domain, ids, 8)); break;
        case F::Solid6z: result.solids.push_back(BindSolid(row, solids.solid6z()[row.reference_index], result, canonical_to_domain, ids, 6)); break;
        case F::Solid18Law44: result.solids.push_back(BindSolid(row, solids.solid18_law44()[row.reference_index], result, canonical_to_domain, ids, 8)); break;
        case F::Solid18Law90: result.solids.push_back(BindSolid(row, solids.solid18_law90()[row.reference_index], result, canonical_to_domain, ids, 8)); break;
        }
    }
    for (unsigned family = 0; family < 5; ++family)
        output::Require(counts[family] == family_counts[family], "Incomplete selected solid family coverage");
    return result;
}
Location ShellLocation(const Inputs& input, std::size_t index) {
    Location location;
    if (index == SIZE_MAX) return location;
    output::Require(index < input.shells.size(), "Invalid coating report shell index");
    const auto& shell = input.shells[index];
    location.canonical_row = shell.canonical_row; location.source_eid = shell.primary.source_id;
    location.source_pid = shell.part_id; location.source_line = shell.source_line;
    return location;
}
} // namespace crash::cases::vehicle_self_contact::native::coated::detail
