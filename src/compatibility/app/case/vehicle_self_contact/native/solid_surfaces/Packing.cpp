#include "Internal.h"
#include <algorithm>

namespace crash::cases::vehicle_self_contact::native::initial_surfaces::detail {
namespace {
bool Selected(const std::vector<std::uint64_t>& parts, std::uint64_t id) {
    return std::binary_search(parts.begin(), parts.end(), id);
}
template<class T> const T* Data(const std::vector<T>& values) {
    return values.empty() ? nullptr : values.data();
}
}
values::Input Packing::Input(bool probe) const {
    values::Input input;
    input.phase = values::ReaderPhase::BeforeGroupingAndInitia;
    input.node_count = nodes;
    input.solids = Data(solids);
    input.solid_count = solids.size();
    input.quads = Data(quads);
    input.quad_count = quads.size();
    input.triangles = Data(triangles);
    input.triangle_count = triangles.size();
    input.clause.kind = probe ? values::ClauseKind::Solids : values::ClauseKind::Parts;
    input.clause.mode = values::SurfaceMode::Exterior;
    if (probe) {
        input.clause.solid_rows = Data(selected_solids);
        input.clause.solid_row_count = selected_solids.size();
    } else {
        input.clause.part_ids = Data(selected_parts);
        input.clause.part_count = selected_parts.size();
    }
    return input;
}
Packing Pack(const coated::Inputs& input, const std::vector<std::uint64_t>& selected) {
    Packing out;
    out.nodes = input.nodes.size();
    out.selected_parts = selected;
    std::sort(out.selected_parts.begin(), out.selected_parts.end());
    output::Require(!out.selected_parts.empty() && out.selected_parts.front() &&
        std::adjacent_find(out.selected_parts.begin(), out.selected_parts.end()) == out.selected_parts.end(),
        "Initial surface part selection is not a unique positive set");
    out.solids.reserve(input.solids.size());
    out.selected_solids.reserve(input.solids.size());
    const auto triangles = std::count_if(input.shells.begin(), input.shells.end(), [](const auto& row) {
        return row.primary.layout == coated::n::ShellLayout::Triangle3;
    });
    const auto quads = input.shells.size() - std::size_t(triangles);
    out.quads.reserve(quads);
    out.triangles.reserve(std::size_t(triangles));
    out.quad_to_physical.reserve(quads);
    out.triangle_to_physical.reserve(std::size_t(triangles));
    for (std::size_t i = 0; i < input.solids.size(); ++i) {
        const auto& row = input.solids[i];
        output::Require(row.phase == coated::PacketPhase::ReaderBeforeInitia,
            "Initial surface needs the authentic reader raw8 packet");
        values::Solid solid;
        solid.element_id = row.source_id;
        solid.part_id = row.part_id;
        output::Require(row.kind == coated::ReaderKind::Hex8 || row.kind == coated::ReaderKind::DeclaredPenta6,
            "Unsupported retained solid reader family");
        solid.topology = row.kind == coated::ReaderKind::DeclaredPenta6 ?
            values::SolidTopology::DeclaredPenta6 : values::SolidTopology::Hex8;
        if (row.kind == coated::ReaderKind::Hex8) {
            // Native reader BRICK keeps all eight slots, including collapsed
            // edges. Select by the authenticated packet shape, never by PID,
            // material, or a repaired PENTA connectivity.
            bool repeated = false;
            for (unsigned k = 0; k < 8; ++k)
                for (unsigned j = 0; j < k; ++j) repeated = repeated || row.nodes[k] == row.nodes[j];
            if (repeated) solid.topology = values::SolidTopology::NativeRaw8;
        }
        std::copy(row.nodes.begin(), row.nodes.end(), solid.nodes);
        out.solids.push_back(solid);
        if (Selected(out.selected_parts, row.part_id)) out.selected_solids.push_back(std::uint32_t(i));
    }
    for (std::size_t i = 0; i < input.shells.size(); ++i) {
        const auto& row = input.shells[i];
        values::Shell shell;
        shell.element_id = row.primary.source_id;
        shell.part_id = row.part_id;
        std::copy(std::begin(row.primary.nodes), std::end(row.primary.nodes), shell.nodes);
        output::Require(Selected(out.selected_parts, row.part_id) == row.contact_selected,
            "Complete source shell membership differs from authenticated selection");
        if (row.primary.layout == coated::n::ShellLayout::Quad4) {
            out.quads.push_back(shell);
            out.quad_to_physical.push_back(std::uint32_t(i));
        } else {
            output::Require(row.primary.layout == coated::n::ShellLayout::Triangle3,
                "Unsupported retained source shell topology");
            out.triangles.push_back(shell);
            out.triangle_to_physical.push_back(std::uint32_t(i));
        }
    }
    return out;
}
Face ExternalFace(const values::Face& raw, const Packing& packed, const coated::Inputs& input) {
    Face out;
    out.source.kind = raw.source.kind;
    out.source.element_id = raw.source.element_id;
    out.source.part_id = raw.source.part_id;
    out.source.solid_face = raw.source.solid_face;
    std::copy(std::begin(raw.nodes), std::end(raw.nodes), out.nodes.begin());
    out.raw_role = raw.raw_role;
    const auto ordinal = raw.source.reader_row; // Private representative table address only.
    if (raw.source.kind == values::ParentKind::Solid) {
        output::Require(ordinal < input.solids.size(), "Initial solid face has a foreign physical parent");
        const auto& source = input.solids[ordinal];
        output::Require(source.source_id == raw.source.element_id && source.part_id == raw.source.part_id &&
            raw.source.solid_face >= 1 && raw.source.solid_face <= 6 && raw.raw_role == 1,
            "Initial solid source identity differs");
        out.source.canonical_row = source.canonical_row;
        out.source.source_line = source.source_line;
    } else {
        const bool triangle = raw.source.kind == values::ParentKind::ShellTriangle;
        output::Require(triangle || raw.source.kind == values::ParentKind::ShellQuad,
            "Unknown source face family");
        const auto& map = triangle ? packed.triangle_to_physical : packed.quad_to_physical;
        output::Require(ordinal < map.size() && map[ordinal] < input.shells.size(),
            "Initial shell face has a foreign physical parent");
        const auto& source = input.shells[map[ordinal]];
        output::Require(source.primary.source_id == raw.source.element_id && source.part_id == raw.source.part_id &&
            raw.source.solid_face == 0 && raw.raw_role == (triangle ? 7 : 3), "Initial shell source identity differs");
        out.source.canonical_row = source.canonical_row;
        out.source.source_line = source.source_line;
    }
    return out;
}
}
