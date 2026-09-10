#include "JsonReader.h"
#include <set>

namespace crash::modelio::assembly::reader {
namespace {
Node ReadNode(const Value& value) {
    Node node;
    node.source_id = Unsigned(value, "source_id", INT64_MAX);
    node.source_line = Unsigned(value, "source_line", SIZE_MAX);
    node.canonical_index = Unsigned(value, "canonical_index", SIZE_MAX);
    node.blank_mask = Unsigned(value, "blank_mask", 255);
    const auto& codes = Array(value, "codes", 2, 2);
    node.codes = {unsigned(Unsigned(codes[0], std::uint64_t{0})), unsigned(Unsigned(codes[1], std::uint64_t{0}))};
    const auto& p = Array(value, "position_m", 3, 3);
    node.position_m = {Real(p[0]), Real(p[1]), Real(p[2])};
    Require(node.source_id && node.source_line && node.blank_mask == 48, "Unsupported source node declaration");
    return node;
}
void SameNode(const Node& a, const Node& b) {
    Require(a.source_id == b.source_id && a.source_line == b.source_line && a.canonical_index == b.canonical_index &&
        a.blank_mask == b.blank_mask && a.codes == b.codes, "Shared source node metadata mismatch");
    Same(a.position_m.x, b.position_m.x); Same(a.position_m.y, b.position_m.y); Same(a.position_m.z, b.position_m.z);
}
void BindParent(const Value& value, Parent& p, const Data& data) {
    Require(Unsigned(value, "parent_index", SIZE_MAX) == p.index && Unsigned(value, "part_index", SIZE_MAX) == p.part_index &&
        Unsigned(value, "part_parent_index", SIZE_MAX) == p.part_parent_index &&
        Unsigned(value, "source_element_id") == p.source_id && Unsigned(value, "source_part_id") == p.part_id,
        "Assembly parent traversal or identity mismatch");
    p.family = p.arity == 4 ? ShellFamily::Qeph : ShellFamily::T3;
    TextIs(value, "family", p.arity == 4 ? "QEPH" : "T3");
    p.family_index = Unsigned(value, "family_index", SIZE_MAX);
    p.material_index = Unsigned(value, "material_index", data.materials.size() - 1);
    p.section_index = Unsigned(value, "section_index", data.sections.size() - 1);
    const auto& curve_index = Member(value, "curve_index");
    const auto& material=data.materials[p.material_index];
    const bool no_curve=material.law==MaterialLaw::LayeredLaw1 || material.hardening==MaterialHardening::LinearLaw44;
    if (no_curve) {
        Require(data.schema != InventorySchema && curve_index.IsNull(), "Non-tabulated parent must have no curve index");
        p.curve_index = NoCurveIndex;
    } else {
        Require(!data.curves.empty(), "Tabulated parent requires a real curve table");
        p.curve_index = Unsigned(curve_index, data.curves.size() - 1);
    }
    p.material_id = Unsigned(value, "source_material_id"); p.section_id = Unsigned(value, "source_section_id");
    p.curve_id = Unsigned(value, "source_curve_id"); p.source_elform = Unsigned(value, "source_elform", 16);
    const auto& part = data.parts[p.part_index];
    Require(p.material_id == part.material_id && p.section_id == part.section_id &&
        p.material_id == data.materials[p.material_index].id && p.section_id == data.sections[p.section_index].id &&
        (no_curve ? p.curve_id == 0 : p.curve_id == data.curves[p.curve_index].id) &&
        p.curve_id == data.materials[p.material_index].curve_id &&
        p.source_elform == data.sections[p.section_index].source_elform, "Assembly parent declaration mapping mismatch");
    const auto& nodes = Array(value, "node_indices", p.arity, p.arity);
    for (unsigned n = 0; n < p.arity; ++n)
        Require(Unsigned(nodes[n], data.nodes.size() - 1) == p.nodes[n], "Assembly global node mapping mismatch");
}
}  // namespace
void ReadGeometry(const Value& document, const ReadLimits& limits, Data& data) {
    const auto& geometry = Member(document, "geometry");
    const auto node_ids = Ids(Member(geometry, "source_node_ids"), limits.nodes, true);
    Require(!node_ids.empty(), "Assembly has no source nodes");
    data.nodes.resize(node_ids.size());
    for (std::size_t i = 0; i < node_ids.size(); ++i) data.nodes[i].source_id = node_ids[i];
    std::vector<unsigned> references(node_ids.size(), 0), incidence(node_ids.size(), 0);
    const auto& parts = Array(geometry, "parts", data.parts.size(), data.parts.size());
    const auto& bindings = Array(document, "parent_bindings", limits.parents, 1);
    std::set<SourceId> parent_ids;
    std::set<std::size_t> parent_canonical;
    for (unsigned i = 0; i < parts.Size(); ++i) {
        auto& part = data.parts[i]; const auto& value = parts[i];
        Require(Unsigned(value, "part_id") == part.id && Unsigned(value, "material_id") == part.material_id &&
            Unsigned(value, "section_id") == part.section_id, "Geometry part/declaration mismatch");
        TextIs(value, "member_sha256", data.member_sha256); TextIs(value, "archive_sha256", data.archive_sha256);
        TextIs(value, "source_frame", "Untransformed original yaris-coarse-v1l.key coordinates");
        part.canonical_manifest_sha256 = Text(value, "canonical_manifest_sha256");
        SourceId previous = 0;
        for (const auto& row : Array(value, "nodes", limits.nodes, 3).GetArray()) {
            const auto node = ReadNode(row); Require(node.source_id > previous, "Repeated/unordered part node"); previous = node.source_id;
            const auto index = NodeIndex(data, node.source_id);
            if (references[index]++) SameNode(data.nodes[index], node); else data.nodes[index] = node;
            part.nodes.push_back(index);
        }
        part.first_parent = data.parents.size();
        for (const auto& row : Array(value, "shells", limits.parents, 1).GetArray()) {
            Require(data.parents.size() < bindings.Size(), "Extra assembly shell without parent binding");
            Parent parent;
            parent.source_id = Unsigned(row, "source_id", INT64_MAX); parent.part_id = part.id;
            parent.index = data.parents.size(); parent.part_index = i;
            parent.part_parent_index = parent.index - part.first_parent;
            parent.source_line = Unsigned(row, "source_line", SIZE_MAX);
            parent.canonical_index = Unsigned(row, "canonical_index", SIZE_MAX);
            parent.arity = Unsigned(row, "arity", 4); parent.blank_mask = Unsigned(row, "blank_mask", 255);
            Require(parent.source_id && parent.source_line && parent_ids.insert(parent.source_id).second &&
                parent_canonical.insert(parent.canonical_index).second && (parent.arity == 3 || parent.arity == 4) &&
                parent.blank_mask == 0, "Invalid/duplicate source shell metadata");
            const auto& record = Array(row, "raw_record", 6, 6);
            const auto& local = Array(row, "local_node_indices", 4, 4);
            const auto& canonical = Array(row, "canonical_node_indices", 4, 4);
            for (unsigned n = 0; n < 6; ++n) parent.raw_record[n] = Unsigned(record[n], INT64_MAX);
            Require(parent.raw_record[0] == parent.source_id && parent.raw_record[1] == part.id,
                "Raw source shell record identity mismatch");
            std::set<std::size_t> unique;
            for (unsigned n = 0; n < 4; ++n) {
                parent.part_local_nodes[n] = Unsigned(local[n], part.nodes.size() - 1);
                parent.nodes[n] = part.nodes[parent.part_local_nodes[n]];
                parent.canonical_nodes[n] = Unsigned(canonical[n], SIZE_MAX);
                const auto& node = data.nodes[parent.nodes[n]];
                Require(node.source_id == parent.raw_record[n + 2] && node.canonical_index == parent.canonical_nodes[n],
                    "Raw source shell connectivity mismatch");
                if (n < parent.arity) {
                    Require(unique.insert(parent.nodes[n]).second, "Repeated active node inside native shell");
                    ++incidence[parent.nodes[n]];
                }
            }
            if (parent.arity == 3) Require(parent.raw_record[4] == parent.raw_record[5], "Native triangle must preserve repeated fourth slot");
            BindParent(bindings[static_cast<rapidjson::SizeType>(parent.index)], parent, data);
            auto& next_family = parent.arity == 4 ? data.qeph_count : data.t3_count;
            Require(parent.family_index == next_family++, "Family traversal index changed");
            data.parents.push_back(std::move(parent));
        }
        part.parent_count = data.parents.size() - part.first_parent;
    }
    Require(data.parents.size() == bindings.Size(), "Unbound assembly parent inventory");
    std::set<std::size_t> canonical_nodes;
    std::vector<SourceId> shared;
    for (std::size_t i = 0; i < node_ids.size(); ++i) {
        Require(references[i] && incidence[i] && canonical_nodes.insert(data.nodes[i].canonical_index).second,
                "Missing/orphaned/duplicate canonical source node");
        if (references[i] > 1) shared.push_back(node_ids[i]);
    }
    Require(shared == Ids(Member(geometry, "shared_node_ids"), limits.nodes, true), "Shared-node inventory mismatch");
    Require(Unsigned(geometry, "shell_count") == data.parents.size() && Unsigned(geometry, "qeph_parent_count") == data.qeph_count &&
        Unsigned(geometry, "t3_parent_count") == data.t3_count, "Geometry counts do not match retained parents");
}
}  // namespace crash::modelio::assembly::reader
