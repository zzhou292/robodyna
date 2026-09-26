// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
namespace type25_surface_source_test {
namespace {
constexpr std::size_t Rows = 64, Nodes = 256, Parts = 256, Faces = 512;
std::mutex native_mutex;
extern "C" void rd_surface_source(const int*, const int*, const int*, const int*, const int*,
    const int*, const int*, int*, int*, int*);
void Check(bool value, const char* text) { if (!value) throw std::invalid_argument(text); }
void Node(std::uint32_t node, std::size_t count) { Check(node < count, "Surface oracle node range"); }
template<class T> void Span(const T* data, std::size_t count, std::size_t cap) {
    Check(count <= cap && (!count || data), "Surface oracle array extent");
}
void Shell(const s::Shell& shell, unsigned arity, std::size_t count) {
    Check(shell.element_id && shell.part_id, "Surface oracle shell identity");
    std::set<std::uint32_t> unique;
    for (unsigned k = 0; k < arity; ++k) {
        Node(shell.nodes[k], count);
        unique.insert(shell.nodes[k]);
    }
    Check(unique.size() == arity && (arity != 3 || shell.nodes[2] == shell.nodes[3]),
        "Surface oracle shell raw node contract");
}
}
NativeResult Oracle(const s::Input& input) {
    Check(input.phase == s::ReaderPhase::BeforeGroupingAndInitia && input.node_count > 0 && input.node_count <= Nodes,
        "Surface oracle early-reader profile");
    Span(input.solids, input.solid_count, Rows);
    Span(input.quads, input.quad_count, Rows);
    Span(input.triangles, input.triangle_count, Rows);
    const auto& clause = input.clause;
    Check(clause.kind == s::ClauseKind::Parts || clause.kind == s::ClauseKind::Solids, "Surface oracle clause kind");
    Check(clause.mode == s::SurfaceMode::Exterior || clause.mode == s::SurfaceMode::ExteriorShellEdges ||
        clause.mode == s::SurfaceMode::All, "Surface oracle clause mode");
    Span(clause.part_ids, clause.part_count, Parts);
    Span(clause.solid_rows, clause.solid_row_count, Rows);
    Check(clause.kind == s::ClauseKind::Parts ? clause.solid_row_count == 0 : clause.part_count == 0,
        "Surface oracle unrelated clause payload");
    std::array<int, 8*Rows> snodes{};
    std::array<int, 4*Rows> qnodes{};
    std::array<int, 3*Rows> tnodes{};
    std::array<int, 3*Rows> pids{};
    std::array<int, Parts> selected_parts{};
    std::array<int, Rows> selected_solids{};
    std::map<std::uint64_t, int> part_map;
    for (std::size_t i = 0; i < input.solid_count; ++i) {
        const auto& solid = input.solids[i];
        Check(solid.element_id && solid.part_id, "Surface oracle solid identity");
        Check(solid.topology == s::SolidTopology::Hex8 || solid.topology == s::SolidTopology::DeclaredPenta6,
            "Surface oracle solid topology");
        std::set<std::uint32_t> unique;
        for (unsigned k = 0; k < 8; ++k) {
            Node(solid.nodes[k], input.node_count);
            snodes[8*i+k] = int(solid.nodes[k])+1;
            unique.insert(solid.nodes[k]);
        }
        Check(solid.topology == s::SolidTopology::Hex8 ? unique.size() == 8 :
            unique.size() == 6 && solid.nodes[3] == solid.nodes[0] && solid.nodes[7] == solid.nodes[4],
            "Surface oracle declared H8/PENTA reader slots");
        part_map.emplace(solid.part_id, 0);
    }
    for (std::size_t i = 0; i < input.quad_count; ++i) {
        Shell(input.quads[i], 4, input.node_count);
        part_map.emplace(input.quads[i].part_id, 0);
        for (unsigned k = 0; k < 4; ++k) qnodes[4*i+k] = int(input.quads[i].nodes[k])+1;
    }
    for (std::size_t i = 0; i < input.triangle_count; ++i) {
        Shell(input.triangles[i], 3, input.node_count);
        part_map.emplace(input.triangles[i].part_id, 0);
        for (unsigned k = 0; k < 3; ++k) tnodes[3*i+k] = int(input.triangles[i].nodes[k])+1;
    }
    std::set<std::uint64_t> clause_parts;
    for (std::size_t i = 0; i < clause.part_count; ++i) {
        Check(clause.part_ids[i] && clause_parts.insert(clause.part_ids[i]).second, "Surface oracle unique PART clause");
        part_map.emplace(clause.part_ids[i], 0);
    }
    Check(part_map.size() <= Parts, "Surface oracle part table cap");
    int next_part = 0;
    for (auto& part : part_map) part.second = ++next_part;
    for (std::size_t i = 0; i < input.solid_count; ++i) pids[i] = part_map.at(input.solids[i].part_id);
    for (std::size_t i = 0; i < input.quad_count; ++i) pids[Rows+i] = part_map.at(input.quads[i].part_id);
    for (std::size_t i = 0; i < input.triangle_count; ++i) pids[2*Rows+i] = part_map.at(input.triangles[i].part_id);
    for (std::size_t i = 0; i < clause.part_count; ++i) selected_parts[i] = part_map.at(clause.part_ids[i]);
    for (std::size_t i = 0; i < clause.solid_row_count; ++i) {
        Check(clause.solid_rows[i] < input.solid_count && (i == 0 || clause.solid_rows[i] > clause.solid_rows[i-1]),
            "Surface oracle sorted unique SOLID clause");
        selected_solids[i] = int(clause.solid_rows[i])+1;
    }
    const int counts[]{int(input.node_count), int(input.solid_count), int(input.quad_count), int(input.triangle_count),
        int(part_map.size()), clause.kind == s::ClauseKind::Parts ? 0 : 1, int(clause.mode),
        int(clause.reverse_shell_normals), int(clause.part_count), int(clause.solid_row_count)};
    std::array<int, 8*Faces> faces{};
    std::array<int, Rows> flags{};
    int count = 0;
    {
        std::lock_guard<std::mutex> serial(native_mutex);
        rd_surface_source(counts, snodes.data(), qnodes.data(), tnodes.data(), pids.data(),
            selected_parts.data(), selected_solids.data(), &count, faces.data(), flags.data());
    }
    Check(count >= 0 && std::size_t(count) <= Faces, "Native surface result cap");
    NativeResult result;
    result.faces.reserve(std::size_t(count));
    for (int i = 0; i < count; ++i) {
        const int* row = faces.data()+8*i;
        s::Face face;
        for (unsigned k = 0; k < 4; ++k) {
            Check(row[k] > 0 && std::size_t(row[k]) <= input.node_count, "Native surface result node");
            face.nodes[k] = std::uint32_t(row[k]-1);
        }
        Check(row[5] > 0 && row[7] > 0 && row[7] <= count, "Native surface result source ordinal");
        const auto source = std::size_t(row[5]-1);
        face.raw_role = row[4];
        face.buffer_ordinal = std::uint32_t(row[7]-1);
        face.source.reader_row = std::uint32_t(source);
        if (row[4] == 1) {
            Check(source < input.solid_count && row[6] > 0 && row[6] <= 6, "Native solid surface identity");
            face.source.kind = s::ParentKind::Solid;
            face.source.element_id = input.solids[source].element_id;
            face.source.part_id = input.solids[source].part_id;
            face.source.solid_face = std::uint8_t(row[6]);
        } else {
            const bool triangle = row[4] == 7;
            Check((triangle || row[4] == 3) && row[6] == 0 && source < (triangle ? input.triangle_count : input.quad_count),
                "Native shell surface identity");
            const auto& shell = triangle ? input.triangles[source] : input.quads[source];
            face.source.kind = triangle ? s::ParentKind::ShellTriangle : s::ParentKind::ShellQuad;
            face.source.element_id = shell.element_id;
            face.source.part_id = shell.part_id;
        }
        result.faces.push_back(face);
    }
    result.surface_solid_flags.reserve(input.solid_count);
    for (std::size_t i = 0; i < input.solid_count; ++i) {
        Check(flags[i] == 0 || flags[i] == 1, "Native emitted-solid observation");
        result.surface_solid_flags.push_back(std::uint8_t(flags[i]));
    }
    return result;
}
}
