#include "Source.h"
#include <array>
#include <algorithm>
#include <utility>
#include <vector>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
TEST(VehicleSelfContactSource, EveryCenteredV5ParentRetainsNativeNodesThicknessAndRole) {
    const auto& physical = Execution().physical();
    const auto& selected = Inventory();
    const auto& surface = Surface();
    ASSERT_EQ(physical.domain()->node_count(),376930);
    ASSERT_EQ(physical.catalog()->parent_count(),349645);
    ASSERT_EQ(selected.centered.size()+selected.excluded.size(),349645);
    ASSERT_EQ(surface.parents().size(),selected.centered.size());
    ASSERT_TRUE(surface.MatchesPhysical(physical));
    std::array<std::size_t,5> roles{};
    // Independent sort/unique oracle, with no per-feature tree allocations.
    // The admitted 524288-parent bound proves these three reserve products fit
    // size_t and at most 52 MiB of value payload, before any reserve call.
    ASSERT_LE(selected.centered.size(), contact::SelfContactSurfaceLimits::Vehicle().max_parents);
    const auto corner_capacity = 4 * selected.centered.size();
    const auto validation_reservation = selected.centered.size()*sizeof(std::uint64_t) +
        corner_capacity*(sizeof(std::uint64_t) + sizeof(std::pair<std::uint64_t,std::uint64_t>));
    ASSERT_LE(validation_reservation, std::size_t{64} << 20);
    std::vector<std::uint64_t> pids, nids;
    std::vector<std::pair<std::uint64_t,std::uint64_t>> edges;
    pids.reserve(selected.centered.size());
    nids.reserve(corner_capacity);
    edges.reserve(corner_capacity);
    std::size_t uses = 0;
    for (std::size_t i = 0; i < selected.centered.size(); ++i) {
        const auto& original = *physical.catalog()->parent(selected.centered[i].catalog_row);
        const auto& actual = surface.parents()[i];
        const auto native = Native(physical,original);
        ASSERT_EQ(actual.source.family,original.family);
        ASSERT_EQ(actual.source.family_index,original.family_index);
        ASSERT_EQ(actual.source.source_parent_id,original.source_parent_id);
        ASSERT_EQ(actual.source.source_part_id,original.source_part_id);
        ASSERT_EQ(actual.source.section_id,original.section_id);
        ASSERT_EQ(actual.source.material_id,original.material_id);
        ASSERT_TRUE(native.centered);
        ASSERT_EQ(actual.arity,native.arity);
        if (native.arity == 4) {
            ASSERT_EQ(actual.q4.feature_id, original.source_parent_id);
            ASSERT_EQ(actual.q4.parent_element_id, original.source_parent_id);
            ASSERT_EQ(actual.q4.parent_face_id, 0u);
            ASSERT_EQ(actual.q4.half_thickness, 0.0);
        } else {
            ASSERT_EQ(actual.t3.feature_id, original.source_parent_id);
            ASSERT_EQ(actual.t3.parent_element_id, original.source_parent_id);
            ASSERT_EQ(actual.t3.parent_face_id, 0u);
            ASSERT_EQ(actual.t3.half_thickness, 0.0);
            ASSERT_EQ(actual.t3.interpolation, contact::SurfaceInterpolation::kLinearTriangle);
        }
        ASSERT_EQ(output::Bits(actual.reference_half_thickness_m),output::Bits(.5*native.thickness));
        unsigned points = 99;
        fe::ShellSectionLaw law;
        ASSERT_TRUE(physical.catalog()->MaterialPointCount(original.family,original.family_index,&points));
        ASSERT_TRUE(physical.catalog()->Law(original.family,original.family_index,&law));
        ASSERT_EQ(actual.material_points,points);
        ASSERT_EQ(actual.law,law);
        ASSERT_LT(points,roles.size());
        ++roles[points];
        pids.push_back(original.source_part_id);
        for (unsigned j = 0; j < native.arity; ++j) {
            const auto domain = physical.mapping()->owner_index(native.nodes[j]);
            const auto actual_node = actual.arity == 4 ? actual.q4.nodes[j] : actual.t3.nodes[j];
            ASSERT_EQ(actual_node,domain);
            ASSERT_LT(domain, physical.domain()->node_count());
            const auto nid = physical.domain()->nodes()[domain].source_id;
            ASSERT_EQ(nid, NativeNodeId(physical, original, j));
            ASSERT_LT(actual.vertices[j], surface.vertices().size());
            ASSERT_EQ(surface.vertices()[actual.vertices[j]].source_node_id,nid);
            ASSERT_EQ(surface.vertices()[actual.vertices[j]].domain_node, domain);
            nids.push_back(nid);
            const auto other = physical.mapping()->owner_index(native.nodes[(j+1)%native.arity]);
            ASSERT_LT(other, physical.domain()->node_count());
            const auto other_nid = physical.domain()->nodes()[other].source_id;
            const std::pair<std::uint64_t,std::uint64_t> pair{std::min(nid,other_nid),std::max(nid,other_nid)};
            ASSERT_LT(actual.edges[j], surface.edges().size());
            const auto& edge = surface.edges()[actual.edges[j]];
            ASSERT_EQ(edge.source_node_ids[0],pair.first);
            ASSERT_EQ(edge.source_node_ids[1],pair.second);
            edges.push_back(pair);
            ++uses;
        }
    }
    const auto unique = [](auto& values) {
        std::sort(values.begin(), values.end());
        values.erase(std::unique(values.begin(), values.end()), values.end());
    };
    unique(pids);
    unique(nids);
    unique(edges);
    EXPECT_EQ(roles[2], 0u);
    EXPECT_EQ(surface.vertices().size(),nids.size());
    EXPECT_EQ(surface.edges().size(),edges.size());
    EXPECT_EQ(surface.vertex_uses().size(),uses);
    EXPECT_EQ(surface.edge_uses().size(),uses);
    std::size_t vertex_uses = 0, edge_uses = 0;
    for (std::size_t i = 0; i < surface.vertices().size(); ++i) {
        const auto& vertex = surface.vertices()[i];
        ASSERT_GT(vertex.use_count, 0u);
        ASSERT_EQ(vertex.use_offset, vertex_uses);
        ASSERT_LE(vertex.use_count, surface.vertex_uses().size()-vertex_uses);
        for (std::size_t j = 0; j < vertex.use_count; ++j) {
            const auto& use = surface.vertex_uses()[vertex_uses++];
            ASSERT_LT(use.parent, surface.parents().size());
            const auto& parent = surface.parents()[use.parent];
            ASSERT_LT(use.local, parent.arity);
            ASSERT_EQ(use.source_parent_id, parent.source.source_parent_id);
            ASSERT_EQ(use.source_node_id, vertex.source_node_id);
            ASSERT_EQ(use.domain_node, vertex.domain_node);
            ASSERT_EQ(parent.vertices[use.local], i);
        }
    }
    for (std::size_t i = 0; i < surface.edges().size(); ++i) {
        const auto& edge = surface.edges()[i];
        ASSERT_GT(edge.use_count, 0u);
        ASSERT_EQ(edge.use_offset, edge_uses);
        ASSERT_LE(edge.use_count, surface.edge_uses().size()-edge_uses);
        for (std::size_t j = 0; j < edge.use_count; ++j) {
            const auto& use = surface.edge_uses()[edge_uses++];
            ASSERT_LT(use.parent, surface.parents().size());
            const auto& parent = surface.parents()[use.parent];
            ASSERT_LT(use.local, parent.arity);
            ASSERT_EQ(use.source_parent_id, parent.source.source_parent_id);
            ASSERT_EQ(use.source_node_ids[0], edge.source_node_ids[0]);
            ASSERT_EQ(use.source_node_ids[1], edge.source_node_ids[1]);
            ASSERT_EQ(parent.edges[use.local], i);
        }
    }
    EXPECT_EQ(vertex_uses, uses);
    EXPECT_EQ(edge_uses, uses);
    ASSERT_EQ(surface.faces().size(), selected.centered.size());
    for (auto face : surface.faces()) ASSERT_LT(face, surface.parents().size());
    for (std::size_t i = 1; i < surface.faces().size(); ++i)
        ASSERT_LT(surface.parents()[surface.faces()[i-1]].source.source_parent_id,
                  surface.parents()[surface.faces()[i]].source.source_parent_id);
    RecordProperty("profile","FrictionlessReferenceThicknessShellSubsetV1");
    RecordProperty("selection_policy","All centered selected shell parents; no original contact-set parity");
    RecordProperty("selected_parents",std::to_string(selected.centered.size()));
    RecordProperty("excluded_offset_parents",std::to_string(selected.excluded.size()));
    RecordProperty("selected_parts",std::to_string(pids.size()));
    RecordProperty("unique_source_vertices",std::to_string(nids.size()));
    RecordProperty("unique_source_edges",std::to_string(edges.size()));
    RecordProperty("parent_corner_uses",std::to_string(uses));
    RecordProperty("validation_vector_reserved_bytes", std::to_string(pids.capacity()*sizeof(pids[0]) +
        nids.capacity()*sizeof(nids[0]) + edges.capacity()*sizeof(edges[0])));
    for (unsigned n : {0u,1u,3u,4u}) RecordProperty("material_points_"+std::to_string(n),std::to_string(roles[n]));
    RecordProperty("startup_payload_bytes",std::to_string(surface.forecast().startup_payload_bytes));
    RecordProperty("arena_bytes",std::to_string(surface.forecast().arena_bytes));
    RecordProperty("retained_source_bytes",std::to_string(surface.forecast().retained_source_bytes));
}
TEST(VehicleSelfContactSource, EveryExcludedParentIsExplicitlyRejectedByNativeSurfacePolicy) {
    const auto& physical = Execution().physical();
    for (const auto& p : Inventory().excluded) {
        const contact::SelfContactSurfaceInput input{&p,1};
        const auto report = contact::SelfContactSurfaceBinding::Preflight(physical,input,contact::SelfContactSurfaceLimits::Vehicle()).report;
        ASSERT_EQ(report.status,contact::SelfContactSurfaceStatus::UnsupportedReferencePlane) << p.source_parent_id;
    }
    RecordProperty("excluded_offset_parents",std::to_string(Inventory().excluded.size()));
}
} // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
