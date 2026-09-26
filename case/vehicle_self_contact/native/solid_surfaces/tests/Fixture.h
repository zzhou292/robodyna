#pragma once
#include "../Internal.h"
#include "lib_utest/qualification/radioss_type25_surface_source/Fixture.h"
namespace crash::cases::vehicle_self_contact::native::initial_surfaces::test {
namespace leaf = ::type25_surface_source_test;
inline coated::Inputs Geometry(const leaf::Case& c) {
    coated::Inputs out;
    out.units = {0.001, 1000, 1};
    for (std::size_t i = 0; i < c.nodes; ++i)
        out.nodes.push_back({1000+i, std::uint32_t(i), {double(i), 0, 0}});
    for (const auto& solid : c.solids) {
        coated::Solid row;
        row.source_id = solid.element_id;
        row.part_id = solid.part_id;
        row.canonical_row = std::uint32_t(solid.element_id);
        row.source_line = std::uint32_t(solid.element_id + 2000);
        row.kind = solid.topology == values::SolidTopology::DeclaredPenta6 ?
            coated::ReaderKind::DeclaredPenta6 : coated::ReaderKind::Hex8;
        std::copy(std::begin(solid.nodes), std::end(solid.nodes), row.nodes.begin());
        out.solids.push_back(row);
    }
    const auto add = [&](const auto& rows, coated::n::ShellLayout layout) {
        for (const auto& shell : rows) {
            coated::Shell row;
            row.primary.source_id = shell.element_id;
            row.primary.layout = layout;
            std::copy(std::begin(shell.nodes), std::end(shell.nodes), row.primary.nodes);
            row.part_id = shell.part_id;
            row.canonical_row = std::uint32_t(shell.element_id);
            row.source_line = std::uint32_t(shell.element_id + 3000);
            row.physical_parent = std::uint32_t(out.shells.size());
            row.contact_selected = std::find(c.parts.begin(), c.parts.end(), shell.part_id) != c.parts.end();
            out.shells.push_back(row);
        }
    };
    add(c.quads, coated::n::ShellLayout::Quad4);
    add(c.triangles, coated::n::ShellLayout::Triangle3);
    return out;
}
struct Extracted {
    tl::util::HostArena output, scratch;
    values::Snapshot snapshot;
    explicit Extracted(const values::Input& in) {
        values::Forecast forecast;
        if (values::Preflight(in, {}, forecast).status != values::Status::Ok ||
            !output.Initialize(forecast.output_bytes) || !scratch.Initialize(forecast.scratch_bytes) ||
            values::Build(in, {}, output, scratch, &snapshot).status != values::Status::Ok)
            throw std::runtime_error("Source certificate fixture extraction failed");
    }
};
inline Report Membership(const leaf::Case& c, Certificate& certificate) {
    const auto geometry = Geometry(c);
    const auto packed = detail::Pack(geometry, c.parts);
    const Extracted probe(packed.Input(true));
    return detail::CertifyMembership(packed, probe.snapshot, certificate);
}
inline std::vector<Face> Faces(const leaf::Case& c, Certificate& certificate) {
    const auto geometry = Geometry(c);
    const auto packed = detail::Pack(geometry, c.parts);
    const Extracted probe(packed.Input(true));
    if (detail::CertifyMembership(packed, probe.snapshot, certificate).status != Status::Ready)
        throw std::runtime_error("Source certificate fixture has ambiguous membership");
    const Extracted actual(packed.Input(false));
    std::vector<Face> faces;
    for (std::size_t i = 0; i < actual.snapshot.face_count; ++i)
        faces.push_back(detail::ExternalFace(actual.snapshot.faces[i], packed, geometry));
    return faces;
}
inline void Same(const Certificate& a, const Certificate& b) {
    EXPECT_EQ(a.queried_solid_faces, b.queried_solid_faces);
    EXPECT_EQ(a.matching_physical_shells, b.matching_physical_shells);
    EXPECT_EQ(a.equal_node_key_groups, b.equal_node_key_groups);
    EXPECT_EQ(a.membership_complete, b.membership_complete);
    EXPECT_EQ(a.sort_order_complete, b.sort_order_complete);
}
inline void Same(const Face& a, const Face& b) {
    EXPECT_EQ(a.source.kind, b.source.kind);
    EXPECT_EQ(a.source.element_id, b.source.element_id);
    EXPECT_EQ(a.source.part_id, b.source.part_id);
    EXPECT_EQ(a.source.canonical_row, b.source.canonical_row);
    EXPECT_EQ(a.source.source_line, b.source.source_line);
    EXPECT_EQ(a.source.solid_face, b.source.solid_face);
    EXPECT_EQ(a.nodes, b.nodes);
    EXPECT_EQ(a.raw_role, b.raw_role);
}
}
