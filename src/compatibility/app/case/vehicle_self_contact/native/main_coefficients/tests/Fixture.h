#pragma once
#include "../Compose.h"
#include "NativeOracle.h"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
struct Fixture {
    coated::Inputs input;
    detail::Packed packed;
    std::vector<modelio::assembly::Material> materials;
    std::vector<modelio::assembly::Section> sections;
    Fixture(bool triangle = false) : materials(3), sections(3) {
        input.nodes = {{1, 0, {0,0,0}}, {2, 1, {1,0,0}}, {3, 2, {1,1,0}}, {4, 3, {0,1,0}}};
        for (unsigned i = 0; i < 3; ++i) {
            auto& material = materials[i];
            material.id = 20 + i;
            material.source.keyword = "*MAT_PIECEWISE_LINEAR_PLASTICITY";
            modelio::assembly::DeclarationCard mc;
            mc.names = {"mid", "ro", "e", "pr"};
            mc.values = {double(material.id), 1., 10., .3};
            material.cards = {mc};
            auto& section = sections[i];
            section.id = 30 + i;
            section.source.keyword = "*SECTION_SHELL";
            section.source_elform = 2;
            section.through_thickness_points = 3;
            auto sc = mc;
            sc.names = {"secid", "elform", "shrf", "nip"};
            sc.values = {double(section.id), 2., .833333, 3.};
            auto thickness = mc;
            thickness.names = {"t1", "t2", "t3", "t4", "nloc"};
            thickness.values = {1., 1., 1., 1., double(i)-1.};
            section.cards = {sc, thickness};
            detail::PartValue part;
            part.pid = 100 + i;
            part.sid = section.id;
            part.mid = material.id;
            part.material = &material;
            part.section = &section;
            part.native_material_id_preserved = true;
            part.ordinary_part_controls = true;
            part.coefficient.property_type = 1;
            part.coefficient.property_thickness = 1.;
            part.coefficient.young = 10.;
            packed.parts.push_back(part);
            coated::Shell shell;
            shell.primary.source_id = 1000 + i;
            shell.primary.layout = triangle ? n::ShellLayout::Triangle3 : n::ShellLayout::Quad4;
            const unsigned nodes[]{0,1,2,triangle ? 2u : 3u};
            std::copy_n(nodes, 4, shell.primary.nodes);
            shell.part_id = part.pid;
            shell.physical_parent = i;
            shell.contact_selected = i == 0;
            input.shells.push_back(shell);
            packed.shells.push_back({i});
        }
        Refresh();
    }
    void Refresh() {
        packed.keys.clear();
        for (std::size_t i = 0; i < input.shells.size(); ++i) packed.keys.push_back(detail::Key(input.shells[i], i));
        std::sort(packed.keys.begin(), packed.keys.end(), detail::KeyLess);
    }
    coated::s::Main Main() const {
        coated::s::Main main;
        std::copy_n(input.shells[0].primary.nodes, 4, main.nodes);
        return main;
    }
    detail::SupportSelection Select(bool context = true) const {
        return detail::SelectSupport(input, packed, Main(), 0, context);
    }
    std::vector<NativeCandidate> Native(const std::vector<unsigned>& order) const {
        std::vector<NativeCandidate> out;
        for (const auto i : order) {
            NativeCandidate c;
            c.layout = input.shells[i].primary.layout;
            std::copy_n(input.shells[i].primary.nodes, 4, c.nodes.begin());
            c.thickness = packed.parts[i].coefficient.property_thickness;
            c.young = packed.parts[i].coefficient.young;
            out.push_back(c);
        }
        return out;
    }
    std::size_t Oracle(const std::vector<unsigned>& order) const {
        std::array<unsigned, 4> face;
        std::copy_n(Main().nodes, 4, face.begin());
        const auto result = NativeSupport(Native(order), face);
        return result.selected == SIZE_MAX ? SIZE_MAX : order[result.selected];
    }
};
inline std::vector<unsigned> StorageOrder(const Fixture& f) {
    std::vector<std::array<std::uint32_t, 8>> keys;
    for (const auto& part : f.packed.parts)
        keys.push_back({0, 0, 7, 11, std::uint32_t(part.mid), std::uint32_t(part.pid), 0, 0});
    return NativeOrder(keys);
}
}
