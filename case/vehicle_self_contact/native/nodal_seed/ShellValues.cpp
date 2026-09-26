#include "Internal.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
#include "lib_src/math/ScalarBits.h"
#include <algorithm>

namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {
namespace {
template<unsigned Slots, class Input>
void Append(const vehicle_startup::ReferenceRow& source, std::size_t source_index,
    const Input& input, const tl::fea::NodalNodeDomain& domain,
    const n::coefficient_detail::UnitFactors& factors, Packed& output) {
    shells::PhysicalShell shell;
    shell.source_element_id = source.element_id;
    shell.layout = Slots == 4 ? n::ShellLayout::Quad4 : n::ShellLayout::Triangle3;
    shell.young = input.young_modulus / factors.pressure;
    shell.structural_thickness = input.thickness / factors.base.length;
    // This call publishes nodal ASSTIFI fields only, with no primary/secondary
    // rows. These gap-only channels are deliberately not a source gap product.
    shell.property_thickness = shell.structural_thickness;

    Contributor row;
    row.kind = Slots == 4 ? ContributorKind::ShellQ4 : ContributorKind::ShellT3;
    row.channel = Channel::ShellAverage;
    row.original_id = source.element_id;
    row.native_id = source.element_id;
    row.part_id = source.part_id;
    row.material_id = source.material_id;
    row.source_index = source_index;
    row.slots = Slots;
    for (unsigned slot = 0; slot < Slots; ++slot) {
        const auto node = domain.Find(input.node_ids[slot]);
        Require(node < domain.node_count() && node <= UINT32_MAX,
            "Physical shell source node is absent from the common contact domain");
        const auto& position = domain.nodes()[node].position;
        Require(tl::math::SameScalarBits(position.x, input.position[slot].x) &&
            tl::math::SameScalarBits(position.y, input.position[slot].y) &&
            tl::math::SameScalarBits(position.z, input.position[slot].z),
            "Physical shell contact reference coordinates differ from the common domain");
        shell.nodes[slot] = static_cast<std::uint32_t>(node);
        row.nodes[slot] = static_cast<std::uint32_t>(node);
    }
    if constexpr (Slots == 3) shell.nodes[3] = shell.nodes[2];
    output.contributors.push_back(row);
    output.shells.push_back(shell);
}
}

void PackShells(const PhysicalModel& model, n::UnitScale units, Packed& output) {
    const auto factors = Factors(units);
    const auto& references = model.shell_source().references();
    const auto& rows = references.rows();
    const auto& domain = model.source_domain().domain();
    Require(references.resolution(), "Complete physical shell source resolution is missing");
    std::vector<std::size_t> order;
    order.reserve(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i) order.push_back(i);
    const auto family = [&](std::size_t i) {
        return rows[i].family == vehicle_startup::ReferenceFamily::T3 ? 1 : 0;
    };
    std::sort(order.begin(), order.end(), [&](auto a, auto b) {
        if (family(a) != family(b)) return family(a) < family(b);
        return rows[a].element_id < rows[b].element_id;
    });
    for (const auto index : order) {
        const auto& row = rows[index];
        const auto* material = references.resolution()->material(row.part_index);
        const auto* section = references.resolution()->section(row.part_index);
        Require(row.status == vehicle_startup::ReferenceStatus::Success &&
            material && section && material->id == row.material_id && section->id == row.section_id,
            "Complete contact shell material/section identity differs");
        if (const auto* q = references.qeph(index)) {
            Require(tl::math::SameScalarBits(q->input.young_modulus, material->young_pa),
                "Contact Q4 modulus differs from the authenticated physical material");
            Append<4>(row, index, q->input, domain, factors, output);
        } else if (const auto* t = references.t3(index)) {
            Require(tl::math::SameScalarBits(t->input.young_modulus, material->young_pa),
                "Contact T3 modulus differs from the authenticated physical material");
            Append<3>(row, index, t->input, domain, factors, output);
        } else {
            const auto* b = references.qbat(index);
            Require(b && tl::math::SameScalarBits(b->input().quadrilateral.young_modulus, material->young_pa),
                "Contact QBAT modulus differs from the authenticated physical material");
            Append<4>(row, index, b->input().quadrilateral, domain, factors, output);
        }
    }
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail
