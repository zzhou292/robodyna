#include "Internal.h"
#include <algorithm>

namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {
namespace {
void Append(Contributor row, const std::uint64_t (&source_nodes)[2],
    const std::size_t (&nodes)[2], const tl::fea::NodalNodeDomain& domain,
    double value, Packed& output) {
    Require(n::coefficient_detail::Finite(value), "Nonfinite prepared direct contact coefficient");
    row.channel = Channel::DirectStiffness;
    row.slots = 2;
    for (unsigned slot = 0; slot < 2; ++slot) {
        Require(nodes[slot] < domain.node_count() && nodes[slot] <= UINT32_MAX &&
            domain.nodes()[nodes[slot]].source_id == source_nodes[slot],
            "Direct contact source endpoint differs from the common physical domain");
        row.nodes[slot] = static_cast<std::uint32_t>(nodes[slot]);
        output.stiffness.push_back({row.nodes[slot], value});
    }
    output.contributors.push_back(row);
}
double Coefficient(const n::NativeSpringNodalInput& input) {
    n::NativeScalarCoefficient result;
    Require(n::EvaluateNativeSpringNodalCoefficient(input, &result) == n::CoefficientStatus::Ok,
        "Native spring contact contribution rejected prepared source operands");
    return result.value;
}
}

void PackDirect(const PhysicalModel& model, const JointModel& joints,
    const ids::Resolution& mapping, n::UnitScale units, Packed& output) {
    Require(mapping.diagnostic.status == ids::Readiness::Ready,
        "Complete native SPRING source order is unavailable");
    const auto factors = Factors(units);
    const auto& domain = model.source_domain().domain();
    const auto* beams = model.structural_beams();
    Require(beams && beams->domain() && beams->domain()->SharesStorage(domain),
        "Complete V5 structural beam contact source is missing or foreign");
    std::vector<std::size_t> beam_order;
    beam_order.reserve(beams->parents().size());
    for (std::size_t i = 0; i < beams->parents().size(); ++i) beam_order.push_back(i);
    std::sort(beam_order.begin(), beam_order.end(), [&](auto a, auto b) {
        return beams->parents()[a].reference.input().source_element_id <
            beams->parents()[b].reference.input().source_element_id;
    });
    std::uint64_t previous = 0;
    for (const auto index : beam_order) {
        const auto& parent = beams->parents()[index];
        const auto& input = parent.reference.input();
        Require(input.source_element_id > previous, "Duplicate or unordered beam source identity");
        previous = input.source_element_id;
        const n::UnitScale beam_units = input.units == tl::fea::beam18::WorkingUnits::TonneMillimetreSecond
            ? n::UnitScale{.001, 1000., 1.} : n::UnitScale{1., 1., 1.};
        Require(SameUnits(beam_units, units), "Beam native contact coefficient working units differ");
        Contributor row;
        row.kind = ContributorKind::Beam18;
        row.original_id = row.native_id = input.source_element_id;
        row.part_id = input.source_part_id;
        row.material_id = input.source_material_id;
        row.source_index = index;
        const std::uint64_t node_ids[2]{input.source_node_id[0], input.source_node_id[1]};
        Append(row, node_ids, parent.domain_nodes, domain,
            parent.reference.native_mass().interface_stiffness, output);
    }

    const auto& type13 = model.beams();
    const auto& type25 = model.welds().model();
    const auto u13 = type13.units();
    const auto u25 = type25.source_units();
    Require(SameUnits({u13.length_to_m, u13.mass_to_kg, u13.time_to_s}, units) &&
        SameUnits({u25.length_to_m, u25.mass_to_kg, u25.time_to_s}, units),
        "Spring native contact working units differ from the complete source");
    std::vector<std::size_t> joint_index(joints.source().data().rows.size(), SIZE_MAX);
    for (std::size_t i = 0; i < joints.source_rows().size(); ++i) {
        const auto source = joints.source_rows()[i];
        Require(source < joint_index.size() && joint_index[source] == SIZE_MAX,
            "Joint physical/source index coverage differs");
        joint_index[source] = i;
    }
    previous = 0;
    for (const auto ordinal : mapping.physical_order) {
        Require(ordinal < mapping.rows.size(), "Native SPRING order index is outside its resolved source");
        const auto& source = mapping.rows[ordinal];
        Require(source.physical_participant && source.native_id > previous,
            "Native SPRING order contains an omitted or repeated record");
        previous = source.native_id;
        Contributor row;
        row.original_id = source.original_id;
        row.native_id = source.native_id;
        row.source_index = source.source_index;
        std::uint64_t node_ids[2]{source.endpoints[0], source.endpoints[1]};
        std::size_t nodes[2]{};
        double value = 0.;
        n::NativeSpringNodalInput input;
        input.interface_initialization = 1; // Admitted declared TYPE25 initialization.
        if (source.kind == ids::SourceKind::Type13) {
            Require(source.source_index < type13.connection_count(), "Native TYPE13 source index differs");
            const auto& connection = type13.connections()[source.source_index];
            const auto* property = type13.property(connection.property);
            const auto* declaration = type13.property_declaration(connection.property);
            const auto* startup = type13.startup(source.source_index);
            Require(connection.source_id == source.original_id && property && declaration && startup &&
                declaration->input.controls.length_normalized == 1,
                "Native TYPE13 prepared property/source binding differs");
            input.kind = n::SpringNodalKind::Type13;
            input.length_mode = 1;
            input.geometric_length = startup->reference.length_native;
            for (unsigned channel = 0; channel < 3; ++channel) {
                const auto& current = property->channel(channel);
                input.translation[channel] = {current.native_stiffness, current.declaration.ordinate_scale};
            }
            for (unsigned slot = 0; slot < 2; ++slot) {
                Require(connection.node[slot] < type13.node_count() &&
                    type13.nodes()[connection.node[slot]].source_id == node_ids[slot],
                    "Native TYPE13 source endpoint differs");
                nodes[slot] = type13.nodes()[connection.node[slot]].global_node;
            }
            row.kind = ContributorKind::Type13;
            value = Coefficient(input);
        } else if (source.kind == ids::SourceKind::DefaultSpotweld) {
            Require(source.source_index < type25.connection_count(), "Native TYPE25 source index differs");
            const auto& connection = type25.connections()[source.source_index];
            Require(connection.source_element_id == source.original_id &&
                connection.property_index < type25.property_count(),
                "Native TYPE25 source/property binding differs");
            input.kind = n::SpringNodalKind::Type25;
            input.length_mode = 0;
            const auto& property = type25.properties()[connection.property_index].property;
            for (unsigned channel = 0; channel < 2; ++channel)
                input.translation[channel] = {property.stiffness[channel] / factors.base.stiffness, 1.};
            for (unsigned slot = 0; slot < 2; ++slot) {
                Require(connection.source_node_id[slot] == node_ids[slot], "Native TYPE25 source endpoint differs");
                nodes[slot] = connection.global_node[slot];
            }
            row.kind = ContributorKind::Type25;
            value = Coefficient(input);
        } else {
            Require(source.kind == ids::SourceKind::RegularJoint && source.source_index < joint_index.size() &&
                joint_index[source.source_index] < joints.model().joints().size(),
                "Native joint source index is omitted or unsupported");
            const auto& joint = joints.model().joints()[joint_index[source.source_index]];
            Require(joint.geometry.source_joint_id == source.original_id,
                "Native joint source/model identity differs");
            for (unsigned slot = 0; slot < 2; ++slot) {
                Require(joint.geometry.source_node_id[slot] == node_ids[slot], "Native joint source endpoint differs");
                nodes[slot] = joint.domain_nodes[slot];
            }
            row.kind = ContributorKind::Type45;
            // Selected converter PARGEO2=ZERO -> original generic GEO3 store
            // -> RINIT3 STR. Automatic Kn is separate and is never consumed.
            value = 0.;
        }
        Append(row, node_ids, nodes, domain, value, output);
    }
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail
