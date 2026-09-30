#include "Internal.h"
#include "ElementRows.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
namespace {
template<class Parent> void Solid(Relations& output, Kind kind, std::size_t row,
                                 const Parent& parent,
                                 tl::util::ConstView<tl::fea::NodalDomainNode> domain) {
    const auto& input = parent.reference.input();
    AppendElementInput(output,kind,row,input,domain,parent.domain_nodes);
}
}
void VisitElements(const VehiclePhysicalAttachments& source, Relations& output) {
    const auto& physical = source.physical();
    const auto& solid_source = physical.source_domain().source().solid_source().data();
    const auto& solids = physical.solids();
    const auto domain = physical.source_domain().domain().nodes();
    for (std::size_t row = 0; row < solid_source.rows.size(); ++row) {
        const auto& declaration = solid_source.rows[row];
        const auto index = declaration.reference_index;
        using Family = modelio::solid_source::Family;
        if (declaration.family == Family::Solid18) {
            Require(index < solids.solid18().size(), "Connectivity Solid18 reference index is absent");
            const auto& parent = solids.solid18()[index];
            Require(parent.reference.input().source_element_id == declaration.element_id &&
                parent.reference.input().source_part_id == declaration.part_id, "Connectivity Solid18 source identity differs");
            Solid(output,Kind::Solid18,row,parent,domain);
        } else if (declaration.family == Family::Solid24) {
            Require(index < solids.solid24().size(), "Connectivity Solid24 reference index is absent");
            const auto& parent = solids.solid24()[index];
            Require(parent.reference.input().source_element_id == declaration.element_id &&
                parent.reference.input().source_part_id == declaration.part_id, "Connectivity Solid24 source identity differs");
            Solid(output,Kind::Solid24,row,parent,domain);
        }
        else if (declaration.family == Family::Solid18Law44) {
            Require(index < solids.solid18_law44().size(), "Connectivity LAW44 reference index is absent");
            const auto& parent = solids.solid18_law44()[index];
            Require(parent.reference.input().source_element_id == declaration.element_id &&
                parent.reference.input().source_part_id == declaration.part_id,
                "Connectivity LAW44 source identity differs");
            Solid(output,Kind::Solid18Law44,row,parent,domain);
        } else if (declaration.family == Family::Solid18Law90) {
            Require(index < solids.solid18_law90().size(), "Connectivity LAW90 reference index is absent");
            const auto& parent = solids.solid18_law90()[index];
            Require(parent.reference.input().source_element_id == declaration.element_id &&
                parent.reference.input().source_part_id == declaration.part_id,
                "Connectivity LAW90 source identity differs");
            Solid(output,Kind::Solid18Law90,row,parent,domain);
        }
        else {
            Require(declaration.family == Family::Solid6z, "Connectivity solid family is unavailable");
            Require(index < solids.solid6z().size(), "Connectivity S6Z reference index is absent");
            const auto& parent = solids.solid6z()[index];
            Require(parent.reference.input().source_element_id == declaration.element_id &&
                parent.reference.input().source_part_id == declaration.part_id, "Connectivity S6Z source identity differs");
            Require(parent.domain_nodes[6] == SIZE_MAX && parent.domain_nodes[7] == SIZE_MAX,
                    "Connectivity S6Z has noncanonical tail slots");
            Solid(output,Kind::Solid6z,row,parent,domain);
        }
    }
    const auto& beams = physical.beams();
    const auto& beam_source = physical.source_domain().source().type13_source().data();
    Require(beam_source.beams.size() == beams.connection_count(), "Connectivity TYPE13 source/model count differs");
    const auto endpoint = physical.coefficients().type13()->records();
    for (std::size_t row = 0; row < beams.connection_count(); ++row) {
        const auto& connection = beams.connections()[row];
        Require(connection.source_id == beam_source.beams[row].id,
                "Connectivity TYPE13 source order differs");
        const std::size_t nodes[]{endpoint[2*row].value.global_node,endpoint[2*row+1].value.global_node};
        // The source reader admits exactly original PART2000486 / SECTION2000486.
        // N3 and release declarations remain in the retained model, never edges.
        output.Append(Kind::Type13,Role::Constitutive,connection.source_id,2000486,row,nodes,2);
    }
    const auto& welds = physical.welds().model();
    const auto& weld_source = physical.welds().source().data().spotwelds;
    Require(weld_source.size() == welds.connection_count(), "Connectivity TYPE25 source/model count differs");
    for (std::size_t row = 0; row < welds.connection_count(); ++row) {
        const auto& weld = welds.connections()[row];
        Require(weld.source_element_id == weld_source[row].id, "Connectivity TYPE25 source order differs");
        output.Append(Kind::Type25,Role::Constitutive,weld.source_element_id,0,row,weld.global_node,2);
    }
    const auto mass = physical.point_masses().contributions().records();
    for (std::size_t row = 0; row < mass.size(); ++row) {
        const auto& record = mass[row].source;
        output.Append(Kind::PointMass,Role::CoefficientOnly,record.source_element_id,0,row,&record.domain_node,1);
    }
    if (const auto* structural = physical.structural_beams()) {
        const auto parents = structural->parents();
        for (std::size_t row = 0; row < parents.size(); ++row)
            AppendElementInput(output,Kind::Beam18,row,parents[row].reference.input(),domain,
                               parents[row].domain_nodes);
    }
}
} // namespace crash::cases::vehicle_startup::connectivity::detail
