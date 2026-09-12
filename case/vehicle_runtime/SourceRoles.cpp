#include "SourceRoles.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_runtime {
SourceRoles ResolveSourceRoles(const vehicle_startup::physical_attachments::VehiclePhysicalAttachments& source,
                              std::size_t max_nodes) {
    using output::Require;
    const auto& physical = source.physical();
    const auto& domain = physical.source_domain().domain();
    const auto& rigid = physical.rigid_assembly();
    const auto& cin = source.attachments().model();
    Require(max_nodes && max_nodes <= 524288 && domain.node_count() <= max_nodes &&
        physical.coefficients().domain()->SharesStorage(domain) && cin.domain()->SharesStorage(domain),
        "Runtime source-role domain exceeds scope or differs from physical coefficients/CIN");
    SourceRoles out;
    out.node.resize(domain.node_count());
    Require(out.node.capacity() <= max_nodes, "Runtime source-role capacity exceeds forecast");
    const auto mark = [&](std::size_t node, SourceRole role) {
        Require(node < out.node.size(), "Runtime source role names a node outside the physical domain");
        out.node[node] |= role;
    };
    const auto map = physical.coefficients().shells()->mapping();
    for (auto node : map) mark(node,Shell);
    for (const auto& group : rigid.groups()) {
        const auto role = group.source_kind == tl::fea::RigidBindingSourceKind::Part ? Part : PlainRigid;
        Require(group.member_offset <= rigid.members().size() &&
            group.member_count <= rigid.members().size() - group.member_offset, "Rigid source member range is invalid");
        for (std::size_t i = 0; i < group.member_count; ++i)
            mark(rigid.members()[group.member_offset+i].domain_node,role);
    }
    const auto rows = cin.rows();
    for (std::size_t i = 0; i < rows.count; ++i) {
        mark(rows.data[i].secondary_domain_node,CinSecondary);
        for (auto node : rows.data[i].master_domain_nodes) mark(node,CinMaster);
    }
    const auto& beams = physical.beams();
    for (std::size_t c = 0; c < beams.connection_count(); ++c) {
        for (unsigned slot = 0; slot < 2; ++slot) {
            tl::fea::type13::EndpointContribution endpoint;
            Require(beams.Endpoint(c,slot,endpoint), "Original TYPE13 endpoint contribution is unavailable");
            mark(endpoint.global_node,Type13Endpoint);
            Require(domain.nodes()[endpoint.global_node].source_id == endpoint.source_node_id,
                    "TYPE13 endpoint identity differs from physical source domain");
        }
    }
    if (const auto* structural = physical.structural_beams()) {
        const auto* coefficients = physical.coefficients().beam18();
        Require(coefficients && coefficients->model()->Matches(*structural) &&
            structural->domain()->SharesStorage(domain), "Structural beam role authority differs from the ledger");
        for (std::size_t parent = 0; parent < structural->parents().size(); ++parent)
            for (unsigned slot = 0; slot < 2; ++slot) {
                tl::fea::beam18::EndpointContribution endpoint;
                Require(structural->Endpoint(parent,slot,endpoint), "Structural beam endpoint is unavailable");
                mark(endpoint.global_node,Beam18Endpoint);
                Require(domain.nodes()[endpoint.global_node].source_id == endpoint.source_node_id,
                        "Structural beam endpoint identity differs from physical domain");
            }
    } else Require(!physical.coefficients().beam18(), "Unexpected structural beam coefficient authority");
    const auto* welds = physical.coefficients().type25();
    Require(welds, "Complete runtime source-role census requires retained TYPE25 model");
    for (std::size_t c = 0; c < welds->connection_count(); ++c) {
        const auto& connection = welds->connections()[c];
        for (unsigned slot = 0; slot < 2; ++slot) mark(connection.global_node[slot],Type25Endpoint);
    }
    return out;
}
} // namespace crash::cases::vehicle_runtime
