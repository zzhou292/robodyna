#include "../SourceRoles.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include <set>
#include <sstream>

namespace crash::cases::vehicle_runtime::test {
TEST(VehiclePhysicalAttachmentsOriginal, Type13ActualEndpointRigidAndCinRoleCensus) {
    const auto& source = vehicle_startup::physical_attachments::test::Actual();
    const auto roles = ResolveSourceRoles(source);
    const auto& physical = source.physical();
    const auto& beams = physical.beams();
    const auto& domain = physical.source_domain().domain();
    std::set<std::size_t> unique, part, plain, secondary, master;
    std::size_t records = 0, part_records = 0, plain_records = 0, secondary_records = 0, master_records = 0;
    std::ostringstream intersections;
    for (std::size_t c = 0; c < beams.connection_count(); ++c) {
        for (unsigned slot = 0; slot < 2; ++slot) {
            tl::fea::type13::EndpointContribution endpoint;
            ASSERT_TRUE(beams.Endpoint(c,slot,endpoint));
            ASSERT_LT(endpoint.global_node,roles.node.size());
            const auto node = endpoint.global_node;
            const auto flags = roles.node[node];
            ASSERT_NE(flags & Type13Endpoint,0);
            ASSERT_EQ(domain.nodes()[node].source_id,endpoint.source_node_id);
            unique.insert(node);
            ++records;
            if (flags & Part) { part.insert(node); ++part_records; }
            if (flags & PlainRigid) { plain.insert(node); ++plain_records; }
            if (flags & CinSecondary) { secondary.insert(node); ++secondary_records; }
            if (flags & CinMaster) { master.insert(node); ++master_records; }
            const auto* rigid = physical.rigid_assembly().FindMember(node);
            EXPECT_EQ(rigid != nullptr,(flags & (Part|PlainRigid)) != 0);
            if (flags & (Part|PlainRigid|CinSecondary|CinMaster))
                intersections << endpoint.source_element_id << ':' << slot << ':' << endpoint.source_node_id
                              << ':' << unsigned(flags) << ';';
        }
    }
    ASSERT_EQ(records,8884);
    EXPECT_EQ(unique.size(),7493);
    EXPECT_EQ(secondary_records,8884);
    EXPECT_EQ(secondary.size(),7493);
    EXPECT_EQ(part_records,0);
    EXPECT_EQ(plain_records,0);
    EXPECT_EQ(master_records,0);
    std::size_t shell_nodes=0, cin_nodes=0, shell_secondary_nodes=0;
    for (auto bits:roles.node) {
        shell_nodes += bool(bits&Shell);
        cin_nodes += bool(bits&CinSecondary);
        shell_secondary_nodes += (bits&Shell) && (bits&CinSecondary);
    }
    EXPECT_EQ(shell_nodes,359785);
    EXPECT_EQ(cin_nodes,11165);
    EXPECT_EQ(shell_secondary_nodes,0);
    RecordProperty("shell_nodes",std::to_string(shell_nodes));
    RecordProperty("cin_secondary_nodes",std::to_string(cin_nodes));
    RecordProperty("shell_cin_secondary_intersection",std::to_string(shell_secondary_nodes));
    RecordProperty("type13_endpoint_records",std::to_string(records));
    RecordProperty("type13_unique_endpoint_nodes",std::to_string(unique.size()));
    RecordProperty("type13_part_records",std::to_string(part_records));
    RecordProperty("type13_plain_records",std::to_string(plain_records));
    RecordProperty("type13_cin_secondary_records",std::to_string(secondary_records));
    RecordProperty("type13_cin_master_records",std::to_string(master_records));
    RecordProperty("type13_part_nodes",std::to_string(part.size()));
    RecordProperty("type13_plain_nodes",std::to_string(plain.size()));
    RecordProperty("type13_cin_secondary_nodes",std::to_string(secondary.size()));
    RecordProperty("type13_cin_master_nodes",std::to_string(master.size()));
    RecordProperty("type13_intersections_eid_slot_nid_role_bits",intersections.str());
    RecordProperty("reference_n3_included",std::string("false"));
    auto retained = roles;
    EXPECT_THROW(retained = ResolveSourceRoles(source,domain.node_count()-1),std::runtime_error);
    EXPECT_EQ(retained.node,roles.node);
}
} // namespace crash::cases::vehicle_runtime::test
