#include "Support.h"
#include <set>

namespace crash::cases::vehicle_startup::physical_model::test {
TEST(VehiclePhysicalModelOriginal,CompleteOnceOnlyCoefficientsFeedEveryOriginalPartAndSelectedPlainMember) {
    const auto& model = Actual();
    const auto& ledger = model.coefficients();
    const auto& domain = model.source_domain().domain();
    const auto& scope = ledger.scope();
    EXPECT_EQ(scope.covered_nodes, domain.node_count()); EXPECT_EQ(scope.uncovered_nodes, 0);
    EXPECT_EQ(scope.qeph_parents, 324094); EXPECT_EQ(scope.t3_parents, 21301); EXPECT_EQ(scope.qbat_parents, 4250);
    EXPECT_EQ(scope.type13_connections, 4442); EXPECT_EQ(scope.type25_connections, 2828);
    EXPECT_EQ(scope.element_mass_records, 148);
    EXPECT_EQ(scope.solid18_parents, 908); EXPECT_EQ(scope.solid24_parents, 1309); EXPECT_EQ(scope.solid6z_parents, 195);
    const auto& rigid = model.rigid_assembly();
    ASSERT_EQ(rigid.groups().size(), 773); ASSERT_EQ(rigid.members().size(), 12824);
    EXPECT_EQ(rigid.parts()->topology()->part_count(), 22);
    ASSERT_EQ(model.plain_groups().group_count(), 753);
    EXPECT_TRUE(model.plain_groups().physical_coefficients());
    std::size_t zero_j = 0;
    for (const auto& member : rigid.members()) {
        const auto& c = ledger.nodes()[member.domain_node].coefficients;
        EXPECT_EQ(member.source_node_id, domain.nodes()[member.domain_node].source_id);
        Same(member.mass_kg, c.mass); Same(member.isotropic_inertia_kg_m2, c.isotropic_inertia);
        zero_j += c.isotropic_inertia == 0;
    }
    EXPECT_GT(zero_j, 0);
    std::size_t p = 0;
    for (const auto& selection : model.source_domain().plain_groups()) {
        if (selection.disposition == modelio::physical_domain::GroupDisposition::Omitted) continue;
        const auto& original = model.source_domain().source().data().plain_groups.at(selection.source_group);
        const auto& group = model.plain_groups().groups()[p++];
        EXPECT_EQ(group.source_group_id, original.id); EXPECT_EQ(group.source_node_set_id, selection.case_node_set_id);
        ASSERT_EQ(group.member_count, selection.members.size());
        EXPECT_GT(group.structural_mass_kg, 0);
        long double independent_mass = 0;
        for (std::size_t k = 0; k < group.member_count; ++k) {
            const auto& member = model.plain_groups().members()[group.member_offset + k];
            EXPECT_EQ(member.source_node_id, selection.members[k]); independent_mass += member.mass_kg;
        }
        EXPECT_LE(std::abs(static_cast<long double>(group.structural_mass_kg) - independent_mass),
                  4 * group.member_count * std::numeric_limits<double>::epsilon() * independent_mass);
    }
    long double mass = 0;
    for (const auto& node : ledger.nodes()) {
        ASSERT_TRUE(fe::HasCoefficientProducer(node.occurrences));
        EXPECT_GE(node.coefficients.mass, 0); EXPECT_GE(node.coefficients.isotropic_inertia, 0);
        const auto& c = node.coefficients;
        mass += static_cast<long double>(c.shell.mass) + c.type25.mass + c.type13.mass + c.element_mass +
                c.solid18_mass + c.solid24_mass + c.solid6z_mass;
    }
    EXPECT_LE(std::abs(static_cast<long double>(ledger.totals().mass) - mass),
              8 * domain.node_count() * std::numeric_limits<double>::epsilon() * mass);
    std::set<std::uint64_t> beam_ids;
    for (const auto& beam : Source().source().type13_source().data().beams) beam_ids.insert(beam.id);
    std::size_t independent_mass_ids = 0;
    for (const auto& point : model.point_masses().contributions().records()) {
        independent_mass_ids += beam_ids.count(point.source.source_element_id);
        EXPECT_EQ(ledger.domain()->nodes()[point.source.domain_node].source_id, point.source.source_node_id);
    }
    EXPECT_EQ(independent_mass_ids, 35); // Genuine original ADMAS/beam namespace overlap, both retained.
    RecordProperty("mass_kg", RecordValue(ledger.totals().mass));
    RecordProperty("point_mass_kg", RecordValue(ledger.totals().element_mass));
    RecordProperty("rigid_members_zero_j", RecordValue(zero_j));
    RecordProperty("coefficient_bytes", RecordValue(ledger.owned_payload_bytes()));
    RecordProperty("rigid_binding_bytes", RecordValue(rigid.owned_payload_bytes()));
}
} // namespace crash::cases::vehicle_startup::physical_model::test
