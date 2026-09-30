#include "Support.h"

namespace crash::cases::vehicle_startup::physical_model::extended_test {
TEST(VehicleExtendedPhysicalOriginal, NativeMassSlotsFeedOneV4LedgerAndRealRigidMembersWithoutInertiaSubstitutes) {
    const auto& model = Model();
    const auto& domain = Domain().domain();
    const auto& ledger = model.coefficients();
    EXPECT_EQ(ledger.order(), fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_V4);
    ASSERT_TRUE(ledger.domain()->SharesStorage(domain));
    ASSERT_EQ(ledger.scope().uncovered_nodes, 0u);
    EXPECT_EQ(ledger.scope().covered_nodes, domain.node_count());
    EXPECT_EQ(ledger.scope().solid18_law44_parents, 306u);
    EXPECT_EQ(ledger.scope().solid18_law90_parents, 1345u);
    EXPECT_EQ(ledger.scope().element_mass_records, 150u);
    // Independent source-slot scatter, including every repeated rear H8 slot.
    std::vector<std::array<double, 5>> masses(domain.node_count());
    std::vector<std::array<std::uint64_t, 5>> occurrences(domain.node_count());
    for (const auto& parent : model.solids().contributions()->parents()) {
        const auto family = static_cast<unsigned>(parent.family);
        ASSERT_LT(family, 5u);
        for (unsigned slot = 0; slot < parent.node_count; ++slot) {
            const auto n = parent.domain_node[slot];
            masses[n][family] += parent.mass_kg[slot]; ++occurrences[n][family];
        }
    }
    for (std::size_t n = 0; n < domain.node_count(); ++n) {
        const auto& value = ledger.nodes()[n];
        const double actual[]{value.coefficients.solid18_mass, value.coefficients.solid24_mass,
            value.coefficients.solid6z_mass, value.coefficients.solid18_law44_mass, value.coefficients.solid18_law90_mass};
        const std::uint64_t counts[]{value.occurrences.solid18, value.occurrences.solid24,
            value.occurrences.solid6z, value.occurrences.solid18_law44, value.occurrences.solid18_law90};
        for (unsigned f = 0; f < 5; ++f) { Same(actual[f], masses[n][f]); EXPECT_EQ(counts[f], occurrences[n][f]); }
    }
    const auto& rigid = model.rigid_assembly();
    EXPECT_EQ(rigid.parts()->topology()->part_count(), 22u);
    EXPECT_EQ(rigid.parts()->topology()->root_count(), 20u);
    EXPECT_EQ(model.plain_groups().group_count(), Domain().counts().complete_groups + Domain().counts().restricted_groups);
    EXPECT_EQ(rigid.members().size(), 5452u + Domain().counts().plain_members);
    for (const auto& member : rigid.members()) {
        const auto& value = ledger.nodes()[member.domain_node].coefficients;
        Same(member.mass_kg, value.mass); Same(member.isotropic_inertia_kg_m2, value.isotropic_inertia);
        EXPECT_EQ(member.source_node_id, domain.nodes()[member.domain_node].source_id);
    }
    unsigned added = 0;
    for (const auto& mass : model.point_masses().contributions().records()) {
        const auto id = mass.source.source_element_id;
        if (id != 2409447 && id != 2409448) continue;
        ++added;
        EXPECT_EQ(mass.source.source_node_id, id == 2409447 ? 2406557u : 2406558u);
        Same(mass.mass_kg, .010001);
        const auto& value = ledger.nodes()[mass.source.domain_node];
        Same(value.coefficients.element_mass, mass.mass_kg);
        Same(value.coefficients.isotropic_inertia, 0.);
        EXPECT_NE(rigid.FindMember(mass.source.domain_node), nullptr);
    }
    EXPECT_EQ(added, 2u);
    RecordProperty("ledger_owned_bytes", ledger.owned_payload_bytes());
    RecordProperty("physical_mass_kg", RecordValue(ledger.totals().mass));
    RecordProperty("law44_mass_kg", RecordValue(ledger.totals().solid18_law44_mass));
    RecordProperty("law90_mass_kg", RecordValue(ledger.totals().solid18_law90_mass));
    RecordProperty("point_mass_kg", RecordValue(ledger.totals().element_mass));
}
TEST(VehicleExtendedPhysicalOriginal, FortyRealJointOperatorsRetainBothAntirollSpheresAndFourRodBoundaries) {
    const auto& value = Joints();
    const auto& source = JointSource();
    ASSERT_EQ(source.policy(), JointPolicy);
    ASSERT_EQ(source.data().rows.size(), 44u);
    EXPECT_EQ(source.data().required, 40u); EXPECT_EQ(source.data().boundaries, 4u);
    ASSERT_EQ(value.model().joints().size(), 40u);
    ASSERT_EQ(value.source_rows().size(), 40u);
    EXPECT_TRUE(value.model().domain()->SharesStorage(Domain().domain()));
    unsigned added = 0;
    for (std::size_t i = 0; i < value.source_rows().size(); ++i) {
        const auto& row = source.data().rows[value.source_rows()[i]];
        const auto& joint = value.model().joints()[i];
        EXPECT_EQ(joint.geometry.source_joint_id, row.source_id);
        EXPECT_EQ(row.disposition, joint_source::Disposition::Required);
        for (unsigned k = 0; k < 2; ++k) {
            EXPECT_EQ(joint.domain_nodes[k], Domain().domain().Find(row.nodes[k].source_id));
            EXPECT_EQ(Model().rigid_assembly().groups()[joint.body_groups[k]].source_id, row.nodes[k].body.source_id);
        }
        if (row.source_id != 2200514 && row.source_id != 2200515) continue;
        ++added;
        EXPECT_EQ(joint.property.kind, fe::type45::Kind::Spherical);
        const bool first = row.source_id == 2200514;
        EXPECT_EQ(row.nodes[0].source_id, first ? 2406559u : 2406560u);
        EXPECT_EQ(row.nodes[1].source_id, first ? 2406558u : 2406557u);
        EXPECT_EQ(row.nodes[0].body.source_id, first ? 2200668u : 2200669u);
        EXPECT_EQ(row.nodes[1].body.source_id, first ? 2200666u : 2200667u);
        Same(joint.property.automatic_stiffness_scale, .01);
        Same(joint.property.critical_damping_ratio, .05);
    }
    EXPECT_EQ(added, 2u);
    for (const auto& row : source.data().rows) if (row.disposition == joint_source::Disposition::OmittedAssemblyBoundary)
        EXPECT_TRUE(row.source_id >= 2200526 && row.source_id <= 2200529);
    RecordProperty("required_joints", value.model().joints().size());
    RecordProperty("joint_boundaries", source.data().boundaries);
    RecordProperty("joint_forecast", std::to_string(value.forecast().total_bytes));
}
} // namespace crash::cases::vehicle_startup::physical_model::extended_test
