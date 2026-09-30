#include "Support.h"
#include <set>
namespace crash::cases::vehicle_startup::physical_model::supports_test {
TEST(VehicleSupportsPhysicalOriginal, All4980SolidsAnd142BeamsRetainSourceOrderAndOwnedCurves) {
    const auto& physical=Model();
    const auto& solids=physical.solids();
    EXPECT_EQ(solids.solid18().size(),908u); EXPECT_EQ(solids.solid24().size(),1991u);
    EXPECT_EQ(solids.solid6z().size(),350u); EXPECT_EQ(solids.solid18_law44().size(),386u);
    EXPECT_EQ(solids.solid18_law90().size(),1345u);
    const auto* beams=physical.structural_beams(); ASSERT_NE(beams,nullptr);
    ASSERT_EQ(beams->parents().size(),142u);
    ASSERT_TRUE(beams->domain()->SharesStorage(Domain().domain()));
    const auto& source=Beams().data();
    std::set<std::size_t> endpoints;
    for (std::size_t i=0;i<source.rows.size();++i) {
        const auto& row=source.rows[i]; const auto& parent=beams->parents()[i];
        const auto& material=beams->materials()[parent.material_index].value;
        const auto& original=source.parts[row.part_index].material;
        EXPECT_EQ(parent.reference.input().source_element_id,row.element_id);
        EXPECT_EQ(parent.reference.input().source_part_id,row.part_id);
        EXPECT_EQ(parent.reference.input().source_node_id[2],source.nodes[row.nodes[2]].id);
        EXPECT_NE(material.curve.plastic_strain,original.curve.plastic_strain);
        ASSERT_EQ(material.curve.count,original.curve.count);
        for (std::size_t k=0;k<material.curve.count;++k) {
            Same(material.curve.plastic_strain[k],original.curve.plastic_strain[k]);
            Same(material.curve.yield_stress_pa[k],original.curve.yield_stress_pa[k]);
        }
        for (unsigned slot=0;slot<2;++slot) {
            fe::beam18::EndpointContribution value; ASSERT_TRUE(beams->Endpoint(i,slot,value));
            EXPECT_EQ(value.source_node_id,source.nodes[row.nodes[slot]].id);
            EXPECT_EQ(value.global_node,Domain().domain().Find(value.source_node_id));
            endpoints.insert(value.global_node);
            Same(value.coefficients.mass_kg,row.reference.endpoint().mass_kg);
            Same(value.coefficients.native_total_inertia_kg_m2,row.reference.endpoint().native_total_inertia_kg_m2);
        }
    }
    EXPECT_EQ(endpoints.size(),146u);
    EXPECT_EQ(physical.beams().connection_count(),4442u);
    const auto copy=physical; EXPECT_TRUE(copy.structural_beams()->SharesStorage(*beams));
    RecordProperty("beam_model_owned_bytes",beams->owned_payload_bytes());
    RecordProperty("beam_model_startup_bytes",beams->startup_payload_bytes());
    RecordProperty("physical_forecast",std::to_string(physical.forecast().total_bytes));
}
TEST(VehicleSupportsPhysicalOriginal, ExactBeamSnapshotAndRigidCoefficientsUseOneCompleteLedger) {
    const auto& physical=Model(); const auto& ledger=physical.coefficients();
    EXPECT_EQ(ledger.order(),fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_Beam18_V5);
    ASSERT_NE(ledger.beam18(),nullptr); ASSERT_EQ(ledger.beam18()->records().size(),284u);
    EXPECT_TRUE(ledger.beam18()->model()->Matches(*physical.structural_beams()));
    EXPECT_EQ(ledger.scope().uncovered_nodes,0u); EXPECT_EQ(ledger.scope().beam18_parents,142u);
    std::vector<unsigned> count(ledger.nodes().size());
    std::vector<fe::CoefficientPair> values(ledger.nodes().size());
    double mass=0,inertia=0;
    for (const auto& record:ledger.beam18()->records()) {
        const auto& value=record.value; ++count[value.global_node];
        values[value.global_node].mass+=value.coefficients.mass_kg;
        values[value.global_node].isotropic_inertia+=value.coefficients.native_total_inertia_kg_m2;
    }
    for (std::size_t n=0;n<values.size();++n) {
        Same(values[n].mass,ledger.nodes()[n].coefficients.beam18.mass);
        Same(values[n].isotropic_inertia,ledger.nodes()[n].coefficients.beam18.isotropic_inertia);
        mass+=values[n].mass; inertia+=values[n].isotropic_inertia;
    }
    Same(mass,ledger.totals().beam18.mass); Same(inertia,ledger.totals().beam18.isotropic_inertia);
    for (std::size_t i=0;i<count.size();++i) EXPECT_EQ(ledger.nodes()[i].occurrences.beam18,count[i]);
    const auto& rigid=physical.rigid_assembly();
    for (const auto& member:rigid.members()) {
        const auto& value=ledger.nodes()[member.domain_node].coefficients;
        Same(member.mass_kg,value.mass); Same(member.isotropic_inertia_kg_m2,value.isotropic_inertia);
    }
    for (std::size_t n=0;n<physical.plain_groups().member_count();++n) {
        const auto& member=physical.plain_groups().members()[n];
        Same(member.unpartitioned_native_inertia_kg_m2,ledger.nodes()[member.global_node].coefficients.beam18.isotropic_inertia);
    }
    EXPECT_EQ(physical.point_masses().contributions().records().size(),154u);
    RecordProperty("beam_mass_kg",test::RecordValue(mass));
    RecordProperty("total_mass_kg",test::RecordValue(ledger.totals().mass));
    RecordProperty("rigid_groups",rigid.groups().size()); RecordProperty("rigid_members",rigid.members().size());
}
} // namespace crash::cases::vehicle_startup::physical_model::supports_test
