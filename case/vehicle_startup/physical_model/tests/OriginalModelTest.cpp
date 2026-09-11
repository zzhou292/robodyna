#include "Support.h"

namespace crash::cases::vehicle_startup::physical_model::test {
TEST(VehiclePhysicalModelOriginal,EveryOriginalConnectionAndSolidRetainsItsSourceAndMaterialOnOneDomain) {
    const auto& model = Actual();
    const auto& domain = model.source_domain().domain();
    const auto& source = Source().source();
    ASSERT_EQ(model.beams().connection_count(), 4442);
    EXPECT_EQ(model.beams().node_count(), 7494);
    EXPECT_EQ(model.beams().global_node_count(), 372435);
    const auto& beams = source.type13_source().data();
    for (std::size_t b = 0; b < beams.beams.size(); ++b) {
        EXPECT_EQ(model.beams().connections()[b].source_id, beams.beams[b].id);
        for (unsigned k = 0; k < 2; ++k) {
            fe::type13::EndpointContribution endpoint;
            ASSERT_TRUE(model.beams().Endpoint(b, k, endpoint));
            const auto& node = beams.nodes.at(beams.beams[b].node_indices[k]);
            EXPECT_EQ(endpoint.global_node, domain.Find(node.id));
            EXPECT_EQ(endpoint.source_node_id, node.id);
            Same(endpoint.coefficients.mass_kg, beams.beams[b].startup.endpoint.mass_kg);
            Same(endpoint.coefficients.isotropic_inertia_kg_m2, beams.beams[b].startup.endpoint.isotropic_inertia_kg_m2);
        }
    }
    std::size_t orientation_only = 0;
    for (std::size_t n = 0; n < model.beams().node_count(); ++n)
        orientation_only += model.beams().nodes()[n].global_node == SIZE_MAX;
    EXPECT_EQ(orientation_only, 1);
    ASSERT_EQ(model.solids().solid18().size(), 908);
    ASSERT_EQ(model.solids().solid24().size(), 1309);
    ASSERT_EQ(model.solids().solid6z().size(), 195);
    EXPECT_TRUE(model.solids().domain()->SharesStorage(domain));
    EXPECT_TRUE(model.solids().contributions()->Matches(*model.coefficients().solids()));
    const auto& solids = source.solid_source().data();
    for (const auto& row : solids.rows) {
        const auto check = [&](const auto& parent) {
            EXPECT_EQ(parent.reference.input().source_element_id, row.element_id);
            EXPECT_EQ(parent.reference.input().source_part_id, row.part_id);
            for (unsigned k = 0; k < (row.family == modelio::solid_source::Family::Solid6z ? 6u : 8u); ++k) {
                const auto slot = row.family == modelio::solid_source::Family::Solid6z ? row.six_to_raw[k] : k;
                EXPECT_EQ(parent.domain_nodes[k], domain.Find(row.raw_node_ids[slot]));
            }
        };
        if (row.family == modelio::solid_source::Family::Solid18) check(model.solids().solid18()[row.reference_index]);
        else if (row.family == modelio::solid_source::Family::Solid24) check(model.solids().solid24()[row.reference_index]);
        else check(model.solids().solid6z()[row.reference_index]);
    }
    EXPECT_EQ(model.welds().model().connection_count(), 2828);
    EXPECT_TRUE(model.welds().domain().SharesStorage(domain));
    EXPECT_TRUE(model.point_masses().contributions().domain()->SharesStorage(domain));
    RecordProperty("inclusive_startup_forecast", RecordValue(model.forecast().total_bytes));
    RecordProperty("solid_model_bytes", RecordValue(model.solids().owned_payload_bytes()));
    RecordProperty("beam_model_bytes", RecordValue(model.beams().owned_payload_bytes()));
}
} // namespace crash::cases::vehicle_startup::physical_model::test
