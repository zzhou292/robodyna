#include "Support.h"

namespace crash::cases::vehicle_startup::physical_model::extended_test {
TEST(VehicleExtendedPhysicalOriginal, All4900NativeReferencesMaterialsAndRepeatedSlotsShareTheCompleteDomain) {
    const auto& physical = Model();
    const auto& model = physical.solids();
    const auto& source = Inputs().solids.data();
    const auto& domain = Domain().domain();
    ASSERT_EQ(source.rows.size(), 4900u);
    EXPECT_EQ(model.profile(), fe::solids::ModelProfile::ExtendedLaw44Law90);
    ASSERT_TRUE(model.domain()->SharesStorage(domain));
    EXPECT_EQ(model.solid18().size(), 908u); EXPECT_EQ(model.solid24().size(), 1991u);
    EXPECT_EQ(model.solid6z().size(), 350u); EXPECT_EQ(model.solid18_law44().size(), 306u);
    EXPECT_EQ(model.solid18_law90().size(), 1345u);
    unsigned repeated = 0;
    const auto check = [&](const auto& parent, const auto& reference, const solid_source::Row& row,
                           const double* mass, unsigned slots, std::size_t coefficient_offset) {
        const auto& input = parent.reference.input();
        EXPECT_EQ(input.source_element_id, row.element_id); EXPECT_EQ(input.source_part_id, row.part_id);
        const auto& coefficient = model.contributions()->parents()[coefficient_offset + row.reference_index];
        EXPECT_EQ(coefficient.source_element_id, row.element_id);
        EXPECT_EQ(coefficient.node_count, slots);
        for (unsigned k = 0; k < slots; ++k) {
            const auto raw_slot = slots == 6 ? row.six_to_raw[k] : k;
            EXPECT_EQ(parent.domain_nodes[k], domain.Find(row.raw_node_ids[raw_slot]));
            EXPECT_EQ(coefficient.domain_node[k], parent.domain_nodes[k]);
            Same(coefficient.mass_kg[k], mass[k]);
            Same(input.position_m[k].x, reference.input().position_m[k].x);
            Same(input.position_m[k].y, reference.input().position_m[k].y);
            Same(input.position_m[k].z, reference.input().position_m[k].z);
        }
    };
    for (const auto& row : source.rows) {
        using F = solid_source::Family;
        const auto r = row.reference_index;
        if (row.family == F::Solid18)
            check(model.solid18()[r], source.solid18[r], row, source.solid18[r].mass().source_nodal_mass_kg, 8, 0);
        else if (row.family == F::Solid24)
            check(model.solid24()[r], source.solid24[r], row, source.solid24[r].mass().source_slot_mass_kg, 8, 908);
        else if (row.family == F::Solid6z)
            check(model.solid6z()[r], source.solid6z[r], row, source.solid6z[r].mass().source_slot_mass_kg, 6, 2899);
        else if (row.family == F::Solid18Law44) {
            check(model.solid18_law44()[r], source.solid18_law44[r], row,
                  source.solid18_law44[r].mass().source_nodal_mass_kg, 8, 3249);
            const auto& parent = model.solid18_law44()[r];
            const auto& material = model.materials44()[parent.material_index].value;
            Same(material.material.young_pa, row.part_id == 2000016 ? 50e9 : 200e9);
            repeated += parent.domain_nodes[4] == parent.domain_nodes[5];
        } else {
            ASSERT_EQ(row.family, F::Solid18Law90);
            check(model.solid18_law90()[r], source.solid18_law90[r], row,
                  source.solid18_law90[r].mass().source_nodal_mass_kg, 8, 3555);
        }
    }
    EXPECT_EQ(repeated, 109u);
    ASSERT_EQ(model.materials90().size(), 1u);
    const auto& material = model.materials90()[0].value;
    EXPECT_EQ(material.reader().loading_flag, 1); Same(material.reader().curve_scale, 1e6);
    EXPECT_NE(material.curve().stress_pa, source.foam_curve_ordinate.data());
    for (std::size_t i = 0; i < material.curve().count; ++i) {
        Same(material.curve().compression_strain[i], source.foam_compression_strain[i]);
        Same(material.curve().stress_pa[i], source.foam_curve_ordinate[i]);
    }
    EXPECT_EQ(physical.beams().global_node_count(), domain.node_count());
    EXPECT_EQ(physical.beams().connection_count(), 4442u);
    EXPECT_TRUE(physical.welds().domain().SharesStorage(domain));
    EXPECT_EQ(physical.welds().model().connection_count(), 2828u);
    EXPECT_GT(model.startup_payload_bytes(), Limits{}.solid_bytes);
    EXPECT_LE(model.startup_payload_bytes(), Limits::ExtendedSolids().solid_bytes);
    RecordProperty("solid_model_owned_bytes", model.owned_payload_bytes());
    RecordProperty("solid_model_startup_bytes", model.startup_payload_bytes());
    RecordProperty("physical_forecast", std::to_string(physical.forecast().total_bytes));
    RecordProperty("packing_bytes", physical.forecast().packing_bytes);
}
} // namespace crash::cases::vehicle_startup::physical_model::extended_test
