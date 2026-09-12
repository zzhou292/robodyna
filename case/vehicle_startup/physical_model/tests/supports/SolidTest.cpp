#include "Support.h"
namespace crash::cases::vehicle_startup::physical_model::supports_test {
TEST(VehicleSupportsPhysicalOriginal, EveryOriginalSolidSlotAndAnalyticAirbagMaterialReachesTheTypedModel) {
    const auto& model=Model().solids(); const auto& source=Inputs().solids.data();
    const auto& domain=Domain().domain();
    const std::size_t sizes[]{model.solid18().size(),model.solid24().size(),model.solid6z().size(),
        model.solid18_law44().size(),model.solid18_law90().size()};
    std::size_t offsets[5]{},seen[5]{};
    for (unsigned f=1;f<5;++f) offsets[f]=offsets[f-1]+sizes[f-1];
    unsigned airbag=0,repeated_airbag=0;
    const auto check=[&](const auto& parent,const auto& reference,const solid_source::Row& row,
                         const double* mass,unsigned slots) {
        const auto family=static_cast<unsigned>(row.family);
        ASSERT_EQ(row.reference_index,seen[family]++);
        const auto& input=parent.reference.input();
        EXPECT_EQ(input.source_element_id,row.element_id); EXPECT_EQ(input.source_part_id,row.part_id);
        const auto& coefficient=model.contributions()->parents()[offsets[family]+row.reference_index];
        EXPECT_EQ(coefficient.source_element_id,row.element_id); EXPECT_EQ(coefficient.node_count,slots);
        for (unsigned k=0;k<slots;++k) {
            const auto raw_slot=slots==6 ? row.six_to_raw[k] : k;
            EXPECT_EQ(parent.domain_nodes[k],domain.Find(row.raw_node_ids[raw_slot]));
            EXPECT_EQ(coefficient.domain_node[k],parent.domain_nodes[k]); Same(coefficient.mass_kg[k],mass[k]);
            Same(input.position_m[k].x,reference.input().position_m[k].x);
            Same(input.position_m[k].y,reference.input().position_m[k].y);
            Same(input.position_m[k].z,reference.input().position_m[k].z);
        }
    };
    for (const auto& row:source.rows) {
        using F=solid_source::Family; const auto i=row.reference_index;
        if (row.family==F::Solid18) check(model.solid18()[i],source.solid18[i],row,source.solid18[i].mass().source_nodal_mass_kg,8);
        else if (row.family==F::Solid24) check(model.solid24()[i],source.solid24[i],row,source.solid24[i].mass().source_slot_mass_kg,8);
        else if (row.family==F::Solid6z) check(model.solid6z()[i],source.solid6z[i],row,source.solid6z[i].mass().source_slot_mass_kg,6);
        else if (row.family==F::Solid18Law90) check(model.solid18_law90()[i],source.solid18_law90[i],row,source.solid18_law90[i].mass().source_nodal_mass_kg,8);
        else {
            ASSERT_EQ(row.family,F::Solid18Law44);
            const auto& parent=model.solid18_law44()[i];
            check(parent,source.solid18_law44[i],row,source.solid18_law44[i].mass().source_nodal_mass_kg,8);
            if (row.part_id!=2000945) continue;
            ++airbag; repeated_airbag+=parent.domain_nodes[4]==parent.domain_nodes[5];
            const auto& value=model.materials44()[parent.material_index].value;
            const auto& original=source.parts[row.part_index].law44;
            EXPECT_EQ(value.material.hardening,tl::material::law44::solid::HardeningKind::Analytic);
            EXPECT_EQ(value.curve.count,0u); EXPECT_EQ(value.curve.plastic_strain,nullptr);
            const auto values=[](const auto& p) {const auto& m=p.material; return std::array<double,12>{
                m.young_pa,m.poisson_ratio,m.density_kg_m3,m.rate_c_per_s,m.rate_p,m.cutoff_hz,
                m.analytic.a_pa,m.analytic.b_pa,m.analytic.exponent,m.analytic.maximum_stress_pa,
                m.analytic.maximum_plastic_strain,p.stress_limit_pa};};
            const auto a=values(value),b=values(original);
            for (unsigned k=0;k<a.size();++k) Same(a[k],b[k]);
        }
    }
    for (unsigned f=0;f<5;++f) EXPECT_EQ(seen[f],sizes[f]);
    EXPECT_EQ(airbag,80u); EXPECT_EQ(repeated_airbag,4u);
}
} // namespace crash::cases::vehicle_startup::physical_model::supports_test
