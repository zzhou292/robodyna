#include "OriginalSupport.h"

namespace crash::cases::vehicle_startup::shell_execution::test {
TEST(VehicleShellExecutionOriginal, EveryParentRoleFailureAndMaterialRetainsOriginalSourceValues) {
    const auto& actual = Actual();
    const auto& resolution = actual.resolution();
    ASSERT_EQ(actual.catalog().parent_count(),349645);
    ASSERT_EQ(actual.failure().parent_count(),349645);
    EXPECT_EQ(actual.execution().counts().rigid_skin,5102);
    EXPECT_EQ(actual.execution().counts().material_points,1037877);
    EXPECT_EQ(&resolution,actual.model().shell_source().references().resolution());
    EXPECT_TRUE(actual.physical().coefficients()->Matches(actual.model().coefficients()));
    EXPECT_EQ(actual.execution().rigid()->members().data(),actual.model().rigid_assembly().members().data());
    std::vector<bool> seen(resolution.parts().size());
    std::size_t skin = 0, one = 0, four = 0;
    for (std::size_t e = 0; e < resolution.parents().size(); ++e) {
        const auto& source_parent = resolution.parents()[e];
        const auto part = source_parent.part_index;
        const auto* parent = actual.catalog().parent(e);
        const auto* row = actual.execution().parent(e);
        const auto* native = resolution.native_parent(e);
        ASSERT_NE(parent,nullptr);
        ASSERT_NE(row,nullptr);
        ASSERT_NE(native,nullptr);
        ASSERT_TRUE(detail::Same(*parent,native->source)) << e;
        ASSERT_TRUE(detail::Same(row->source,*parent)) << e;
        ASSERT_EQ(actual.execution().parent(parent->family,parent->family_index),row);
        SameFailure(*actual.failure().parent(e),*native);
        if (resolution.role(part) == source::SourceShellRole::OriginalRigidPart) {
            ++skin;
            ASSERT_EQ(row->law,fe::ShellSectionLaw::RigidSkin);
            ASSERT_EQ(row->material_points,0u);
            ASSERT_EQ(row->root_index,resolution.rigid_root_index(part));
            ASSERT_EQ(resolution.material(part)->source.keyword,"*MAT_RIGID");
            ASSERT_EQ(resolution.section(part)->through_thickness_points,3);
            fe::sections::PointParameters plastic;
            tl::material::ShellElasticLaw1PointParameters elastic;
            EXPECT_FALSE(actual.catalog().Parameters(parent->family,parent->family_index,&plastic));
            EXPECT_FALSE(actual.catalog().ElasticParameters(parent->family,parent->family_index,&elastic));
            continue;
        }
        EXPECT_EQ(row->part_index,SIZE_MAX);
        EXPECT_EQ(row->root_index,SIZE_MAX);
        one += row->material_points == 1;
        four += row->material_points == 4;
        if (seen[part]) continue;
        seen[part] = true;
        const auto& material = *resolution.native_material(part);
        if (material.law == fe::ShellSectionLaw::LayeredLaw1Nip3) {
            tl::material::ShellElasticLaw1PointParameters elastic;
            ASSERT_TRUE(actual.catalog().ElasticParameters(parent->family,parent->family_index,&elastic));
            Same(elastic.young_pa,material.young_pa);
            Same(elastic.poisson_ratio,material.poisson_ratio);
            Same(elastic.density_kg_m3,material.density_kg_m3);
        } else {
            fe::sections::PointParameters value;
            ASSERT_TRUE(actual.catalog().Parameters(parent->family,parent->family_index,&value));
            auto reproduced = material;
            reproduced.young_pa = value.young_pa;
            reproduced.poisson_ratio = value.poisson_ratio;
            reproduced.density_kg_m3 = value.density_kg_m3;
            reproduced.rate = value.rate;
            reproduced.hardening = value.hardening;
            reproduced.linear = value.linear;
            reproduced.continuation = value.continuation;
            ASSERT_TRUE(detail::Same(reproduced,material));
            if (material.curve_id) {
                const auto* curve = OriginalCurve(resolution,material.curve_id);
                ASSERT_NE(curve,nullptr);
                ASSERT_EQ(value.curve.count,curve->plastic_strain.size());
                EXPECT_NE(value.curve.plastic_strain,curve->plastic_strain.data());
                for (std::size_t i = 0; i < value.curve.count; ++i) {
                    Same(value.curve.plastic_strain[i],curve->plastic_strain[i]);
                    Same(value.curve.yield_stress_pa[i],curve->stress_pa[i]);
                }
            } else EXPECT_EQ(value.curve.count,0);
        }
    }
    EXPECT_EQ(skin,5102);
    EXPECT_EQ(one,1);
    EXPECT_EQ(four,4250);
    RecordProperty("inclusive_startup_bytes",std::to_string(actual.forecast().total_bytes));
    RecordProperty("packing_bytes",std::to_string(actual.forecast().packing_bytes));
    RecordProperty("catalog_host_bytes",std::to_string(actual.catalog().host_bytes()));
    RecordProperty("execution_startup_bytes",std::to_string(actual.execution().forecast().startup_payload_bytes));
    RecordProperty("physical_host_bytes",std::to_string(actual.physical().owned_payload_bytes()));
}
} // namespace crash::cases::vehicle_startup::shell_execution::test
