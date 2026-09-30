#include "PackingFixture.h"
#include "lib_utest/qualification/shell_execution/Fixture.h"

namespace crash::cases::vehicle_startup::shell_execution::test {
TEST(VehicleShellExecutionSmall, CompleteTypedPackingJoinsTheSameLedgerAndMergedPartRoles) {
    shell_execution_test::Fixture fixture;
    detail::Packing packed;
    packed.Reserve(3,7,3);
    for (unsigned i = 0; i < 3; ++i) {
        Part part(1000+i);
        part.native = fixture.source.base.materials[i];
        if (i == 0) part.Rigid();
        part.id.material_id = part.material.id = part.native.material_id;
        const auto& section = fixture.source.base.sections[i];
        part.id.section_id = part.section.id = section.section_id;
        part.material.young_pa = part.native.young_pa;
        part.material.poisson_ratio = part.native.poisson_ratio;
        part.material.density_kg_m3 = part.native.density_kg_m3;
        part.section.thickness_m.fill(section.thickness_m);
        if (i != 0) {
            part.section.through_thickness_points = section.through_thickness_points;
            part.formulation = section.formulation;
        }
        part.Add(packed);
    }
    const auto& curve = fixture.source.base.curve;
    std::vector<modelio::assembly::Curve> supplied(1);
    supplied[0].id = curve.curve_id;
    supplied[0].plastic_strain.assign(curve.curve.plastic_strain,curve.curve.plastic_strain+curve.curve.count);
    supplied[0].stress_pa.assign(curve.curve.yield_stress_pa,curve.curve.yield_stress_pa+curve.curve.count);
    detail::PackCurves(packed,supplied,{});
    packed.parents.assign(fixture.source.parents.begin(),fixture.source.parents.end());
    const auto rows = fixture.source.Failures();
    fe::ShellBatchPlasticityBinding catalog;
    ASSERT_EQ(catalog.InitializeExecutionCatalog(fixture.shells,packed.input()).status,
              fe::ShellPlasticityBindingStatus::Success);
    EXPECT_TRUE(catalog.SameScope(fixture.catalog));
    fe::ShellBatchFailureBinding failure;
    ASSERT_EQ(failure.InitializeExecution(catalog,rows.data(),rows.size()).status,
              fe::ShellPlasticityBindingStatus::Success);
    fe::ShellExecutionBinding execution;
    ASSERT_EQ(execution.Initialize(catalog,fixture.ledger,fixture.rigid).status,
              fe::ShellPlasticityBindingStatus::Success);
    fe::ShellPhysicalBinding physical;
    ASSERT_TRUE(physical.InitializeExecution({&fixture.shells,&catalog,&failure,nullptr},fixture.ledger,execution));
    EXPECT_EQ(execution.counts().rigid_skin,3);
    EXPECT_TRUE(physical.coefficients()->Matches(fixture.ledger));
    EXPECT_EQ(physical.execution()->rigid()->members().data(),fixture.rigid.members().data());
    for (const auto& parent : packed.parents) {
        const auto* role = execution.parent(parent.family,parent.family_index);
        ASSERT_NE(role,nullptr);
        EXPECT_TRUE(detail::Same(role->source,parent));
        if (parent.material_id == 1000) {
            EXPECT_EQ(role->law,fe::ShellSectionLaw::RigidSkin);
            EXPECT_EQ(role->material_points,0u);
            EXPECT_EQ(role->root_index,0);
        } else if (parent.material_id == 2000524) {
            EXPECT_EQ(role->material_points,parent.family == fe::ShellBindingFamily::T3 ? 1u : 4u);
        }
    }
    // Same Q4/T3 node sets keep separate original layer parents and placement.
    EXPECT_EQ(fixture.shells.qeph_nodes(0),fixture.shells.qeph_nodes(1));
    EXPECT_EQ(fixture.shells.qeph_nodes(0),fixture.shells.qbat_nodes(0));
    EXPECT_NE(fixture.shells.qeph_reference(0).input.placement,fixture.shells.qeph_reference(1).input.placement);
}
} // namespace crash::cases::vehicle_startup::shell_execution::test
