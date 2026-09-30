#include "../Internal.h"
#include "../../tests/Fixture.h"
#include "lib_utest/qualification/qbat_catalog/Fixture.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
namespace crash::cases::vehicle_wall::native::execution_test {
namespace d=execution_detail;
namespace fe=tl::fea;
namespace {
modelio::assembly::Law1ExecutionPolicy Policy() {
    using namespace modelio::assembly;
    const SourceUnits units{1000,.001,1};
    return Law1ExecutionPolicy::Resolve(Law1ExecutionProfile::NativeA62OrdinaryNpt0,units,
        QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,units));
}
EnvironmentParent Environment() {return {105,106,107,108,2,6,{5,6,7,8},{5,6,7,8}};}
d::original::detail::Packing Packed(const qbat_catalog_test::Fixture& f) {
    d::original::detail::Packing p;
    p.Reserve(4,7,1);p.curves.push_back(f.curve);
    p.materials.assign(f.materials.begin(),f.materials.end());p.sections.assign(f.sections.begin(),f.sections.end());
    p.parents.assign(f.parents.begin(),f.parents.end());p.failure.assign(f.failures.begin(),f.failures.end());return p;
}
}
TEST(EnvelopeExecutionValues, NewElasticWallHasZeroGlobalPointsAndOriginalFailuresRemainExact) {
    qbat_catalog_test::Fixture f;auto packed=Packed(f);const auto prior=packed;
    const auto wall=detail::BuildGeometry(test::Box(),test::Decl(),test::Ids(),test::Units());
    std::vector<fe::ShellQephBindingInput> q(f.geometry.q.begin(),f.geometry.q.end());
    fe::ShellQephBindingInput extra;extra.source_parent_id=105;extra.reference=wall.reference_input;extra.nodes={5,6,7,8};q.push_back(extra);
    fe::ShellBatchBinding shells;
    ASSERT_EQ(shells.InitializeFormulations({{q.data(),f.triangles.data(),3,3,9},&f.geometry.b,1}).status,fe::ShellBindingStatus::Success);
    d::AppendDeclaredWall(packed,Environment(),MaterialDeclaration{},.001,Policy());
    ASSERT_EQ(packed.parents.size(),7u);
    for(unsigned i=0;i<6;++i) {
        EXPECT_TRUE(d::original::detail::Same(packed.parents[i],prior.parents[i]));
        EXPECT_EQ(packed.failure[i].policy,prior.failure[i].policy);
        EXPECT_EQ(output::Bits(packed.failure[i].constant.failure_strain),output::Bits(prior.failure[i].constant.failure_strain));
    }
    fe::ShellBatchPlasticityBinding catalog;
    ASSERT_EQ(catalog.InitializeExecutionCatalog(shells,packed.input()).status,fe::ShellPlasticityBindingStatus::Success);
    fe::ShellBatchFailureBinding failure;
    ASSERT_EQ(failure.InitializeExecution(catalog,packed.failure.data(),packed.failure.size()).status,fe::ShellPlasticityBindingStatus::Success);
    unsigned points=99;
    ASSERT_TRUE(catalog.MaterialPointCount(fe::ShellBindingFamily::Qeph,2,&points));
    EXPECT_EQ(points,0u);
    fe::ShellSectionLaw law;
    ASSERT_TRUE(catalog.Law(fe::ShellBindingFamily::Qeph,2,&law));
    EXPECT_EQ(law,fe::ShellSectionLaw::GlobalLaw1Npt0);
    EXPECT_EQ(packed.sections.back().through_thickness_points,3u);
    EXPECT_EQ(failure.parent(0)->policy,fe::ShellFailurePolicy::ConstantAllPoints);
    EXPECT_EQ(failure.parent(6)->policy,fe::ShellFailurePolicy::None);
    fe::ShellGlobalLaw1Profile profile;
    ASSERT_TRUE(catalog.GlobalLaw1Profile(fe::ShellBindingFamily::Qeph,2,&profile));
    EXPECT_EQ(profile.thickness,fe::ShellLaw1Thickness::Accepted);
    EXPECT_EQ(profile.coefficient_working_length_m,.001);
    auto missing=packed.failure;missing[0].policy=fe::ShellFailurePolicy::None;
    fe::ShellBatchFailureBinding rejected;
    EXPECT_NE(rejected.InitializeExecution(catalog,missing.data(),missing.size()).status,fe::ShellPlasticityBindingStatus::Success);
}
TEST(EnvelopeExecutionValues, SourceIdentityAndWorkingLengthMismatchesDoNotAppendPartialRows) {
    qbat_catalog_test::Fixture f;
    for(unsigned which=0;which<4;++which) {
        auto packed=Packed(f);auto e=Environment();double length=.001;
        if(which==0)e.material_id=f.materials[0].material_id;
        if(which==1)e.section_id=f.sections[0].section_id;
        if(which==2)e.element_id=f.parents[0].source_parent_id;
        if(which==3)length=.01;
        const auto parts=packed.parents.size(),materials=packed.materials.size(),sections=packed.sections.size();
        EXPECT_THROW(d::AppendDeclaredWall(packed,e,MaterialDeclaration{},length,Policy()),std::exception);
        EXPECT_EQ(packed.parents.size(),parts);EXPECT_EQ(packed.materials.size(),materials);EXPECT_EQ(packed.sections.size(),sections);
    }
}
}
