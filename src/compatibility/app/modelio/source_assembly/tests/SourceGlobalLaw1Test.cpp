#include "SectionTestSupport.h"
#include "../SourceAssemblyShellInput.h"
#include "../SourceAssemblyMaterialInput.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include <limits>
namespace crash::modelio::assembly::test {
namespace fe=tl::fea;
TEST(SourceGlobalLaw1, ActualMixedDeclarationsResolvePerParentWithoutChangingRawTables) {
    const auto source=section::Load(true);const auto& d=source.data();
    const SourceAssemblyShellInput geometry(source,QephMetricProfile::AuthenticatedSourceLength);
    const SourceAssemblyMaterialInput input(geometry,MaterialRatePolicy::OpenRadiossDirectImportDefault,
        Law1ExecutionProfile::NativeA62OrdinaryNpt0);
    const auto values=input.input();
    fe::ShellBatchBinding shells;
    ASSERT_EQ(shells.Initialize(geometry.input(),{}).status,fe::ShellBindingStatus::Success);
    fe::ShellBatchPlasticityBinding catalog;
    ASSERT_EQ(catalog.InitializeExecutionCatalog(shells,values).status,fe::ShellPlasticityBindingStatus::Success);
    std::size_t q=0,t=0;
    for(const auto& p:d.parents) {
        const auto& m=d.materials[p.material_index];const auto& s=d.sections[p.section_index];
        const auto driver=ResolveLaw1SourceDriver(m,s);const auto& row=values.parents[p.index];
        ASSERT_EQ(row.material_id,m.id);ASSERT_EQ(row.section_id,s.id);
        EXPECT_EQ(values.sections[p.section_index].through_thickness_points,3u);
        EXPECT_EQ(s.through_thickness_points,3u);
        fe::ShellSectionLaw law;unsigned points=99;
        ASSERT_TRUE(catalog.Law(row.family,row.family_index,&law));
        ASSERT_TRUE(catalog.MaterialPointCount(row.family,row.family_index,&points));
        if(m.law==MaterialLaw::LayeredLaw1) {
            ASSERT_TRUE(driver.available());EXPECT_EQ(driver.ismstr(),2);EXPECT_EQ(driver.ithick(),1);
            EXPECT_EQ(driver.iplas(),1);EXPECT_EQ(driver.raw_nip(),3u);EXPECT_EQ(driver.resolved_npt(),0);
            EXPECT_EQ(law,fe::ShellSectionLaw::GlobalLaw1Npt0);EXPECT_EQ(points,0u);
            fe::ShellGlobalLaw1Profile effective;
            ASSERT_TRUE(catalog.GlobalLaw1Profile(row.family,row.family_index,&effective));
            EXPECT_EQ(effective.thickness,fe::ShellLaw1Thickness::Accepted);
            SameBits(effective.coefficient_working_length_m,d.units.length_to_m);
            row.family==fe::ShellBindingFamily::Qeph?++q:++t;
        } else {
            EXPECT_EQ(driver.status(),Law1DriverStatus::NotElastic);
            EXPECT_EQ(law,fe::ShellSectionLaw::LayeredLaw44Nip3);EXPECT_EQ(points,3u);
            EXPECT_TRUE(fe::SameShellParentExecution(row.execution,{}));
        }
    }
    EXPECT_EQ(q,135u);EXPECT_EQ(t,14u);
    EXPECT_EQ(source.data().identity.sha256,section::Identity(true).sha256);
    const auto tagged=fe::ShellBatchLayeredSection::GlobalLaw1();
    EXPECT_EQ(tagged.elastic(),nullptr);EXPECT_EQ(tagged.plastic(),nullptr);EXPECT_EQ(tagged.one_point(),nullptr);
}
TEST(SourceGlobalLaw1, LegacyDefaultsAndCopiedInputsKeepExactDeclarations) {
    const auto source=section::Load(true);
    const SourceAssemblyShellInput refs(source);
    const SourceAssemblyMaterialInput old(source,MaterialRatePolicy::OpenRadiossDirectImportDefault);
    auto initial=std::make_unique<SourceAssemblyMaterialInput>(refs,MaterialRatePolicy::OpenRadiossDirectImportDefault,
        Law1ExecutionProfile::LegacyLayered);
    auto copy=*initial;initial.reset();const auto a=old.input(),b=copy.input();
    ASSERT_EQ(a.parent_count,b.parent_count);ASSERT_EQ(a.material_count,b.material_count);ASSERT_EQ(a.section_count,b.section_count);
    for(std::size_t i=0;i<a.parent_count;++i) {
        EXPECT_EQ(a.parents[i].source_parent_id,b.parents[i].source_parent_id);
        EXPECT_EQ(a.parents[i].material_id,b.parents[i].material_id);EXPECT_EQ(a.parents[i].section_id,b.parents[i].section_id);
        EXPECT_TRUE(fe::SameShellParentExecution(a.parents[i].execution,b.parents[i].execution));
        EXPECT_TRUE(fe::SameShellParentExecution(b.parents[i].execution,{}));
    }
    for(std::size_t i=0;i<a.material_count;++i) {
        EXPECT_EQ(a.materials[i].law,b.materials[i].law);SameBits(a.materials[i].young_pa,b.materials[i].young_pa);
        SameBits(a.materials[i].density_kg_m3,b.materials[i].density_kg_m3);
    }
    for(std::size_t i=0;i<a.section_count;++i) {
        EXPECT_EQ(a.sections[i].through_thickness_points,b.sections[i].through_thickness_points);
        SameBits(a.sections[i].thickness_m,b.sections[i].thickness_m);
    }
    EXPECT_THROW((SourceAssemblyMaterialInput(refs,MaterialRatePolicy::OpenRadiossDirectImportDefault,
        Law1ExecutionProfile::NativeA62OrdinaryNpt0)),std::exception);
}
TEST(SourceGlobalLaw1, IndependentUnitContextsAndUnknownInputsAreExplicit) {
    const auto source=section::Load(false);const auto& d=source.data();
    const auto driver=ResolveLaw1SourceDriver(d.materials[0],d.sections[0]);ASSERT_TRUE(driver.available());
    for(double length:{1.,.001,.01,.0254}) {
        const SourceUnits units{2.5,length,.25};
        const auto metric=QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,units);
        const auto policy=Law1ExecutionPolicy::Resolve(Law1ExecutionProfile::NativeA62OrdinaryNpt0,units,metric);
        EXPECT_TRUE(policy.requires_ordinary_explicit_defaults());SameBits(policy.coefficient_working_length_m(),length);
        for(auto family:{fe::ShellBindingFamily::Qeph,fe::ShellBindingFamily::T3}) {
            const auto result=policy.Parent(driver,family,fe::ShellSectionFormulation::LayeredNip3);
            EXPECT_EQ(result.policy,fe::ShellParentExecutionPolicy::GlobalLaw1Npt0);
            SameBits(result.global_law1.coefficient_working_length_m,length);
        }
        EXPECT_THROW(policy.Parent(driver,fe::ShellBindingFamily::Qbat,fe::ShellSectionFormulation::LayeredNip3),std::exception);
        EXPECT_THROW(policy.Parent({},fe::ShellBindingFamily::Qeph,fe::ShellSectionFormulation::LayeredNip3),std::exception);
        const auto other=QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,{2.5,2*length,.25});
        EXPECT_THROW(Law1ExecutionPolicy::Resolve(Law1ExecutionProfile::NativeA62OrdinaryNpt0,units,other),std::exception);
    }
    const auto metric=QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,d.units);
    for(double value:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        auto units=d.units;units.length_to_m=value;
        EXPECT_THROW(Law1ExecutionPolicy::Resolve(Law1ExecutionProfile::NativeA62OrdinaryNpt0,units,metric),std::exception);
    }
    EXPECT_THROW(Law1ExecutionPolicy::Resolve(static_cast<Law1ExecutionProfile>(99),d.units,metric),std::exception);
}
TEST(SourceGlobalLaw1, RawOptionsCannotMintAnUnsupportedDriver) {
    const auto source=section::Load(false);const auto& d=source.data();
    for(unsigned fault=0;fault<8;++fault) {
        auto m=d.materials[0];auto s=d.sections[0];
        switch(fault) {
        case 0:s.cards.clear();break;
        case 1:s.cards[0].blank_mask=0;break;
        case 2:s.source_elform=9;break;
        case 3:s.through_thickness_points=1;break;
        case 4:s.cards[0].values[3]=2;break;
        case 5:m.cards[0].values[4]=1;break;
        case 6:m.law=MaterialLaw::LayeredLaw44;break;
        case 7:s.thickness_m[3]*=2;break;
        }
        EXPECT_EQ(ResolveLaw1SourceDriver(m,s).status(),Law1DriverStatus::Unavailable);
    }
    const auto driver=ResolveLaw1SourceDriver(d.materials[0],d.sections[0]);
    EXPECT_EQ(driver.material_id(),d.materials[0].id);EXPECT_EQ(driver.section_id(),d.sections[0].id);
    EXPECT_EQ(d.materials[0].law,MaterialLaw::LayeredLaw1);EXPECT_EQ(d.sections[0].through_thickness_points,3u);
}
TEST(SourceGlobalLaw1, SharedRawSidIsNotRewrittenByPerParentElasticSelection) {
    const auto source=section::Load(true);const auto& d=source.data();
    const auto elastic=std::find_if(d.materials.begin(),d.materials.end(),[](const auto& m){return m.law==MaterialLaw::LayeredLaw1;});
    const auto plastic=std::find_if(d.materials.begin(),d.materials.end(),[](const auto& m){return m.law==MaterialLaw::LayeredLaw44;});
    ASSERT_NE(elastic,d.materials.end());ASSERT_NE(plastic,d.materials.end());
    const auto p=std::find_if(d.parents.begin(),d.parents.end(),[&](const auto& p){return p.material_id==elastic->id;});
    ASSERT_NE(p,d.parents.end());const auto& shared=d.sections[p->section_index];
    const auto policy=Law1ExecutionPolicy::Resolve(Law1ExecutionProfile::NativeA62OrdinaryNpt0,d.units,
        QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength,d.units));
    const auto a=policy.Parent(ResolveLaw1SourceDriver(*elastic,shared),fe::ShellBindingFamily::Qeph,fe::ShellSectionFormulation::LayeredNip3);
    const auto b=policy.Parent(ResolveLaw1SourceDriver(*plastic,shared),fe::ShellBindingFamily::T3,fe::ShellSectionFormulation::LayeredNip3);
    EXPECT_EQ(a.policy,fe::ShellParentExecutionPolicy::GlobalLaw1Npt0);
    EXPECT_EQ(b.policy,fe::ShellParentExecutionPolicy::FromSection);
    EXPECT_EQ(shared.through_thickness_points,3u);EXPECT_EQ(shared.id,p->section_id);
    EXPECT_EQ(elastic->law,MaterialLaw::LayeredLaw1);EXPECT_EQ(plastic->law,MaterialLaw::LayeredLaw44);
}

}
