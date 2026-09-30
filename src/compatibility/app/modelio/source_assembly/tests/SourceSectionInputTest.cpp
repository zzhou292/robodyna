#include "SectionTestSupport.h"
#include "AnalyticTestSupport.h"
#include "case/source_assembly/SourceAssemblyBindings.h"
#include "output/source_assembly/SourceAssemblyWallSchema.h"
#include <cmath>

namespace crash::modelio::assembly::test {
namespace fe=tl::fea;
namespace cases=crash::cases::source_assembly;

TEST(SourceSectionInputs, OriginalElasticArmAndMixedFamiliesBindEveryTypedNativeParent) {
    for(bool mixed:{false,true}) {
        SCOPED_TRACE(mixed);const auto source=section::Load(mixed);const auto& d=source.data();
        EXPECT_EQ(d.schema,SectionInventorySchema);EXPECT_EQ(d.parents.size(),mixed?631u:149u);
        EXPECT_EQ(d.nodes.size(),mixed?683u:145u);
        EXPECT_EQ(d.qeph_count,mixed?581u:135u);EXPECT_EQ(d.t3_count,mixed?50u:14u);
        EXPECT_EQ(d.curves.size(),mixed?1u:0u);EXPECT_EQ(d.materials.size(),mixed?3u:1u);
        const SourceAssemblyMaterialInput input(source,MaterialRatePolicy::OpenRadiossDirectImportDefault);
        if(!mixed)EXPECT_EQ(input.input().curves,nullptr);
        const auto bindings=cases::SourceAssemblyBindings::Prepare(source,{0x563353454354,
            MaterialRatePolicy::OpenRadiossDirectImportDefault});
        const auto& catalog=bindings.materials();ASSERT_TRUE(catalog.heterogeneous_sections());
        EXPECT_EQ(bindings.source().data().identity.sha256,section::Identity(mixed).sha256);
        fe::ShellSectionCounts q,t;ASSERT_TRUE(catalog.Counts(fe::ShellBindingFamily::Qeph,&q));
        ASSERT_TRUE(catalog.Counts(fe::ShellBindingFamily::T3,&t));
        EXPECT_EQ(q.law1,135u);EXPECT_EQ(t.law1,14u);
        EXPECT_EQ(q.law44,mixed?446u:0u);EXPECT_EQ(t.law44,mixed?36u:0u);
        for(const auto& p:d.parents) {
            const auto family=p.family==ShellFamily::Qeph?fe::ShellBindingFamily::Qeph:fe::ShellBindingFamily::T3;
            const auto& m=d.materials[p.material_index];const auto* mapped=catalog.parent(p.index);
            ASSERT_NE(mapped,nullptr);EXPECT_EQ(mapped->source_parent_id,p.source_id);EXPECT_EQ(mapped->source_part_id,p.part_id);
            fe::ShellSectionLaw law;ASSERT_TRUE(catalog.Law(family,p.family_index,&law));
            tl::material::ShellElasticLaw1PointParameters elastic;fe::sections::PointParameters plastic;
            elastic.young_pa=71;plastic.young_pa=83;
            if(m.law==MaterialLaw::LayeredLaw1) {
                EXPECT_EQ(law,fe::ShellSectionLaw::LayeredLaw1Nip3);EXPECT_EQ(p.part_id,2000511u);
                EXPECT_EQ(p.curve_index,NoCurveIndex);EXPECT_EQ(p.curve_id,0u);
                EXPECT_FALSE(catalog.Parameters(family,p.family_index,&plastic));SameBits(plastic.young_pa,83);
                ASSERT_TRUE(catalog.ElasticParameters(family,p.family_index,&elastic));
                SameBits(elastic.young_pa,m.young_pa);SameBits(elastic.poisson_ratio,m.poisson_ratio);
                SameBits(elastic.density_kg_m3,m.density_kg_m3);
            } else {
                EXPECT_EQ(law,fe::ShellSectionLaw::LayeredLaw44Nip3);
                EXPECT_FALSE(catalog.ElasticParameters(family,p.family_index,&elastic));SameBits(elastic.young_pa,71);
                ASSERT_TRUE(catalog.Parameters(family,p.family_index,&plastic));SameBits(plastic.young_pa,m.young_pa);
                EXPECT_EQ(plastic.hardening,p.part_id==2000064?tl::material::ShellPlasticityHardeningKind::LinearLaw44:
                    tl::material::ShellPlasticityHardeningKind::Tabulated);
            }
        }
        for(std::size_t i=0;i<d.nodes.size();++i) {
            EXPECT_EQ(bindings.shells().nodes()[i].source_id,d.nodes[i].source_id);
            EXPECT_GT(bindings.coefficients(i).mass,0);
        }
    }
}

TEST(SourceSectionInputs, TypedAdapterOwnsMixedScalarsCurvesAndLegacyAdmissionStaysClosed) {
    auto initial=std::make_unique<SourceAssemblyMaterialInput>(section::Load(true),MaterialRatePolicy::OpenRadiossDirectImportDefault);
    auto copy=*initial;initial.reset();auto moved=std::move(copy);const auto input=moved.input();
    ASSERT_EQ(input.material_count,3u);ASSERT_EQ(input.curve_count,1u);
    EXPECT_GT(input.curves[0].curve.yield_stress_pa[0],0);
    for(std::size_t i=0;i<input.material_count;++i)if(input.materials[i].law==fe::ShellSectionLaw::LayeredLaw1Nip3) {
        const auto& m=input.materials[i];const tl::material::TabulatedShellPlasticityRate rate{};
        EXPECT_EQ(m.curve_id,0u);EXPECT_EQ(m.rate.enabled,rate.enabled);
        SameBits(m.rate.cowper_symonds_c_per_s,rate.cowper_symonds_c_per_s);SameBits(m.rate.cowper_symonds_p,rate.cowper_symonds_p);
        SameBits(m.rate.cutoff_hz,rate.cutoff_hz);SameBits(m.linear.initial_yield_pa,0);
        SameBits(m.linear.tangent_modulus_pa,0);
    }
    const SourceAssemblyShellInput shell(moved.source());fe::ShellBatchBinding geometry;
    ASSERT_EQ(geometry.Initialize(shell.input(),{}).status,fe::ShellBindingStatus::Success);
    fe::ShellBatchPlasticityBinding legacy;
    EXPECT_EQ(legacy.Initialize(geometry,input).status,fe::ShellPlasticityBindingStatus::InvalidMaterial);
    EXPECT_FALSE(legacy.prepared());
    EXPECT_EQ(legacy.InitializeSections(geometry,input).status,fe::ShellPlasticityBindingStatus::Success);
    EXPECT_THROW(output::assembly::CheckWallSourceSchema(SectionInventorySchema),std::runtime_error);
    EXPECT_THROW(output::assembly::CheckWallSourceSchema(Law44InventorySchema),std::runtime_error);
    EXPECT_NO_THROW(output::assembly::CheckWallSourceSchema(InventorySchema));
    EXPECT_EQ(Load().data().schema,InventorySchema);
    EXPECT_EQ(analytic::Load(false).data().schema,Law44InventorySchema);
}
} // namespace crash::modelio::assembly::test
