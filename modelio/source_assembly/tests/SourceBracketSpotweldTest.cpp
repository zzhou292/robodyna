#include "AssemblyTestSupport.h"
#include "modelio/source_assembly/SourceAssemblySpotweldInput.h"
#include "lib_src/elements/type25/Type25Model.h"
#include <cmath>

namespace crash::modelio::assembly::test {
namespace spring=tl::fea::type25;
namespace {
constexpr std::uint64_t Instance=383348001107,PropertyId=383348001108;
constexpr SpotweldDeclaration Policy{SpotweldPolicy::OpenRadiossTonneMillimetreSecondDirectImport,PropertyId};
SourceAssembly Bracket() {
    const auto* path=std::getenv("ROBO_DYNA_SOURCE_BRACKET_INVENTORY");
    output::Require(path&&*path,"Explicit original bracket fixture is required");
    return SourceAssembly::Read(path,PinnedYarisSevenPartInventory());
}
}
TEST(SourceBracketSpotweld, ExplicitSourcePolicyRetainsLiteralWeldAndResolvedNativeCoefficients) {
    const SourceAssemblySpotweldInput input(Bracket(),Instance,Policy);
    const auto data=input.input();
    EXPECT_EQ(data.source_instance_id,Instance); EXPECT_EQ(data.global_node_count,1093u);
    ASSERT_EQ(data.connection_count,1u); ASSERT_EQ(data.property_count,1u);
    EXPECT_EQ(data.source_units.mass_to_kg,1000.); EXPECT_EQ(data.source_units.length_to_m,.001);
    EXPECT_EQ(data.source_units.time_to_s,1.);
    const auto& weld=data.connections[0];
    EXPECT_EQ(weld.source_element_id,2101297u);
    EXPECT_EQ(weld.source_node_id[0],2181504u); EXPECT_EQ(weld.source_node_id[1],2204838u);
    EXPECT_EQ(weld.property_index,0u); EXPECT_EQ(data.properties[0].source_property_id,PropertyId);
    const auto& source=input.source().data();
    for(unsigned e=0;e<2;++e) {
        const auto& original=source.nodes[weld.global_node[e]];
        EXPECT_EQ(original.source_id,weld.source_node_id[e]);
        SameBits(original.position_m.x,weld.position[e].x);
        SameBits(original.position_m.y,weld.position[e].y);
        SameBits(original.position_m.z,weld.position[e].z);
    }
    const auto& p=data.properties[0].property;
    EXPECT_DOUBLE_EQ(p.mass_kg,.001); EXPECT_DOUBLE_EQ(p.isotropic_inertia_kg_m2,1e-8);
    for(unsigned c=0;c<4;++c) {
        EXPECT_DOUBLE_EQ(p.stiffness[c],c<2?1e8:1000.);
        EXPECT_EQ(p.damping[c],0.); EXPECT_EQ(p.failure_weight[c],1.);
        EXPECT_EQ(p.failure_exponent[c],2.);
        EXPECT_DOUBLE_EQ(p.failure_positive[c],c<2?1e30:1e27);
        EXPECT_EQ(p.failure_negative[c],-p.failure_positive[c]);
    }
    spring::Model model; const auto report=model.Initialize(data);
    ASSERT_TRUE(report)<<report.message;
    EXPECT_GT(model.references()[0].length_m,.003);
    EXPECT_LT(model.references()[0].length_m,.011);
    for(unsigned e=0;e<2;++e) {
        EXPECT_EQ(model.endpoint_mass()[e].source_element_id,2101297u);
        EXPECT_DOUBLE_EQ(model.endpoint_mass()[e].mass_kg,.0005);
        EXPECT_DOUBLE_EQ(model.endpoint_mass()[e].isotropic_inertia_kg_m2,5e-9);
    }
}
TEST(SourceBracketSpotweld, MissingPolicyAndOriginalSectionIdentityCannotBecomeGeneratedProperty) {
    const auto source=Bracket();
    EXPECT_THROW(SourceAssemblySpotweldInput(source,0,Policy),std::runtime_error);
    EXPECT_THROW(SourceAssemblySpotweldInput(source,Instance,{Policy.policy,0}),std::runtime_error);
    EXPECT_THROW(SourceAssemblySpotweldInput(source,Instance,{static_cast<SpotweldPolicy>(0),PropertyId}),std::runtime_error);
    EXPECT_THROW(SourceAssemblySpotweldInput(source,Instance,{Policy.policy,2000204}),std::runtime_error);
    EXPECT_THROW(SourceAssemblySpotweldInput(Load(),Instance,Policy),std::runtime_error);
}
TEST(SourceBracketSpotweld, OwnedSourceAndFreshInputRangesSurviveCopiesAndProducerLifetime) {
    auto copy=[] {
        const SourceAssemblySpotweldInput source(Bracket(),Instance,Policy);
        return SourceAssemblySpotweldInput(source);
    }();
    spring::Model first; ASSERT_TRUE(first.Initialize(copy.input()));
    SourceAssemblySpotweldInput moved(std::move(copy));
    spring::Model second; ASSERT_TRUE(second.Initialize(moved.input()));
    EXPECT_TRUE(first.Matches(second));
    EXPECT_EQ(moved.input().connections[0].source_element_id,2101297u);
}
} // namespace crash::modelio::assembly::test
