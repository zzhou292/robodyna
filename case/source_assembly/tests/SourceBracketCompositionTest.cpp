#include "SourceAssemblyBindingTestSupport.h"
#include "case/source_assembly/SourceAssemblyInitialKinetic.h"

namespace crash::cases::source_assembly::test {
namespace {
SourceAssemblyBindingOptions BracketOptions() {
    auto value=Options();
    value.spotweld=source::SpotweldDeclaration{
        source::SpotweldPolicy::OpenRadiossTonneMillimetreSecondDirectImport,383348001108};
    return value;
}
source::SourceAssembly BracketSource() {
    const auto* path=std::getenv("ROBO_DYNA_SOURCE_BRACKET_INVENTORY");
    output::Require(path&&*path,"Explicit bracket source inventory required");
    return source::SourceAssembly::Read(path,source::PinnedYarisSevenPartInventory());
}
}
TEST(SourceBracketComposition, CompleteSourceUsesExactlyOnceNativeEndpointContributions) {
    const auto b=SourceAssemblyBindings::Prepare(BracketSource(),BracketOptions());
    ASSERT_NE(b.connectors(),nullptr);ASSERT_NE(b.combined_mass(),nullptr);
    EXPECT_EQ(b.shells().node_count(),1093u);EXPECT_EQ(b.shells().qeph_count(),845u);
    EXPECT_EQ(b.shells().t3_count(),114u);ASSERT_EQ(b.connectors()->connection_count(),1u);
    EXPECT_TRUE(b.combined_mass()->Matches(b.shells()));EXPECT_TRUE(b.combined_mass()->Matches(*b.connectors()));
    const auto& c=b.connectors()->connections()[0];
    EXPECT_EQ(c.source_element_id,2101297u);EXPECT_EQ(c.source_node_id[0],2181504u);EXPECT_EQ(c.source_node_id[1],2204838u);
    long double total_mass=0,total_j=0,connector_mass=0,connector_j=0;
    for(std::size_t n=0;n<b.shells().node_count();++n) {
        const auto p=b.coefficients(n);const auto& shell=b.shells().nodes()[n].native;
        const bool endpoint=n==c.global_node[0]||n==c.global_node[1];
        const double m=endpoint?.0005:0,j=endpoint?5e-9:0;
        SameBits(p.connector_mass,m);SameBits(p.connector_inertia,j);
        SameBits(p.mass,shell.mass+m);SameBits(p.isotropic_inertia,shell.isotropic_inertia+j);
        SameBits(p.shell.physical_inertia,shell.physical_inertia);SameBits(p.shell.added_inertia,shell.added_inertia);
        total_mass+=p.mass;total_j+=p.isotropic_inertia;connector_mass+=m;connector_j+=j;
    }
    Near(connector_mass,.001L);Near(connector_j,1e-8L);
    Near(b.combined_mass()->totals().mass,total_mass);Near(b.combined_mass()->totals().isotropic_inertia,total_j);
    ASSERT_EQ(b.rigid_groups()->group_count(),6u);
    for(std::size_t i=0;i<b.rigid_groups()->member_count();++i) {
        const auto& member=b.rigid_groups()->members()[i];const auto p=b.coefficients(member.global_node);
        EXPECT_EQ(p.connector_mass,0);EXPECT_EQ(p.connector_inertia,0);
        SameBits(member.mass_kg,p.mass);SameBits(member.total_inertia_kg_m2,p.isotropic_inertia);
    }
}
TEST(SourceBracketComposition, InitialWallKineticIncludesConnectorAndEachPrimaryOnce) {
    const auto b=SourceAssemblyBindings::Prepare(BracketSource(),BracketOptions());
    SourceAssemblyInitialKinetic energy;ASSERT_TRUE(EncloseSourceAssemblyInitialKinetic(b,8,&energy));
    long double native=0,primary=0;
    for(std::size_t n=0;n<b.shells().node_count();++n)native+=32.L*b.coefficients(n).mass;
    for(std::size_t g=0;g<b.rigid_groups()->group_count();++g)
        primary+=b.rigid_groups()->groups()[g].regularization.primary_mass_kg;
    EXPECT_LE(energy.native_nodes.lower,native);EXPECT_GE(energy.native_nodes.upper,native);
    EXPECT_LE(energy.with_aggregate_groups.lower,native+32.L*primary);
    EXPECT_GE(energy.with_aggregate_groups.upper,native+32.L*primary);
    long double shell=0;for(const auto& n:b.shells().nodes())shell+=32.L*n.native.mass;
    Near(native-shell,.032L);
}
TEST(SourceBracketComposition, IndependentPolicyModelAndCombinedMassFailuresPreserveRetainedInputs) {
    auto input=std::make_unique<source::SourceAssembly>(BracketSource());
    const auto kept=SourceAssemblyBindings::Prepare(*input,BracketOptions());input.reset();
    const auto* nodes=kept.combined_mass()->nodes().data();
    SourceAssemblyBindings copied(kept),moved(std::move(copied));
    EXPECT_EQ(moved.combined_mass(),kept.combined_mass());EXPECT_EQ(copied.connectors(),kept.connectors());
    for(unsigned fault=0;fault<5;++fault) {
        auto options=BracketOptions();
        if(fault==0)options.spotweld->policy=static_cast<source::SpotweldPolicy>(0);
        if(fault==1)options.spotweld->generated_property_id=2000204;
        if(fault==2)options.connector_limits.max_connections=0;
        if(fault==3)options.mass_limits.max_nodes=1092;
        if(fault==4)options.mass_limits.max_host_bytes=1;
        try { (void)SourceAssemblyBindings::Prepare(kept.source(),options);FAIL()<<"Expected explicit startup rejection"; }
        catch(const SourceAssemblyBindingError& error) {
            EXPECT_EQ(error.stage,fault<3?SourceAssemblyBindingStage::Connectors:SourceAssemblyBindingStage::CombinedMass);
        }
        EXPECT_EQ(nodes,kept.combined_mass()->nodes().data());
        EXPECT_TRUE(kept.combined_mass()->Matches(*kept.connectors()));
    }
}
TEST(SourceBracketComposition, ShellOnlyBindingRetainsExactCoefficientsAndRejectsUnusedConnectorPolicy) {
    const auto b=SourceAssemblyBindings::Prepare(Load(),Options());
    EXPECT_EQ(b.connectors(),nullptr);EXPECT_EQ(b.combined_mass(),nullptr);EXPECT_EQ(b.spotweld_declaration(),nullptr);
    for(std::size_t n=0;n<b.shells().node_count();++n) {
        const auto p=b.coefficients(n);const auto& shell=b.shells().nodes()[n].native;
        SameBits(p.mass,shell.mass);SameBits(p.isotropic_inertia,shell.isotropic_inertia);
        EXPECT_EQ(p.connector_mass,0);EXPECT_EQ(p.connector_inertia,0);
    }
    EXPECT_THROW(SourceAssemblyBindings::Prepare(b.source(),BracketOptions()),SourceAssemblyBindingError);
}
TEST(SourceBracketComposition, AuthenticatedFixtureWithGroupedWeldEndpointRejectsBeforeNativeAttachment) {
    const auto original=BracketSource();const auto& data=original.data();
    const auto part=std::find_if(data.parts.begin(),data.parts.end(),[](const auto& p){return p.id==2000145;});
    ASSERT_NE(part,data.parts.end());std::uint64_t grouped_node=0;
    for(const auto& group:data.nodal_rigid_groups)if(group.internal)
        for(const auto node:group.selected_global_nodes)
            if(std::find(part->nodes.begin(),part->nodes.end(),node)!=part->nodes.end())grouped_node=data.nodes[node].source_id;
    ASSERT_NE(grouped_node,0u);
    const auto bytes=output::ReadBounded(std::getenv("ROBO_DYNA_SOURCE_BRACKET_INVENTORY"),source::PinnedYarisSevenPartInventory().bytes);
    output::Document document;document.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());
    ASSERT_FALSE(document.HasParseError());auto& allocator=document.GetAllocator();
    for(auto& weld:document["attachments"]["spotwelds"].GetArray())
        if(std::string(weld["classification"].GetString())=="internal") {
            weld["weld"]["node_ids"][0u].SetUint64(grouped_node);
            auto& card=weld["weld"]["cards"][1u];std::ostringstream field;field<<std::setw(9)<<grouped_node;
            const auto text=field.str();card["fields"][0u].SetString(text.c_str(),allocator);
            const auto raw=text+card["fields"][1u].GetString();card["raw_text"].SetString(raw.c_str(),allocator);
            for(auto& membership:weld["selected_membership"].GetArray())
                if(membership["part_id"].GetUint64()==2000145)membership["node_ids"][0u].SetUint64(grouped_node);
        }
    rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);ASSERT_TRUE(document.Accept(writer));
    const std::string changed(buffer.GetString(),buffer.GetSize());source::test::Scratch scratch;
    const auto fixture=source::SourceAssembly::Read(scratch.Write(changed),source::test::ExplicitTestIdentity(changed));
    try {(void)SourceAssemblyBindings::Prepare(fixture,BracketOptions());FAIL()<<"Grouped connector endpoint admitted";}
    catch(const SourceAssemblyBindingError& error) {
        EXPECT_EQ(error.stage,SourceAssemblyBindingStage::Connectors);EXPECT_EQ(error.source_element_id,2101297u);
        EXPECT_NE(std::string(error.what()).find("rigid group"),std::string::npos);
    }
    EXPECT_EQ(data.identity.sha256,source::PinnedYarisSevenPartInventory().sha256);
}
} // namespace crash::cases::source_assembly::test
