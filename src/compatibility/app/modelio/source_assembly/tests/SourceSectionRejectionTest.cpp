#include "SectionTestSupport.h"
#include "case/source_assembly/SourceAssemblyWallSetup.h"
#include <cmath>

namespace crash::modelio::assembly::test {
TEST(SourceSectionInputs, SourceLawUnitBitsAndLateMappingMutationsRejectAtomically) {
    const auto source=section::Load(false);const auto* original=source.data().nodes.data();
    for(unsigned fault=0;fault<13;++fault) {
        const auto bytes=section::Alter([&](auto& d) {
            auto& m=d["declarations"]["materials"][0];auto& parents=d["parent_bindings"];
            if(fault==0)d["schema"].SetString(Law44InventorySchema,d.GetAllocator());
            if(fault==1)m["material_law"].SetString("guess",d.GetAllocator());
            if(fault==2)m["material_law"].SetString("layered_law44",d.GetAllocator());
            if(fault==3)m.AddMember("hardening_curve_id",0,d.GetAllocator());
            if(fault==4)m.AddMember("rate_type",0,d.GetAllocator());
            if(fault==5)m["young_pa"].SetDouble(std::nextafter(m["young_pa"].GetDouble(),INFINITY));
            if(fault==6)m["poisson_ratio"].SetDouble(-0.0);
            if(fault==7)m["cards"][0]["blank_field_mask"].SetUint(224);
            if(fault==8)parents[parents.Size()-1]["curve_index"].SetUint(0);
            if(fault==9)parents[parents.Size()-1]["source_curve_id"].SetUint(1);
            if(fault==10)d["section_policy"]["law1"]["NIP"].SetUint(1);
            if(fault==11)d["declarations"]["units"]["length_to_m"].SetDouble(1);
            if(fault==12)d["section_policy"]["law1"]["case_integration_qualified"].SetBool(true);
        });
        EXPECT_THROW(SourceAssembly::ReadBytes(bytes,ExplicitTestIdentity(bytes)),std::runtime_error)<<fault;
        EXPECT_EQ(source.data().nodes.data(),original);
        EXPECT_EQ(source.data().identity.sha256,section::Identity(false).sha256);
    }
    EXPECT_EQ(section::Load(false).data().parents.size(),149u);
}

TEST(SourceSectionInputs, HostCompositionBudgetFailureRetriesAndWallCaseRejectsBeforePlacement) {
    namespace cases=crash::cases::source_assembly;
    const auto source=section::Load(false);
    cases::SourceAssemblyBindingOptions options(0x563342554447,MaterialRatePolicy::OpenRadiossDirectImportDefault);
    options.material_limits.max_owned_bytes=1;
    EXPECT_THROW(cases::SourceAssemblyBindings::Prepare(source,options),cases::SourceAssemblyBindingError);
    options.material_limits={};const auto bindings=cases::SourceAssemblyBindings::Prepare(source,options);
    ASSERT_TRUE(bindings.materials().prepared());
    cases::SourceAssemblyWallSetup wall;
    // An unopened canonical wall and empty settings must never be inspected or
    // allocated for a host-only source schema; the explicit schema gate wins.
    const crash::case_data::CanonicalWall canonical;
    const auto result=wall.Initialize(bindings,canonical,"",{});
    EXPECT_EQ(result.status,cases::SourceAssemblyWallStatus::InvalidInput);
    EXPECT_STREQ(result.message,"Layered source V3 is qualified for host binding only; wall case admission is separate");
    EXPECT_FALSE(wall.initialized());EXPECT_EQ(wall.startup_payload_bytes(),0u);
}
} // namespace crash::modelio::assembly::test
