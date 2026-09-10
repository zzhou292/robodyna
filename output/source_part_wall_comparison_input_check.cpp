#include "SourcePartWallComparisonInput.h"
#include <gtest/gtest.h>

namespace crash::output::wall_comparison {
namespace {
Document Configuration() {
    Document d;d.Parse(R"({"owner_id":1,"run_id":2,"topology_id":3,"fixed_dt_s":1,"required_steps":32768,
      "frame_every":128,"interval_ledger_segments":[],"configuration_id":9,"qualification_id":10,
      "reference_nodes":[{"mass_kg":0.2,"isotropic_inertia_kg_m2":0.0001,"source_node_id":42}],
      "source_parents":[{"source_element_id":2213540,"local_connectivity":[0,1,2]}],
      "wall_setup":{"fixed_dt_s":1,"stiffness_per_area_N_m3":200000000000,"wall_binding_id":11,
        "nodal_area_m2":[[0.01,0.009,0.011,0.001]],"declared_leading_gap_m":0.0005},
      "future_physical_extension":{"must_match":true}})");return d;
}
Document Copy(const Value& d) {Document c;c.CopyFrom(d,c.GetAllocator());return c;}
TEST(SourceWallComparisonInput, OnlyExplicitTimestepAndOutputIdentityFieldsMayDiffer) {
    auto coarse=Configuration(),fine=Copy(coarse);
    fine["owner_id"].SetUint64(55);fine["run_id"].SetUint64(66);fine["topology_id"].SetUint64(77);
    fine["fixed_dt_s"].SetDouble(.5);fine["required_steps"].SetUint64(65536);fine["frame_every"].SetUint64(256);
    fine["wall_setup"]["fixed_dt_s"].SetDouble(.5);
    EXPECT_EQ(Encode(PhysicalConfiguration(coarse)),Encode(PhysicalConfiguration(fine)));
    fine["configuration_id"].SetUint64(19);
    EXPECT_NE(Encode(PhysicalConfiguration(coarse)),Encode(PhysicalConfiguration(fine)));
}
TEST(SourceWallComparisonInput, NativeMassInertiaSourceMappingWallCertificatesAndExtensionsMustMatch) {
    const auto original=Configuration();const auto expected=Encode(PhysicalConfiguration(original));
    for(unsigned variant=0;variant<6;++variant) {
        auto changed=Copy(original);
        if(variant==0)changed["reference_nodes"][0]["mass_kg"].SetDouble(.3);
        if(variant==1)changed["reference_nodes"][0]["isotropic_inertia_kg_m2"].SetDouble(.0002);
        if(variant==2)changed["source_parents"][0]["local_connectivity"][2].SetInt(3);
        if(variant==3)changed["wall_setup"]["nodal_area_m2"][0][1].SetDouble(.008);
        if(variant==4)changed["wall_setup"]["declared_leading_gap_m"].SetDouble(.0006);
        if(variant==5)changed["future_physical_extension"]["must_match"].SetBool(false);
        EXPECT_NE(expected,Encode(PhysicalConfiguration(changed)));
    }
    auto missing=Copy(original);missing.RemoveMember("wall_setup");
    EXPECT_THROW(PhysicalConfiguration(missing),std::runtime_error);
}
} // namespace
} // namespace crash::output::wall_comparison
