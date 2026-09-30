#include "SectionTestSupport.h"
#include "case/source_assembly/SourceAssemblyBindings.h"
#include <algorithm>
#include <cmath>

namespace crash::modelio::assembly::test {
TEST(SourceSectionInputs, OriginalExternalMassAndJointNodesRemainReleasedEvidenceOnly) {
    for(bool mixed:{false,true}) {
        const auto source=section::Load(mixed);const auto& d=source.data();const auto& a=d.boundary.auxiliary;
        EXPECT_EQ(a.source_node_ids,(std::vector<SourceId>{2406582,2411580}));
        ASSERT_EQ(a.point_masses.size(),2u);ASSERT_EQ(a.spherical_joints.size(),2u);EXPECT_EQ(a.source_blocks.size(),3u);
        EXPECT_EQ(a.point_masses[0].source_element_id,2409471u);EXPECT_EQ(a.point_masses[1].source_element_id,2409487u);
        EXPECT_EQ(a.spherical_joints[0].source_joint_id,2200530u);EXPECT_EQ(a.spherical_joints[1].source_joint_id,2200533u);
        for(const auto& m:a.point_masses) {
            SameBits(m.supplied_mass_source,1.000100e-5);SameBits(m.supplied_mass_kg,1.000100e-5*1000);
            EXPECT_TRUE(std::none_of(d.nodes.begin(),d.nodes.end(),[&](const auto& n){return n.source_id==m.source_node_id;}));
        }
        for(const auto& block:a.source_blocks)EXPECT_EQ(output::Sha256(block.raw_text),block.sha256);
        for(auto gid:{2200007u,2200670u}) {
            const auto g=std::find_if(d.nodal_rigid_groups.begin(),d.nodal_rigid_groups.end(),[&](const auto& g){return g.id==gid;});
            ASSERT_NE(g,d.nodal_rigid_groups.end());EXPECT_FALSE(g->internal);EXPECT_EQ(g->external_nodes.size(),1u);
            EXPECT_EQ(g->members.size(),g->selected_global_nodes.size()+g->external_nodes.size());
            EXPECT_TRUE(std::binary_search(d.boundary.nodal_rigid_ids.begin(),d.boundary.nodal_rigid_ids.end(),gid));
        }
        const auto binding=cases::source_assembly::SourceAssemblyBindings::Prepare(source,
            {0x56334155584d,MaterialRatePolicy::OpenRadiossDirectImportDefault});
        EXPECT_EQ(binding.rigid_groups(),nullptr);EXPECT_EQ(binding.combined_mass(),nullptr);
        for(std::size_t n=0;n<d.nodes.size();++n) {
            const auto exact=binding.coefficients(n);const auto& native=binding.shells().nodes()[n].native;
            SameBits(exact.mass,native.mass);SameBits(exact.isotropic_inertia,native.isotropic_inertia);
        }
    }
}

TEST(SourceSectionInputs, MissingChangedAuxiliaryCardsAndOwnerClaimsRejectWithoutPublication) {
    const auto original=section::Load(false);const auto* held=original.data().nodes.data();
    for(unsigned fault=0;fault<12;++fault) {
        const auto bytes=section::Alter([&](auto& d) {
            auto& a=d["attachments"]["auxiliary_frontier"];auto& m=a["point_masses"][1];
            auto& block=a["source_blocks"][2];
            if(fault==0)a["point_masses"].PopBack();
            if(fault==1)a["spherical_joints"].PopBack();
            if(fault==2)block["raw_text"].SetString("*ELEMENT_MASS\n",d.GetAllocator());
            if(fault==3)m["supplied_mass_kg"].SetDouble(std::nextafter(m["supplied_mass_kg"].GetDouble(),INFINITY));
            if(fault==4)m["supplied_mass_source"].SetDouble(std::nextafter(m["supplied_mass_source"].GetDouble(),INFINITY));
            if(fault==5)m["source_node_id"].SetUint(2346746);
            if(fault==6)a["source_node_ids"].PopBack();
            if(fault==7)block["cards"][m["source_card_index"].GetUint()]["text"].SetString("altered",d.GetAllocator());
            if(fault==8)a["spherical_joints"][1]["source_joint_id"].SetUint(2200534);
            if(fault==9)a["selected_owner_mass_added"].SetBool(true);
            if(fault==10)a["source_blocks"][0]["keyword"].SetString("*CONSTRAINED_JOINT_CYLINDRICAL_ID",d.GetAllocator());
            if(fault==11)a["source_blocks"].PopBack();
        });
        EXPECT_THROW(SourceAssembly::ReadBytes(bytes,ExplicitTestIdentity(bytes)),std::runtime_error)<<fault;
        EXPECT_EQ(original.data().nodes.data(),held);
    }
    EXPECT_EQ(section::Load(false).data().boundary.auxiliary.point_masses.size(),2u);
}
} // namespace crash::modelio::assembly::test
