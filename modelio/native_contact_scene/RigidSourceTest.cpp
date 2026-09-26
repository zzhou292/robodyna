#include "DeclaredSource.h"
#include "output/BoundedArrayJson.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::modelio::native_scene {
namespace {
std::filesystem::path RigidExport(){const auto* p=std::getenv("ROBO_DYNA_NATIVE_RIGID_SCENE_EXPORT");if(!p)throw std::runtime_error("Explicit rigid export required");return p;}
}
TEST(RigidSceneSource, ActualDeclarationKeepsAuxiliaryPrimaryOutsidePhysicalDomain) {
    const auto file=RigidExport();const auto source=DeclaredSource::Read(file,output::Sha256(output::ReadBounded(file,4u<<20)));
    const auto& d=source.data();ASSERT_TRUE(d.rigid_patch);EXPECT_EQ(d.definition_schema,"robo_dyna.native_contact_scene.v3");
    EXPECT_EQ(d.nodes.size(),18u);EXPECT_EQ(d.contact_surface,DeclaredContactSurface::AllShells);
    EXPECT_EQ(d.rigid_patch->reference_primary_id,19u);EXPECT_EQ(d.rigid_patch->source_part_id,2u);
    const std::vector<std::uint64_t> members{10,11,12,13,14,15,16,17,18};
    const std::vector<std::uint64_t> centroid{10,11,14,13,12,15,17,16,18};
    EXPECT_EQ(d.rigid_patch->member_source_ids,members);EXPECT_EQ(d.rigid_patch->centroid_source_order,centroid);
    for(const auto& node:d.nodes)EXPECT_NE(node.id,d.rigid_patch->reference_primary_id);
    EXPECT_EQ(d.rigid_patch->reference_primary_mm.z,2.);EXPECT_EQ(d.step_cap_s,3e-7);
}
TEST(RigidSceneSource, RehashedMetadataCannotSubstituteControlsPrimaryOrMembership) {
    for(unsigned fault=0;fault<6;++fault) {
        SCOPED_TRACE(fault);
        output::full_shell::test::Directory copy;const auto original=RigidExport();
        for(const auto* name:{"scene.json","contact_scene_0000.rad","contact_scene_0001.rad"})
            std::filesystem::copy_file(original.parent_path()/name,copy.path/name);
        auto doc=output::array_json::Parse(output::ReadBounded(original,4u<<20),4u<<20);
        if(fault==0)doc["reference_rigid_body"]["primary"]["id"].SetUint64(18);
        if(fault==1)doc["reference_rigid_body"]["member_node_ids"][0].SetUint64(1);
        if(fault==2)doc["reference_rigid_body"]["centroid_node_order"][0].SetUint64(11);
        if(fault==3)doc["reference_rigid_body"]["converter_mass_tonne"].SetDouble(0.);
        if(fault==4)doc["scene"]["coupling"]["tied_secondary_removal"].SetUint(3);
        if(fault==5)doc["schema"].SetString("robo_dyna.native_contact_scene_export.v2",doc.GetAllocator());
        const auto path=copy.path/"declared-scene.json";output::WriteJson(path,doc);
        EXPECT_THROW(DeclaredSource::Read(path,output::Sha256(output::ReadBounded(path,4u<<20))),std::exception);
    }
    const auto file=RigidExport();EXPECT_NO_THROW(DeclaredSource::Read(file,output::Sha256(output::ReadBounded(file,4u<<20))));
}
}
