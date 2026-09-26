#include "DeclaredSource.h"
#include "output/BoundedArrayJson.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::modelio::native_scene {
namespace {
std::filesystem::path File(){const auto* p=std::getenv("ROBO_DYNA_NATIVE_CIN_SCENE_EXPORT");if(!p)throw std::runtime_error("Explicit CIN declaration export required");return p;}
}
TEST(TiedSceneSource, ExactPhysicalRosterAndRawControlsAreRetained) {
    const auto file=File();const auto source=DeclaredSource::Read(file,output::Sha256(output::ReadBounded(file,4u<<20)));
    const auto& d=source.data();ASSERT_TRUE(d.tied_patch);EXPECT_FALSE(d.rigid_patch);
    EXPECT_EQ(d.nodes.size(),17u);EXPECT_EQ(d.wall.size(),8u);EXPECT_EQ(d.patch.size(),2u);
    EXPECT_EQ(d.patch[0].id,9u);EXPECT_EQ(d.patch[0].part,2u);EXPECT_EQ(d.patch[1].id,10u);EXPECT_EQ(d.patch[1].part,3u);
    EXPECT_EQ(d.definition_schema,"robo_dyna.native_contact_scene.v4");
    const auto& t=*d.tied_patch;
    EXPECT_EQ(t.master_nodes,(std::array<std::uint32_t,4>{9,10,12,11}));
    EXPECT_EQ(t.secondary_nodes,(std::array<std::uint32_t,4>{13,14,15,16}));
    EXPECT_EQ(t.spotflag,28);EXPECT_EQ(t.ignore,2);EXPECT_EQ(t.search,0);EXPECT_EQ(t.resolved_search,2);
    EXPECT_EQ(t.tied_removal,1);EXPECT_EQ(t.deletion,1);EXPECT_EQ(t.stiffness_scale,1.);EXPECT_EQ(t.viscosity,.05);
}
TEST(TiedSceneSource, RehashedExportCannotReplacePhysicalRolesGeometryOrControls) {
    const auto original=File();
    for(unsigned fault=0;fault<8;++fault) {
        SCOPED_TRACE(fault);
        output::full_shell::test::Directory copy;
        for(const auto* name:{"scene.json","contact_scene_0000.rad","contact_scene_0001.rad"})
            std::filesystem::copy_file(original.parent_path()/name,copy.path/name);
        auto doc=output::array_json::Parse(output::ReadBounded(original,4u<<20),4u<<20);
        if(fault==0)doc["mesh"]["nodes"][13]["xyz_mm"][0].SetDouble(-3.5);
        if(fault==1)doc["mesh"]["patch"][1]["part"].SetUint(2);
        if(fault==2)doc["reference_tied_interface"]["secondary_node_ids"][0].SetUint(10);
        if(fault==3)doc["scene"]["coupling"]["tied_secondary_removal"].SetUint(3);
        if(fault==4)doc["reference_tied_interface"]["master_source_element_id"].SetUint(10);
        if(fault==5)doc["scene"]["coupling"]["ignore"].SetBool(true);
        if(fault==6)doc["scene"]["coupling"]["dependent"]["z_mm"].SetDouble(2.1);
        if(fault==7)doc["mesh"]["patch_nodes"][0].SetUint(14);
        const auto path=copy.path/"declared-scene.json";output::WriteJson(path,doc);
        EXPECT_THROW(DeclaredSource::Read(path,output::Sha256(output::ReadBounded(path,4u<<20))),std::exception);
    }
}
}
