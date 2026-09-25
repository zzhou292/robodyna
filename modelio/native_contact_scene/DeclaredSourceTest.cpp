#include "DeclaredSource.h"
#include "output/ArtifactIO.h"
#include "output/BoundedArrayJson.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <unistd.h>
namespace crash::modelio::native_scene {
namespace {
std::filesystem::path Original() {
    const auto* path=std::getenv("ROBO_DYNA_NATIVE_SCENE_EXPORT");
    if(!path)throw std::runtime_error("Explicit scene export fixture required");return path;
}
struct Copy {
    std::filesystem::path root,manifest;
    Copy() {
        const auto original=Original();
        auto name=std::filesystem::temp_directory_path()/"native-scene-source-XXXXXX";
        std::string text=name.string();char* created=mkdtemp(text.data());
        if(!created)throw std::runtime_error("Cannot create private reader fixture");root=created;
        manifest=root/original.filename();std::filesystem::copy_file(original,manifest);
        for(const auto* file:{"scene.json","contact_scene_0000.rad","contact_scene_0001.rad"})
            std::filesystem::copy_file(original.parent_path()/file,root/file);
    }
    ~Copy(){std::error_code ignored;std::filesystem::remove_all(root,ignored);}
    std::string Hash() const{return output::Sha256(output::ReadBounded(manifest,4u<<20));}
    output::Document Document() const{return output::array_json::Parse(output::ReadBounded(manifest,4u<<20),4u<<20);}
    void Replace(const output::Document& document) {
        // Only this private test copy is replaceable; original source is never modified.
        std::filesystem::remove(manifest);output::WriteJson(manifest,document);
    }
};
}
TEST(NativeSceneSource, ExactExportRetainsOriginalNodeParentUnitsAndControls) {
    Copy copy;const auto source=DeclaredSource::Read(copy.manifest,copy.Hash());const auto& d=source.data();
    ASSERT_EQ(d.nodes.size(),18u);ASSERT_EQ(d.wall.size(),8u);ASSERT_EQ(d.patch.size(),4u);
    EXPECT_EQ(d.wall_nodes.size(),9u);EXPECT_EQ(d.patch_nodes.size(),9u);
    EXPECT_EQ(d.nodes[0].id,1u);EXPECT_EQ(d.nodes[0].xyz_mm.x,-20);
    EXPECT_EQ(d.patch[0].id,9u);EXPECT_EQ(d.patch[0].nodes[0],9u);
    for(const auto& wall:d.wall)EXPECT_EQ(wall.nodes[3],wall.nodes[2]);
    EXPECT_EQ(d.material.plastic_hardening_n_mm2,1000);EXPECT_EQ(d.velocity_mm_s.z,-10000);
    ASSERT_TRUE(d.step_cap_s);EXPECT_EQ(*d.step_cap_s,3e-7);EXPECT_EQ(d.end_time_s,.0003);
    EXPECT_EQ(d.definition_sha256,output::Sha256(d.definition_bytes));
}
TEST(NativeSceneSource, HashAndCompleteReferenceFileInventoryAreRequired) {
    Copy copy;EXPECT_THROW(DeclaredSource::Read(copy.manifest,std::string(64,'0')),std::exception);
    const auto expected=copy.Hash();
    {std::ofstream stream(copy.root/"contact_scene_0001.rad",std::ios::app);stream<<"# changed\n";}
    EXPECT_THROW(DeclaredSource::Read(copy.manifest,expected),std::exception);
    Copy fresh;auto document=fresh.Document();document["files"][0]["path"].SetString("../scene.json",document.GetAllocator());fresh.Replace(document);
    EXPECT_THROW(DeclaredSource::Read(fresh.manifest,fresh.Hash()),std::exception);
}
TEST(NativeSceneSource, MalformedRolesConnectivityMaterialAndCapacityRejectBeforePublication) {
    for(unsigned fault=0;fault<5;++fault) {
        Copy copy;auto document=copy.Document();
        if(fault==0)document["mesh"]["nodes"][1]["id"].SetUint64(1);
        if(fault==1)document["mesh"]["wall"][0]["nodes"][0].SetUint64(10);
        if(fault==2)document["mesh"]["patch_nodes"][0].SetUint64(1);
        if(fault==3)document["scene"]["material"]["plastic_hardening_n_mm2"].SetDouble(999);
        if(fault==4)document["scene"]["velocity_mm_s"][2].SetDouble(-9000);
        copy.Replace(document);EXPECT_THROW(DeclaredSource::Read(copy.manifest,copy.Hash()),std::exception);
    }
    Copy copy;ReadLimits limits;limits.nodes=17;
    EXPECT_THROW(DeclaredSource::Read(copy.manifest,copy.Hash(),limits),std::exception);
    limits={};limits.parents=11;EXPECT_THROW(DeclaredSource::Read(copy.manifest,copy.Hash(),limits),std::exception);
    EXPECT_NO_THROW(DeclaredSource::Read(copy.manifest,copy.Hash()));
}
}
