#include "DeclaredSource.h"
#include "output/ArtifactIO.h"
#include "output/BoundedArrayJson.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::modelio::native_scene {
namespace {
std::filesystem::path Original(){const auto* value=std::getenv("ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT");
    if(!value)throw std::runtime_error("Explicit moving source fixture required");return value;}
}
TEST(MovingSceneSource, ExplicitVersionedSurfaceSurvivesSourceReadbackWithoutChangingPhysics) {
    const auto file=Original();const auto source=DeclaredSource::Read(file,output::Sha256(output::ReadBounded(file,4u<<20)));
    const auto& data=source.data();EXPECT_EQ(data.contact_surface,DeclaredContactSurface::AllShells);
    EXPECT_EQ(data.nodes.size(),18u);EXPECT_EQ(data.wall.size(),8u);EXPECT_EQ(data.patch.size(),4u);
    EXPECT_EQ(data.velocity_mm_s.z,-10000.);EXPECT_EQ(data.thickness_mm,1.);EXPECT_EQ(data.step_cap_s,3e-7);
    const auto copied=source;EXPECT_EQ(copied.data().contact_surface,DeclaredContactSurface::AllShells);
}
TEST(MovingSceneSource, ExportScopeAndSchemaMustMatchTheOriginalHashedDeclaration) {
    for(unsigned fault=0;fault<3;++fault) {
        output::full_shell::test::Directory copy;const auto original=Original();
        for(const auto* name:{"scene.json","contact_scene_0000.rad","contact_scene_0001.rad"})
            std::filesystem::copy_file(original.parent_path()/name,copy.path/name);
        auto doc=output::array_json::Parse(output::ReadBounded(original,4u<<20),4u<<20);
        if(fault==0)doc["scene"]["contact_surface"].SetString("fixed_wall",doc.GetAllocator());
        if(fault==1)doc["schema"].SetString("robo_dyna.native_contact_scene_export.v1",doc.GetAllocator());
        if(fault==2)doc["scene"].RemoveMember("contact_surface");
        const auto file=copy.path/"declared-scene.json";output::WriteJson(file,doc);
        EXPECT_THROW(DeclaredSource::Read(file,output::Sha256(output::ReadBounded(file,4u<<20))),std::exception);
    }
}
} // namespace crash::modelio::native_scene
