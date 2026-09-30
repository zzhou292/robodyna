#include "ContactSource.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::cases::native_scene {
TEST(NativeMovingSceneAdmission, FixedFactoryCannotSilentlyConsumeMovingPrimaryDeclaration) {
    const auto* file=std::getenv("ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT");ASSERT_NE(file,nullptr);
    const auto declared=modelio::native_scene::DeclaredSource::Read(file,output::Sha256(output::ReadBounded(file,4u<<20)));
    ASSERT_EQ(declared.data().contact_surface,modelio::native_scene::DeclaredContactSurface::AllShells);
    const auto physical=PhysicalSource::Prepare(declared,817);
    EXPECT_EQ(physical.physical().shells()->qeph_count(),4u);
    EXPECT_EQ(physical.physical().shells()->t3_count(),8u);
    EXPECT_THROW(ContactSource::Prepare(physical,{1,2,3}),std::exception);
    EXPECT_EQ(physical.declared().data().contact_surface,modelio::native_scene::DeclaredContactSurface::AllShells);
    EXPECT_EQ(physical.physical().domain()->node_count(),18u);
}
}
