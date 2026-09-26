#include "PhysicalSource.h"
#include "MovingContactSource.h"
#include "ArchiveSource.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::cases::native_scene {
namespace {
modelio::native_scene::DeclaredSource RigidDeclared(){const auto* p=std::getenv("ROBO_DYNA_NATIVE_RIGID_SCENE_EXPORT");if(!p)throw std::runtime_error("Explicit rigid export required");return modelio::native_scene::DeclaredSource::Read(p,output::Sha256(output::ReadBounded(p,4u<<20)));}
}
TEST(RigidScenePhysicalSource, RealPartCoverageRetainsAllPhysicalMassAndSuppressesOnlyCoveredExecution) {
    const auto p=PhysicalSource::Prepare(RigidDeclared(),771);const auto& f=p.physical();
    ASSERT_EQ(f.domain()->node_count(),18u);ASSERT_TRUE(p.rigid().parts());
    ASSERT_EQ(p.rigid().groups().size(),1u);ASSERT_EQ(p.rigid().members().size(),9u);
    EXPECT_EQ(f.domain()->Find(19),SIZE_MAX);EXPECT_TRUE(p.cin().explicitly_empty());
    for(std::size_t i=0;i<18;++i) {
        const bool patch=f.domain()->nodes()[i].source_id>=10;
        EXPECT_EQ(p.rigid().FindMember(i)!=nullptr,patch);
        EXPECT_GT(f.coefficients()->nodes()[i].coefficients.mass,0);
        EXPECT_GT(f.coefficients()->nodes()[i].coefficients.isotropic_inertia,0);
        EXPECT_EQ(p.translation_fixed_bits()[i],patch?0:7);
    }
    for(const auto& row:f.execution()->parents()) {
        const bool covered=row.source.source_part_id==2;
        EXPECT_EQ(row.law,covered?tl::fea::ShellSectionLaw::RigidSkin:tl::fea::ShellSectionLaw::LayeredLaw44Nip3);
        EXPECT_EQ(row.material_points,covered?0u:3u);
    }
    const auto& primary=p.rigid().parts()->original_bodies()[0].primary;
    EXPECT_EQ(primary.mass,1e-20*1000);EXPECT_EQ(primary.inertia,((1e-20*1000)*.001)*.001);
    EXPECT_EQ(p.declared().data().material.plastic_hardening_n_mm2,1000.); // Raw source material is preserved.
}
TEST(RigidScenePhysicalSource, MovingSourceUsesAcceptedMassAndArchiveKeepsPhysicalSourceIdentity) {
    const auto p=PhysicalSource::Prepare(RigidDeclared(),771);const auto c=MovingContactSource::Prepare(p,{1,1,1});
    EXPECT_EQ(c.config().response_mass,native::ResponseMassPolicy::AcceptedOwnerCoefficients);
    EXPECT_EQ(c.source().selection.node_count,18u);EXPECT_EQ(c.source().primary_main_count,12u);
    EXPECT_EQ(c.forecast().native_model_nodes,19u);
    EXPECT_EQ(p.rigid().parts()->topology()->part_count(),1u);
    output::full_shell::test::Directory directory;
    EXPECT_NO_THROW(ArchiveSource::Write(p,directory.path));
    EXPECT_THROW(ContactSource::Prepare(p,{1,1,1}),std::exception);
    EXPECT_EQ(p.declared().data().definition_schema,"robo_dyna.native_contact_scene.v3");
}
}
