#include "PhysicalSource.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace crash::cases::native_scene {
namespace {
modelio::native_scene::DeclaredSource Declared() {
    const auto* raw=std::getenv("ROBO_DYNA_NATIVE_SCENE_EXPORT");
    if(!raw)throw std::runtime_error("Explicit capped source export required");
    const std::filesystem::path path(raw);
    return modelio::native_scene::DeclaredSource::Read(path,output::Sha256(output::ReadBounded(path,4u<<20)));
}
}
TEST(NativeScenePhysicalSource, CompleteRealShellMassMaterialAndEmptyConstraintBindings) {
    const auto source=PhysicalSource::Prepare(Declared(),771);const auto& physical=source.physical();
    ASSERT_TRUE(physical.prepared());ASSERT_EQ(physical.domain()->node_count(),18u);
    EXPECT_EQ(physical.shells()->qeph_count(),4u);EXPECT_EQ(physical.shells()->t3_count(),8u);EXPECT_EQ(physical.shells()->qbat_count(),0u);
    EXPECT_EQ(physical.coefficients()->scope().uncovered_nodes,0u);
    EXPECT_TRUE(source.rigid().explicitly_empty());EXPECT_EQ(source.rigid().parts(),nullptr);
    EXPECT_EQ(source.rigid().groups().size(),0u);EXPECT_EQ(source.rigid().members().size(),0u);
    EXPECT_TRUE(source.cin().explicitly_empty());EXPECT_EQ(source.cin().rows().count,0u);EXPECT_EQ(source.cin().rows().data,nullptr);
    EXPECT_TRUE(source.cin().domain()->SharesStorage(*physical.domain()));
    EXPECT_TRUE(source.rigid().coefficients()->Matches(*physical.coefficients()));
    for(const auto& row:physical.coefficients()->nodes()){EXPECT_GT(row.coefficients.mass,0);EXPECT_GT(row.coefficients.isotropic_inertia,0);}
    // Independent planar-area total; no observed native mass is copied into TL.
    const double expected=(1600.+100.*std::sqrt(1.+.02*.02))*1.*7.85e-9*1000.;
    EXPECT_NEAR(physical.coefficients()->totals().mass,expected,64*std::numeric_limits<double>::epsilon()*expected);
    for(const auto& row:physical.execution()->parents()) {
        EXPECT_EQ(row.law,tl::fea::ShellSectionLaw::LayeredLaw44Nip3);EXPECT_EQ(row.material_points,3u);
        EXPECT_EQ(physical.failure()->parent(row.source.family,row.source.family_index)->policy,tl::fea::ShellFailurePolicy::None);
        tl::fea::sections::PointParameters parameters;
        ASSERT_TRUE(physical.catalog()->Parameters(row.source.family,row.source.family_index,&parameters));
        EXPECT_EQ(output::Bits(parameters.plastic_hardening_pa),output::Bits(source.hardening().prepared_h_pa));
    }
    EXPECT_EQ(source.hardening().prepared_h_ulp_difference,1u);EXPECT_FALSE(source.hardening().exact_source_h_identity);
}
TEST(NativeScenePhysicalSource, OriginalIdsOrderingAndConstrainedMotionAreRetained) {
    const auto source=PhysicalSource::Prepare(Declared(),771);const auto& d=source.declared().data();
    const auto& physical=source.physical();
    for(std::size_t i=0;i<d.nodes.size();++i) {
        EXPECT_EQ(physical.domain()->nodes()[i].source_id,d.nodes[i].id);
        EXPECT_EQ(output::Bits(physical.domain()->nodes()[i].position.x),output::Bits(d.nodes[i].xyz_mm.x*.001));
    }
    for(auto node:d.wall_nodes){EXPECT_EQ(source.translation_fixed_bits()[node],7);EXPECT_EQ(source.rotation_fixed()[node],1);}
    for(auto node:d.patch_nodes){EXPECT_EQ(source.translation_fixed_bits()[node],0);EXPECT_EQ(source.rotation_fixed()[node],0);}
    EXPECT_EQ(source.startup().kind,tl::fea::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation);
    EXPECT_EQ(source.startup().uniform_velocity.x,1);EXPECT_EQ(source.startup().uniform_velocity.y,0);EXPECT_EQ(source.startup().uniform_velocity.z,-10);
    const auto parents=physical.execution()->parents();
    ASSERT_EQ(parents.size(),12u);EXPECT_EQ(parents[0].source.source_parent_id,9u);
    EXPECT_EQ(parents[4].source.source_parent_id,1u);
}
TEST(NativeScenePhysicalSource, WrongIdentityAndResourceScopeRejectWithoutChangingSource) {
    const auto declaration=Declared();const auto old=PhysicalSource::Prepare(declaration,771);
    EXPECT_THROW(PhysicalSource::Prepare(declaration,0),std::exception);
    EXPECT_THROW(PhysicalSource::Prepare(declaration,771,{17,1024}),std::exception);
    EXPECT_THROW(PhysicalSource::Prepare(declaration,771,{2048,11}),std::exception);
    EXPECT_TRUE(old.physical().prepared());EXPECT_EQ(old.physical().domain()->source_instance_id(),771u);
    const auto retry=PhysicalSource::Prepare(declaration,772);EXPECT_EQ(retry.physical().domain()->source_instance_id(),772u);
    EXPECT_EQ(old.declared().data().export_sha256,retry.declared().data().export_sha256);
}
}
