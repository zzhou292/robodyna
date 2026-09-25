#include "MovingContactSource.h"
#include "output/full_shell/tests/TestSupport.h"
#include "lib_src/math/ScalarBits.h"
#include <cstdlib>
namespace crash::cases::native_scene {
namespace {
PhysicalSource MovingPhysical() {
    const char* p=std::getenv("ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT");
    output::Require(p,"Explicit v2 declared fixture required");
    return PhysicalSource::Prepare(modelio::native_scene::DeclaredSource::Read(p,
        output::Sha256(output::ReadBounded(p,4u<<20))),771);
}
void Same(native::StoredNormal a,native::StoredNormal b) {
    EXPECT_TRUE(tl::math::SameScalarBits(a.x,b.x));EXPECT_TRUE(tl::math::SameScalarBits(a.y,b.y));
    EXPECT_TRUE(tl::math::SameScalarBits(a.z,b.z));
}
}
TEST(NativeMovingContactSource, CompleteAllShellSourceKeepsGenuineStarterCacheAndPhysicalOrder) {
    const auto p=MovingPhysical();const auto c=MovingContactSource::Prepare(p,{81,82,83});
    const auto& source=c.source();const auto& s=source.selection;const auto& t=source.starter;
    const auto& d=p.declared().data();
    ASSERT_EQ(source.primary_main_count,d.wall.size()+d.patch.size());ASSERT_EQ(source.primary_main_count,12u);
    ASSERT_EQ(s.main_count,24u);ASSERT_EQ(s.secondary_count,18u);ASSERT_EQ(s.normal_count,36u);
    EXPECT_EQ(t.source_generation,83u);EXPECT_EQ(s.generation,83u);
    EXPECT_EQ(source.activation.free_roster,native::normal_activation::FreeRosterPolicy::FreshComplete);
    for(std::size_t i=0;i<source.primary_main_count;++i) {
        const auto& parent=i<d.wall.size()?d.wall[i]:d.patch[i-d.wall.size()];
        EXPECT_EQ(source.primary_parent_ids[i],parent.id);EXPECT_EQ(t.mains[i].source_id,parent.id);
        for(unsigned j=0;j<4;++j)EXPECT_EQ(s.mains[i].nodes[j],parent.nodes[j]);
    }
    for(std::size_t i=0;i<s.node_count;++i) {
        EXPECT_EQ(s.nodes[s.secondary[i].node].source_id,i+1);
        EXPECT_EQ(s.secondary[i].coefficient,d.material.young_n_mm2*d.thickness_mm);
        EXPECT_EQ(s.secondary[i].gap,.5*d.thickness_mm); // Every shell is a selected main now.
        EXPECT_EQ(s.secondary[i].initial_contact_flag,0);
    }
    for(std::size_t i=0;i<s.main_count;++i)for(unsigned j=0;j<4;++j)
        Same(s.mains[i].normal_slot[j],t.starter.face_normals[4*i+j]);
    std::size_t boundaries=0;
    for(std::size_t i=0;i<s.normal_count;++i) {
        EXPECT_LE(s.normals[i].boundary,1);EXPECT_EQ(s.normals[i].boundary,t.starter.references[i].boundary);
        boundaries+=s.normals[i].boundary!=0;
        for(unsigned j=0;j<2;++j)Same(s.normals[i].bisector[j],t.starter.references[i].bisector[j]);
    }
    EXPECT_GT(boundaries,0u);EXPECT_EQ(s.removed_main_by_secondary.entry_count,0u);
    const auto retained=c;EXPECT_EQ(retained.source().starter.mains,t.mains);
    EXPECT_TRUE(retained.physical_source().physical().Matches(p.physical()));
}
TEST(NativeMovingContactSource, FactoriesRejectOtherSurfaceBeforeAllocationAndCapsAreComplete) {
    const auto p=MovingPhysical();const auto good=MovingContactSource::Prepare(p,{1,2,3});
    EXPECT_THROW(ContactSource::Prepare(p,{1,2,3}),std::exception);
    const char* fixed_path=std::getenv("ROBO_DYNA_NATIVE_SCENE_EXPORT");ASSERT_NE(fixed_path,nullptr);
    const auto fixed=PhysicalSource::Prepare(modelio::native_scene::DeclaredSource::Read(fixed_path,
        output::Sha256(output::ReadBounded(fixed_path,4u<<20))),771);
    EXPECT_THROW(MovingContactSource::Prepare(fixed,{1,2,3}),std::exception);
    ContactLimits limits;limits.host_bytes=good.forecast().peak_bytes;limits.scratch_bytes=good.forecast().startup_scratch_bytes;
    EXPECT_NO_THROW(MovingContactSource::Prepare(p,{1,2,3},limits));
    --limits.host_bytes;EXPECT_THROW(MovingContactSource::Prepare(p,{1,2,3},limits),std::exception);
    limits={};limits.physical_shells=11;EXPECT_THROW(MovingContactSource::Prepare(p,{1,2,3},limits),std::exception);
    limits={};limits.preprocessing=native::search_startup::Initialization::SerialNative;
    const auto serial=MovingContactSource::Prepare(p,{1,2,3},limits);
    EXPECT_EQ(output::Bits(serial.source().margin),output::Bits(good.source().margin));
    EXPECT_EQ(good.source().selection.generation,3u);
}
TEST(NativeMovingContactSource, CopyRetainsAllBorrowedArraysAfterOriginalSourceHandlesEnd) {
    const auto retained=[] {return MovingContactSource::Prepare(MovingPhysical(),{1,2,3});}();
    EXPECT_EQ(retained.source().starter.main_count,24u);
    EXPECT_EQ(retained.source().starter.mains[23].source_id,retained.source().primary_parent_ids[11]);
    EXPECT_EQ(retained.source().selection.mains[23].segment_type,-12);
    EXPECT_EQ(retained.physical_source().declared().data().contact_surface,modelio::native_scene::DeclaredContactSurface::AllShells);
}
}
