#include "ContactSource.h"
#include "output/full_shell/tests/TestSupport.h"
#include <algorithm>
#include <cstdlib>
#ifdef ROBO_DYNA_NATIVE_SCENE_EXPECTED_FIELDS
#include "ObservedScene.h" // Qualification-only independent native observations.
#endif
namespace crash::cases::native_scene {
namespace ft=output::full_shell::test;
namespace n=tlfea::contact::radioss_type25;
namespace {
std::filesystem::path Export() {
    const char* p=std::getenv("ROBO_DYNA_NATIVE_SCENE_EXPORT");if(!p)throw std::runtime_error("Explicit source fixture required");return p;
}
PhysicalSource Source(const std::filesystem::path& file) {
    return PhysicalSource::Prepare(modelio::native_scene::DeclaredSource::Read(file,output::Sha256(output::ReadBounded(file,4u<<20))),771);
}
}
TEST(NativeSceneContactSource, CompletePhysicalContributionsAndProducedSourceScopes) {
    const auto physical=Source(Export());const auto source=ContactSource::Prepare(physical,{81,82,83});
    const auto& s=source.source();const auto& l=s.selection;const auto& d=physical.declared().data();
    ASSERT_EQ(l.node_count,18u);ASSERT_EQ(l.secondary_count,18u);ASSERT_EQ(l.main_count,16u);ASSERT_EQ(s.primary_main_count,8u);
    EXPECT_EQ(s.source_id,81u);EXPECT_EQ(s.topology_generation,82u);EXPECT_EQ(l.generation,83u);
    EXPECT_GT(s.margin,0);EXPECT_EQ(s.gap_load,0);EXPECT_EQ(s.drad,0);EXPECT_EQ(s.force_packet_size,128u);EXPECT_EQ(s.native_workers,1);
    EXPECT_EQ(source.config().units.length_m,.001);EXPECT_EQ(source.config().lifecycle.optcd_response_precision,0);
    EXPECT_EQ(source.preprocessing(),n::search_startup::Initialization::InvariantNoExpansion);
    EXPECT_EQ(source.forecast().native_model_nodes,18u);
    for(std::size_t i=0;i<l.secondary_count;++i) {
        const auto& row=l.secondary[i];EXPECT_EQ(l.nodes[row.node].source_id,i+1);
        // Independent uniform-material expectation; noncontact patch shells must contribute too.
        EXPECT_EQ(row.coefficient,d.material.young_n_mm2*d.thickness_mm);
        EXPECT_EQ(row.gap,i<9?.5*d.thickness_mm:0.);
        EXPECT_EQ(row.initial_contact_flag,0);
        EXPECT_EQ(l.nodes[row.node].constraint,i<9?7:0);EXPECT_EQ(l.nodes[row.node].skew,i<9?1:0);
    }
    for(std::size_t i=0;i<l.main_count;++i) {
        EXPECT_EQ(l.mains[i].global_id,i+1);EXPECT_EQ(l.mains[i].coefficient,d.material.young_n_mm2*d.thickness_mm);
        EXPECT_EQ(l.mains[i].maximum_gap,.5*d.thickness_mm);
        for(unsigned j=0;j<4;++j)EXPECT_EQ(l.mains[i].gap[j],l.mains[i].maximum_gap);
    }
    EXPECT_EQ(l.normal_to_main.offset_count,l.normal_count+1);
    EXPECT_EQ(l.removed_main_by_secondary.offset_count,l.secondary_count+1);
    EXPECT_EQ(l.removed_main_by_secondary.entry_count,0u);EXPECT_EQ(l.removed_main_by_secondary.entries,nullptr);
    const auto copied=source;EXPECT_EQ(copied.source().selection.mains,l.mains);
    EXPECT_TRUE(copied.physical_source().physical().Matches(physical.physical()));
}
TEST(NativeSceneContactSource, SecondaryOrderComesFromIdsNotDenseStorageOrCapturedOrdinals) {
    ft::Directory copy;const auto original=Export();
    for(const auto* name:{"scene.json","contact_scene_0000.rad","contact_scene_0001.rad"})
        std::filesystem::copy_file(original.parent_path()/name,copy.path/name);
    auto document=output::array_json::Parse(output::ReadBounded(original,4u<<20),4u<<20);
    auto nodes=document["mesh"]["nodes"].GetArray();std::reverse(nodes.Begin(),nodes.End());
    output::WriteJson(copy.path/"declared-scene.json",document);
    const auto p=Source(copy.path/"declared-scene.json");const auto c=ContactSource::Prepare(p,{1,2,3});
    const auto& s=c.source().selection;
    for(std::size_t i=0;i<s.secondary_count;++i){EXPECT_EQ(s.secondary[i].node,17-i);EXPECT_EQ(s.nodes[s.secondary[i].node].source_id,i+1);}
    EXPECT_EQ(s.nodes[0].source_id,18u);EXPECT_EQ(s.nodes[17].source_id,1u);
}
TEST(NativeSceneContactSource, ExactIncrementalCapAndRejectedAttemptPreservePublishedSource) {
    const auto p=Source(Export());const auto good=ContactSource::Prepare(p,{1,2,3});
    ContactLimits limits;limits.host_bytes=good.forecast().peak_bytes;limits.scratch_bytes=good.forecast().startup_scratch_bytes;
    const auto exact=ContactSource::Prepare(p,{1,2,3},limits);EXPECT_EQ(exact.forecast().peak_bytes,good.forecast().peak_bytes);
    --limits.host_bytes;EXPECT_THROW(ContactSource::Prepare(p,{1,2,3},limits),std::exception);
    limits={};limits.nodes=17;EXPECT_THROW(ContactSource::Prepare(p,{1,2,3},limits),std::exception);
    limits={};limits.preprocessing=n::search_startup::Initialization::Unspecified;
    EXPECT_THROW(ContactSource::Prepare(p,{1,2,3},limits),std::exception);
    EXPECT_THROW(ContactSource::Prepare(p,{0,2,3}),std::exception);
    limits={};limits.preprocessing=n::search_startup::Initialization::SerialNative;
    const auto serial=ContactSource::Prepare(p,{1,2,3},limits);
    EXPECT_EQ(serial.source().margin,good.source().margin);
    EXPECT_EQ(good.source().selection.generation,3u);EXPECT_GT(good.source().margin,0);
}
#ifdef ROBO_DYNA_NATIVE_SCENE_EXPECTED_FIELDS
TEST(NativeSceneContactSource, ImmutableFieldsMatchIndependentNativeSceneObservation) {
    namespace expected=native_scene_fixture;
    const auto c=ContactSource::Prepare(Source(Export()),{1,1,1});const auto& source=c.source();const auto& s=source.selection;
    ASSERT_EQ(s.node_count,18u);ASSERT_EQ(s.secondary_count,18u);ASSERT_EQ(s.main_count,16u);ASSERT_EQ(s.normal_count,18u);
    for(unsigned i=0;i<18;++i) {
        EXPECT_EQ(s.nodes[i].source_id,expected::NodeIds[i]);EXPECT_EQ(s.nodes[i].constraint,expected::ConstraintCodes[i]);EXPECT_EQ(s.nodes[i].skew,expected::SkewCodes[i]);
        EXPECT_EQ(s.secondary[i].node,expected::SecondaryNodes[i]);EXPECT_EQ(output::Bits(s.secondary[i].coefficient),output::Bits(expected::SecondaryK[i]));
        EXPECT_EQ(output::Bits(s.secondary[i].gap),output::Bits(expected::SecondaryGaps[i]));EXPECT_EQ(s.secondary[i].initial_contact_flag,expected::InitialContact[i]);

    }
    for(unsigned i=0;i<16;++i){const auto& main=s.mains[i];
        EXPECT_EQ(main.global_id,expected::MainGlobalIds[i]);EXPECT_EQ(main.segment_type,expected::MainRoles[i]);
        EXPECT_EQ(output::Bits(main.coefficient),output::Bits(expected::MainK[i]));EXPECT_EQ(output::Bits(main.maximum_gap),output::Bits(expected::MainMaximumGaps[i]));
        for(unsigned j=0;j<4;++j){const auto k=4*i+j;
            EXPECT_EQ(main.nodes[j],expected::MainNodes[k]);EXPECT_EQ(main.normal_reference[j],expected::MainNormalReferences[k]);
            EXPECT_EQ(main.neighbors[j],expected::Neighbors[k]);EXPECT_EQ(output::Bits(main.gap[j]),output::Bits(expected::MainGaps[k]));

        }
    }
    for(unsigned i=0;i<19;++i)EXPECT_EQ(s.normal_to_main.offsets[i],expected::NormalOffsets[i]);
    ASSERT_EQ(s.normal_to_main.entry_count,std::size(expected::NormalEntries));
    for(std::size_t i=0;i<s.normal_to_main.entry_count;++i)EXPECT_EQ(s.normal_to_main.entries[i],expected::NormalEntries[i]);
    EXPECT_EQ(output::Bits(source.margin),output::Bits(expected::NativeControls[0]));
    EXPECT_EQ(output::Bits(source.drad),output::Bits(expected::NativeControls[1]));EXPECT_EQ(output::Bits(source.gap_load),output::Bits(expected::NativeControls[2]));
    for(unsigned i=0;i<19;++i)EXPECT_EQ(s.removed_main_by_secondary.offsets[i],0u);
    EXPECT_EQ(s.removed_main_by_secondary.entry_count,0u);
}
#endif
}
