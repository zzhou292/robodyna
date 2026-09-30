#include "MidlayerFixture.h"
#include <gtest/gtest.h>

namespace crash::modelio::vehicle::test {
TEST(VehicleMidlayerMapping, AvailableFormulationIndicesAreIndependentOfSourceTopology) {
    std::vector<PartDisposition> source(3,MidlayerFields());
    source[0].part_id=1;
    source[2].part_id=3;
    std::vector<SectionPartResolution> parts(3);
    parts[0].status=SectionDisposition::Existing;
    parts[1].status=SectionDisposition::Midlayer;
    parts[1].failure_strain=2.5;
    const std::vector<SectionParentResolution> parents{
        {901,8,0,100,SourceShellTopology::Q4}, {801,9,1,101,SourceShellTopology::Q4},
        {701,10,1,17,SourceShellTopology::T3}, {601,11,2,102,SourceShellTopology::Q4},
        {501,12,0,103,SourceShellTopology::Q4}, {401,13,0,18,SourceShellTopology::T3}};
    auto visible=resolution::MapNativeParents(parents,parts,source);
    EXPECT_EQ(visible.counts.qeph,2);
    EXPECT_EQ(visible.counts.qbat,1);
    EXPECT_EQ(visible.counts.t3,2);
    const std::size_t indices[]{0,0,0,SIZE_MAX,1,1};
    for(unsigned i=0;i<6;++i) {
        EXPECT_EQ(visible.mapping[i].family_index,indices[i]);
        if(i!=3) EXPECT_EQ(visible.parents[i].source.source_parent_id,parents[i].source_parent_id);
    }
    EXPECT_EQ(visible.mapping[1].family,tl::fea::ShellBindingFamily::Qbat);
    EXPECT_EQ(visible.mapping[3].family,tl::fea::ShellBindingFamily::None);
    EXPECT_EQ(visible.parents[1].policy,tl::fea::ShellFailurePolicy::ConstantAllPoints);
    EXPECT_EQ(visible.parents[2].constant.failure_strain,2.5);
    auto bad=parents;
    bad.back().part_index=99;
    EXPECT_THROW(visible=resolution::MapNativeParents(bad,parts,source),std::runtime_error);
    EXPECT_EQ(visible.parents.back().source.source_parent_id,401);
    parts[1].status=static_cast<SectionDisposition>(99);
    EXPECT_THROW(resolution::MapNativeParents(parents,parts,source),std::runtime_error);
    parts[1].status=SectionDisposition::Midlayer;
    visible=resolution::MapNativeParents(parents,parts,source);
    EXPECT_EQ(visible.mapping.back().family_index,1);
    EXPECT_EQ(parents[4].topology_index,103);
}
} // namespace crash::modelio::vehicle::test
