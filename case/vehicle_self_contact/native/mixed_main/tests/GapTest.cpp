#include "../Internal.h"
#include "lib_utest/qualification/radioss_type25_gap_source/NativeOracle.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::post_gapm::test {
TEST(PostGapmGapValues, SourceSelectedProfileRetainsNativeModesAndCapAssociation) {
    const auto p=detail::SourceGapProfile();
    EXPECT_EQ(p.property_type,1);EXPECT_EQ(p.input_thickness_mode,0);EXPECT_EQ(p.level,1);
    EXPECT_EQ(p.gap_mode,1);EXPECT_EQ(p.free_edge_gap,0);EXPECT_EQ(p.contact_thickness_update,0);
    EXPECT_EQ(output::Bits(p.scale),output::Bits(1.));
    EXPECT_EQ(output::Bits(p.maximum_secondary),output::Bits(n::native_constant::ep20*n::native_constant::ep10));
    EXPECT_EQ(output::Bits(p.maximum_main),output::Bits(p.maximum_secondary));
}
TEST(PostGapmGapValues, PrimaryOnlyPermutationPreservesPartnerAndOriginalNativeGapFields) {
    std::vector<s::Main> raw(3);
    const std::uint32_t nodes[3][4]{{0,1,2,3},{0,4,5,5},{3,2,1,0}};
    for(unsigned i=0;i<3;++i){std::copy_n(nodes[i],4,raw[i].nodes);raw[i].global_id=i+1;raw[i].source_id=10+i;}
    raw[0].segment_type=3;raw[2].segment_type=-1;
    s::MixedSidesSnapshot side;side.mains=raw.data();side.node_count=6;side.primary_count=2;side.main_count=3;side.shell_primary_count=1;
    std::vector<s::PrimaryCornerPermutation> corners(2);
    for(unsigned k=0;k<4;++k)corners[0].source_corner[k]=3-k;
    corners[1].source_corner[0]=1;corners[1].source_corner[1]=0;corners[1].source_corner[3]=2;
    const auto mains=detail::OrientedMains(side,corners);
    for(unsigned k=0;k<4;++k){EXPECT_EQ(mains[0].nodes[k],raw[0].nodes[3-k]);EXPECT_EQ(mains[2].nodes[k],raw[2].nodes[k]);}
    for(unsigned i=0;i<3;++i){EXPECT_EQ(mains[i].segment_type,raw[i].segment_type);EXPECT_EQ(mains[i].global_id,raw[i].global_id);}
    n::source_gaps::PhysicalShell shell;shell.source_element_id=10;shell.layout=n::ShellLayout::Quad4;
    std::copy_n(nodes[0],4,shell.nodes);shell.property_thickness=2.;
    n::source_gaps::Line beam{21,{4,5},0.,4.};
    const std::uint32_t nsv[]{0,1,2,3,4,5},msr[]{3,2,1,0,4,5};
    n::source_gaps::Input input;input.profile={1,0,1,1,0,0,1.,1e30,1e30};input.node_count=6;
    input.shells=&shell;input.shell_count=1;input.beams=&beam;input.beam_count=1;
    input.mains=mains.data();input.main_count=3;input.primary_count=2;
    input.secondary_nodes=nsv;input.secondary_count=6;input.main_nodes=msr;input.main_node_count=6;
    n::source_gaps::Forecast forecast;
    ASSERT_EQ(n::source_gaps::Preflight(input,{},forecast).status,n::source_gaps::Status::Ok);
    tl::util::HostArena arena;ASSERT_TRUE(arena.Initialize(forecast.scratch_bytes));
    std::vector<double> secondary(6),main_nodes(6);std::vector<n::source_gaps::MainGapFields> fields(3);
    const auto report=n::source_gaps::Build(input,{},arena.data(),arena.bytes(),{secondary.data(),6,main_nodes.data(),6,fields.data(),3});
    ASSERT_EQ(report.status,n::source_gaps::Status::Ok);
    const auto expected=gap_source_test::Oracle(input);
    for(unsigned i=0;i<6;++i){EXPECT_EQ(output::Bits(secondary[i]),output::Bits(expected.secondary[i]));EXPECT_EQ(output::Bits(main_nodes[i]),output::Bits(expected.main_nodes[i]));}
    for(unsigned i=0;i<3;++i){
        EXPECT_EQ(output::Bits(fields[i].maximum),output::Bits(expected.mains[i].maximum));
        for(unsigned k=0;k<4;++k)EXPECT_EQ(output::Bits(fields[i].corner[k]),output::Bits(expected.mains[i].corner[k]));
    }
    EXPECT_EQ(secondary[4],1.);EXPECT_EQ(fields[1].maximum,0.); // Native beam restore vs role-zero main clearing.
}
TEST(PostGapmGapValues, InvalidPrimaryPermutationRejectsWithoutTouchingSides) {
    s::Main main;main.source_id=17;main.nodes[0]=0;main.nodes[1]=1;main.nodes[2]=2;main.nodes[3]=3;
    s::MixedSidesSnapshot side;side.mains=&main;side.primary_count=1;side.main_count=1;
    std::vector<s::PrimaryCornerPermutation> corners(1);corners[0].source_corner[2]=4;
    EXPECT_THROW(detail::OrientedMains(side,corners),std::exception);
    EXPECT_EQ(main.source_id,17u);EXPECT_EQ(main.nodes[2],2u);
}
}
