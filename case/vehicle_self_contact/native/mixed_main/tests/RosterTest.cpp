#include "../Internal.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_self_contact::native::post_gapm::test {
namespace {
coated::Inputs Nodes() {
    coated::Inputs in;
    for(unsigned i=0;i<6;++i)in.nodes.push_back({11+3*i,i,{double(i),0,0}});
    return in;
}
std::vector<initial_surfaces::Face> Faces() {
    initial_surfaces::Face a,b,c;a.nodes={3,1,0,0};b.nodes={4,3,1,5};c.nodes=a.nodes;
    a.source.element_id=1;b.source.element_id=2;c.source.element_id=99;
    return {a,c,b};
}
}
TEST(PostGapmSourceRosters, OrdinaryPropertyDispatchRejectsIntegrationAndSpecialMaterial) {
    modelio::assembly::Material material;material.source.keyword="*MAT_ELASTIC";
    modelio::assembly::Section section;section.source.keyword="*SECTION_SHELL";section.cards.resize(2);
    section.cards[0].values.resize(8);
    EXPECT_TRUE(detail::OrdinaryNativeProperty(material,section));
    section.cards[0].values[5]=7.;EXPECT_FALSE(detail::OrdinaryNativeProperty(material,section));
    section.cards[0].values[5]=0.;EXPECT_TRUE(detail::OrdinaryNativeProperty(material,section));
    material.source.keyword="*MAT_FABRIC";EXPECT_FALSE(detail::OrdinaryNativeProperty(material,section));
}
TEST(PostGapmSourceRosters, OriginalS1FirstEncounterAndSortedNsvRetainRepeatedAndMultiOriginWords) {
    auto input=Nodes();auto faces=Faces();std::vector<std::uint32_t> nsv,msr;
    detail::SourceNodeRosters(input,faces,nsv,msr);
    EXPECT_EQ(nsv,(std::vector<std::uint32_t>{0,1,3,4,5}));
    EXPECT_EQ(msr,(std::vector<std::uint32_t>{3,1,0,4,5}));
    std::swap(faces[0],faces[1]);std::vector<std::uint32_t> nsv2,msr2;
    detail::SourceNodeRosters(input,faces,nsv2,msr2);EXPECT_EQ(nsv2,nsv);EXPECT_EQ(msr2,msr);
}
TEST(PostGapmSourceRosters, InvalidNodeOrItabRangePreservesBothOutputs) {
    for(unsigned variant=0;variant<3;++variant) {
        auto in=Nodes();auto faces=Faces();std::vector<std::uint32_t> nsv{71},msr{19};
        if(variant==0)faces.back().nodes[3]=6;
        if(variant==1)in.nodes.back().source_id=std::uint64_t{1}<<32;
        if(variant==2)in.nodes[2].source_id=in.nodes[1].source_id;
        EXPECT_THROW(detail::SourceNodeRosters(in,faces,nsv,msr),std::exception);
        EXPECT_EQ(nsv,(std::vector<std::uint32_t>{71}));EXPECT_EQ(msr,(std::vector<std::uint32_t>{19}));
    }
}
}
