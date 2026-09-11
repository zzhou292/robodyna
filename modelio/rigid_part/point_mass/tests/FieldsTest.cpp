#include "../Fields.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <cstdio>

namespace crash::modelio::vehicle::rigid_part::point_mass::test {
namespace {
std::string Card(std::uint64_t eid,std::uint64_t nid,const char* mass) {
    char row[128];std::snprintf(row,sizeof(row),"%8llu%8llu%16s",
        static_cast<unsigned long long>(eid),static_cast<unsigned long long>(nid),mass);
    return row;
}
std::vector<tied_shell::SourceEvidence> Fixture() {
    tied_shell::SourceEvidence source;
    source.block={"yaris-coarse-v1l.key","*ELEMENT_MASS",{}, {},30,34};
    source.cards={{31,Card(101,7,".0011")},{32,Card(103,9,".0023")},
                  {33,""},{34,Card(107,7,"1e-9")}};
    return {source};
}
}
TEST(RigidPointMassFields, ExactOriginalFieldsSourceOrderAndRepeatedNodeRemainSeparate) {
    auto input=Fixture();const auto values=detail::Read(input,1000,{});
    ASSERT_EQ(values.size(),3u);
    EXPECT_EQ(values[0].value.source_element_id,101);
    EXPECT_EQ(values[2].value.source_element_id,107);
    EXPECT_EQ(values[0].value.source_node_id,values[2].value.source_node_id);
    EXPECT_EQ(values[2].value.source_card_index,3u);EXPECT_EQ(values[2].retained_source,0u);
    EXPECT_EQ(values[2].value.source_block_line,30u);
    EXPECT_EQ(output::Bits(values[0].value.supplied_mass_source),output::Bits(.0011));
    EXPECT_EQ(output::Bits(values[0].value.supplied_mass_kg),output::Bits(.0011*1000));
    input.clear();EXPECT_EQ(values[2].value.source_element_id,107);
}
TEST(RigidPointMassFields, OutsideSelectionTailStillRejectsAndCleanRetryHasNoMutation) {
    const auto input=Fixture();const auto old=detail::Read(input,1000,{});
    for(unsigned i=0;i<8;++i) {
        auto bad=input;auto& tail=bad[0].cards.back();
        if(i==0) tail.second=Card(101,7,".1");
        if(i==1) tail.second=Card(107,7,"-1");
        if(i==2) tail.second=Card(107,7,"nan");
        if(i==3) tail.second=Card(107,7,"1e308");
        if(i==4) tail.second=Card(107,7,"&DUMMY");
        if(i==5) tail.second=Card(107,7,".1")+"1";
        if(i==6) tail.first=32;
        if(i==7) bad[0].block.filename="wall.key";
        EXPECT_THROW(detail::Read(bad,1000,{}),std::runtime_error);
    }
    const auto retry=detail::Read(input,1000,{});
    ASSERT_EQ(retry.size(),old.size());
    for(std::size_t i=0;i<old.size();++i) {
        EXPECT_EQ(retry[i].value.source_element_id,old[i].value.source_element_id);
        EXPECT_EQ(output::Bits(retry[i].value.supplied_mass_kg),output::Bits(old[i].value.supplied_mass_kg));
    }
}
TEST(RigidPointMassFields, ExplicitCapsUnitsAndBlockOrderReject) {
    const auto source=Fixture();
    auto cap=Limits{};cap.records=3;
    EXPECT_THROW(detail::Read(source,1000,cap),std::runtime_error); // Blank card is retained in input budget.
    cap=Limits{};cap.host_bytes=1;
    EXPECT_THROW(detail::Read(source,1000,cap),std::runtime_error);
    EXPECT_THROW(detail::Read(source,0,{}),std::runtime_error);
    auto repeated=source;repeated.push_back(source[0]);
    EXPECT_THROW(detail::Read(repeated,1000,{}),std::runtime_error);
}
}
