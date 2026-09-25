#include "MovingContactSource.h"
#include "output/full_shell/tests/TestSupport.h"
#include <cstdlib>
#include <sstream>
namespace crash::cases::native_scene {
namespace {
output::Document Record(const std::string& stage) {
    const auto raw=output::ReadBounded(ROBO_DYNA_MOVING_SOURCE_OBSERVATION,64u<<10);
    output::Require(output::Sha256(raw)==ROBO_DYNA_MOVING_SOURCE_OBSERVATION_SHA,"Independent source fixture hash differs");
    std::istringstream lines(raw);std::string line;
    while(std::getline(lines,line)) {
        auto record=output::array_json::Parse(line,64u<<10);
        if(record["stage"].GetString()==stage)return record;
    }
    throw std::runtime_error("Missing independent native source record: "+stage);
}
}
TEST(NativeMovingContactSource, ImmutableTopologyCoefficientsGapsAndIdsMatchExternalEngine) {
    const char* path=std::getenv("ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT");ASSERT_NE(path,nullptr);
    const auto physical=PhysicalSource::Prepare(modelio::native_scene::DeclaredSource::Read(path,
        output::Sha256(output::ReadBounded(path,4u<<20))),771);
    const auto contact=MovingContactSource::Prepare(physical,{1,2,3});const auto& source=contact.source();const auto& s=source.selection;
    const auto main=Record("main"),classification=Record("classification"),boundary=Record("boundary");
    const auto& ids=main["observation"]["arrays"]["ITAB"];const auto& expected=classification["observation"]["arrays"];
    ASSERT_EQ(ids.Size(),s.node_count);ASSERT_EQ(expected["STF"].Size(),s.main_count);
    ASSERT_EQ(expected["STFN"].Size(),s.secondary_count);
    // IRECT/NSV are native internal-node indices. Translate through the actual
    // observed ITAB map before comparing source IDs; never assume dense ID order.
    for(std::size_t i=0;i<s.main_count;++i) {
        const auto& m=s.mains[i];const auto ordinal=rapidjson::SizeType(i);
        EXPECT_EQ(m.segment_type,expected["MSEGTYP"][ordinal].GetInt());
        EXPECT_EQ(m.global_id,boundary["observation"]["arrays"]["MSEGLO"][ordinal].GetInt());
        EXPECT_EQ(output::Bits(m.coefficient),output::Bits(expected["STF"][ordinal].GetDouble()));
        EXPECT_EQ(output::Bits(m.maximum_gap),output::Bits(expected["GAP_M"][ordinal].GetDouble()));
        for(unsigned j=0;j<4;++j) {
            const auto k=rapidjson::SizeType(4*i+j);const auto native_node=expected["IRECT"][k].GetUint();
            ASSERT_GT(native_node,0u);ASSERT_LE(native_node,ids.Size());
            EXPECT_EQ(s.nodes[m.nodes[j]].source_id,ids[native_node-1].GetUint64());
            EXPECT_EQ(m.normal_reference[j],expected["ADMSR"][k].GetInt());EXPECT_EQ(m.neighbors[j],expected["MVOISIN"][k].GetInt());
            EXPECT_EQ(output::Bits(m.gap[j]),output::Bits(expected["GAPN_M"][k].GetDouble()));
        }
    }
    for(std::size_t i=0;i<s.secondary_count;++i) {
        const auto k=rapidjson::SizeType(i);const auto native_node=expected["NSV"][k].GetUint();
        ASSERT_GT(native_node,0u);ASSERT_LE(native_node,ids.Size());
        EXPECT_EQ(s.nodes[s.secondary[i].node].source_id,ids[native_node-1].GetUint64());
        EXPECT_EQ(output::Bits(s.secondary[i].coefficient),output::Bits(expected["STFN"][k].GetDouble()));
        EXPECT_EQ(output::Bits(s.secondary[i].gap),output::Bits(expected["GAP_S"][k].GetDouble()));
    }
    const auto& scalars=boundary["observation"]["scalars"];
    EXPECT_EQ(output::Bits(source.margin),output::Bits(scalars["MARGE"].GetDouble()));
    EXPECT_EQ(output::Bits(source.drad),output::Bits(scalars["DRAD"].GetDouble()));
    EXPECT_EQ(output::Bits(source.gap_load),output::Bits(scalars["DGAPLOAD"].GetDouble()));
    EXPECT_GT(classification["observation"]["clock"]["NCYCLE"].GetUint(),0u);
    // Dynamic NOD_NORMAL / LBOUND / bisectors at that cycle are deliberately
    // not Starter expectations. The independent source oracle checks that phase.
}
}
