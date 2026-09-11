#include "modelio/type13/SourceType13.h"
#include "output/ArtifactIO.h"
#include "lib_utest/qualification/type13/source_fixture/YarisType13SourceFixture.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <functional>

namespace crash::modelio::type13 {
namespace {
const ArtifactIdentity Identity{5150841,"c15fc2096317ac0206397ac50776f8456ddd23495e0c65aeee98e093ebd0b1b1"};
std::filesystem::path FixturePath() {
    const char* path=std::getenv("ROBO_DYNA_TYPE13_DECLARATION");
    output::Require(path&&*path,"Explicit original TYPE13 declaration fixture is required");return path;
}
std::string Alter(const std::string& bytes,const std::function<void(output::Document&)>& change) {
    output::Document document;
    document.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(bytes.data(),bytes.size());
    output::Require(!document.HasParseError(),"Malformed original test fixture");change(document);
    rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    output::Require(document.Accept(writer),"Corruption fixture serialization failed");return {buffer.GetString(),buffer.GetSize()};
}
}
TEST(Type13ActualSource,CompleteOriginalDeclarationLoadsAndInitializesAll4442) {
    const auto input=SourceType13::Read(FixturePath(),Identity);const auto& data=input.data();
    ASSERT_EQ(data.nodes.size(),7494);ASSERT_EQ(data.beams.size(),4442);
    EXPECT_GT(data.owned_payload_bytes,Identity.bytes);EXPECT_LE(data.owned_payload_bytes,data.startup_budget_bytes);
    EXPECT_EQ(data.source_property.young,50000);EXPECT_EQ(data.source_property.tangent,5000);
    EXPECT_EQ(data.source_property.outer1,5);EXPECT_EQ(data.source_property.inner1,0);
    EXPECT_EQ(data.source_property.failure_deformation,2e20);
    for(std::size_t k=0;k<data.nodes.size();++k) {
        const auto& actual=data.nodes[k];const auto& source=yaris_type13_fixture::Nodes[k];
        EXPECT_EQ(actual.id,source.id);EXPECT_EQ(actual.source_line,source.line);EXPECT_EQ(actual.blank_mask,source.blank_mask);
        const double native[3]={actual.position_native.x,actual.position_native.y,actual.position_native.z};
        const double si[3]={actual.position_m.x,actual.position_m.y,actual.position_m.z};
        for(unsigned axis=0;axis<3;++axis){EXPECT_EQ(output::Bits(native[axis]),output::Bits(source.native[axis]));
            EXPECT_EQ(output::Bits(si[axis]),output::Bits(source.si[axis]));}
    }
    long double mass=0;
    for(std::size_t k=0;k<data.beams.size();++k) {
        const auto& actual=data.beams[k];const auto& source=yaris_type13_fixture::Beams[k];
        EXPECT_EQ(actual.id,source.id);EXPECT_EQ(actual.source_line,source.line);
        for(unsigned n=0;n<3;++n)EXPECT_EQ(actual.node_indices[n],source.nodes[n]);
        EXPECT_EQ(actual.startup.reference.branch,native::FrameBranch::ThirdNode);
        EXPECT_GT(actual.startup.endpoint.isotropic_inertia_kg_m2,0);
        EXPECT_EQ(actual.startup.endpoint.added_inertia_kg_m2,0);
        mass+=2*static_cast<long double>(actual.startup.endpoint.mass_kg);
    }
    EXPECT_GT(mass,1.61L);EXPECT_LT(mass,1.62L);
    const auto copy=input;EXPECT_EQ(copy.data().nodes.data(),data.nodes.data());
}
TEST(Type13ActualSource,RehashedSemanticCorruptionAndCapsRejectWithoutReplacingInput) {
    auto input=SourceType13::Read(FixturePath(),Identity);const auto* before=input.data().beams.data();
    const auto bytes=output::ReadBounded(FixturePath(),Identity.bytes);
    const std::function<void(output::Document&)> changes[]={
        [](auto& d){d["simulation_ready"].SetBool(true);},
        [](auto& d){d["units"]["length_to_m"].SetDouble(1);},
        [](auto& d){d["policy"]["resolved"]["H"].SetInt(3);},
        [](auto& d){d["property"]["material"]["cards"][0]["values"][5].SetDouble(4999);},
        [](auto& d){d["property"]["section"]["cards"][1]["values"][2].SetDouble(0);},
        [](auto& d){auto& n=d["nodes"];n[n.Size()-1]["position_native"][2].SetDouble(1);},
        [](auto& d){auto& n=d["nodes"];n[n.Size()-1]["source_id"].SetUint64(n[0]["source_id"].GetUint64());},
        [](auto& d){auto& b=d["beams"];b[b.Size()-1]["node_indices"][1].SetUint(0);},
        [](auto& d){auto& b=d["beams"];b[b.Size()-1]["raw_record"][5].SetUint(1);},
        [](auto& d){d["source"]["arrays"]["node_ids"]["bytes"].SetUint(1);},
        [](auto& d){auto& n=d["nodes"];n[n.Size()-1]["source_line"].SetUint(700000);},
        [](auto& d){d["units"].SetInt(3);},
        [](auto& d){auto& n=d["nodes"][d["nodes"].Size()-1];
            std::string raw=n["raw_text"].GetString();raw.replace(8,16,"            42.0");
            n["raw_text"].SetString(raw.c_str(),raw.size(),d.GetAllocator());
            n["position_native"][0].SetDouble(42);n["position_m"][0].SetDouble(42*.001);},
        [](auto& d){auto& b=d["beams"];bool changed=false;
            for(unsigned i=1;i<b.Size();++i)if(b[i]["canonical_index"].GetUint()>b[i-1]["canonical_index"].GetUint()+1){
                b[i]["canonical_index"].SetUint(b[i]["canonical_index"].GetUint()-1);changed=true;break;}
            output::Require(changed,"Actual source must retain an intervening nonscope beam interval");},
        [](auto& d){auto& b=d["beams"];b[b.Size()-1]["source_line"].SetUint(b[b.Size()-1]["source_line"].GetUint()+1);},
    };
    for(std::size_t i=0;i<std::size(changes);++i) {
        SCOPED_TRACE(i);const auto bad=Alter(bytes,changes[i]);
        EXPECT_THROW(input=SourceType13::ReadBytes(bad,{bad.size(),output::Sha256(bad)}),std::runtime_error);
        EXPECT_EQ(input.data().beams.data(),before);
    }
    EXPECT_THROW(SourceType13::Read(FixturePath(),{}),std::runtime_error);
    EXPECT_THROW(SourceType13::Read(FixturePath(),Identity,{Identity.bytes,7493,4442}),std::runtime_error);
    EXPECT_THROW(SourceType13::Read(FixturePath(),Identity,{Identity.bytes,7494,4441}),std::runtime_error);
    EXPECT_THROW(SourceType13::Read(FixturePath(),Identity,{Identity.bytes-1,7494,4442}),std::runtime_error);
    const auto budget=input.data().startup_budget_bytes;
    EXPECT_THROW(SourceType13::Read(FixturePath(),Identity,{Identity.bytes,8192,8192,budget-1}),std::runtime_error);
    EXPECT_NO_THROW(SourceType13::Read(FixturePath(),Identity,{Identity.bytes,8192,8192,budget}));
    input=SourceType13::Read(FixturePath(),Identity);
    EXPECT_EQ(input.data().beams.back().id,2409378);
}
}
