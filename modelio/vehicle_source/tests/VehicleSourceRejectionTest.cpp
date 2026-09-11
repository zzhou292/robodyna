#include "TestSupport.h"

namespace crash::modelio::vehicle::test {
TEST(VehicleSourcePlan, LateMalformedDeclarationsNeverReplacePublishedPlanAndRetryExactly) {
    const auto& before=Plan();const auto* parents=before.parents().data();const auto* nodes=before.canonical_nodes().data();
    for(unsigned fault=0;fault<21;++fault) {
        SCOPED_TRACE(fault);const auto bytes=Alter([&](auto& d) {
            auto& rows=d["parts"];auto& typed=d["supported_declarations"]["declarations"];
            auto& last=rows[rows.Size()-1];
            switch(fault) {
            case 0:last["part_id"].SetUint64(9999999);break;
            case 1:last["shells"].SetUint64(last["shells"].GetUint64()+1);break;
            case 2:last["material_id"].SetUint64(9999999);break;
            case 3:rows.PopBack();break;
            case 4:d["counts"]["supported_shells"].SetUint64(349645);break;
            case 5:d["simulation_ready"].SetBool(true);break;
            case 6:d["source"]["member_sha256"].SetString(std::string(64,'0').c_str(),d.GetAllocator());break;
            case 7:typed["materials"][typed["materials"].Size()-1]["young_pa"].SetDouble(123);break;
            case 8:typed["sections"][typed["sections"].Size()-1]["thickness_m"][3].SetDouble(123);break;
            case 9:typed["materials"][0]["material_law"].SetString("layered_law1",d.GetAllocator());break;
            case 10:typed["materials"][0]["cards"][0]["values"][2].SetDouble(123);break;
            case 11:d["supported_declarations"]["selected_part_ids"].PopBack();break;
            case 19: {
                auto& m=typed["materials"][0];m["young_pa"].SetDouble(m["young_pa"].GetDouble()*.5);
                auto& value=m["cards"][0]["values"][2];value.SetDouble(value.GetDouble()*.5);break;
            }
            case 20:rows[0]["status"].SetString("unresolved",d.GetAllocator());break;
            default:
                for(auto& row:rows.GetArray())if(std::string(row["status"].GetString())=="unresolved") {
                    if(fault==12)row["status"].SetString("supported_declaration",d.GetAllocator());
                    if(fault==13)row["obligations"].Clear();
                    if(fault==14)row["source_blocks"]["material"]["sha256"].SetString(std::string(64,'0').c_str(),d.GetAllocator());
                    if(fault==15)row["source_blocks"]["section"]["filename"].SetString("different.key",d.GetAllocator());
                    if(fault==16)row["source_blocks"].RemoveMember("part");
                    if(fault==17)row["source_blocks"]["part"]["first_line"].SetUint64(1);
                    if(fault==18)row["source_blocks"]["part"]["keyword"].SetString("*MAT_RIGID",d.GetAllocator());
                    break;
                }
            }
        });
        EXPECT_THROW(VehicleSourcePlan::ReadBytes(Canonical(),bytes,Identity(bytes)),std::runtime_error);
        EXPECT_EQ(before.parents().data(),parents);EXPECT_EQ(before.canonical_nodes().data(),nodes);EXPECT_EQ(before.counts().parents,349645);
    }
    auto retry=VehicleSourcePlan::ReadBytes(Canonical(),Bytes(),Identity(Bytes()));
    ASSERT_EQ(retry.parents().size(),before.parents().size());
    for(std::size_t i=0;i<retry.parents().size();++i) {
        ASSERT_EQ(retry.parents()[i].canonical_parent,before.parents()[i].canonical_parent);
        ASSERT_EQ(retry.parents()[i].part_index,before.parents()[i].part_index);
    }
}
TEST(VehicleSourcePlan, CountByteIdentityAndInvalidLimitsRejectBeforeFileReads) {
    const auto identity=Identity(Bytes());
    for(unsigned fault=0;fault<8;++fault) {
        Limits l;auto id=identity;
        switch(fault) {
        case 0:l.parents=349644;break;case 1:l.nodes=359784;break;case 2:l.parts=866;break;
        case 3:l.declaration_bytes=identity.bytes-1;break;case 4:l.host_bytes=Plan().startup_budget_bytes()-1;break;
        case 5:l.parents=524289;break;case 6:id.sha256="bad";break;case 7:l.tables=0;break;
        }
        try {
            VehicleSourcePlan::Read(Canonical(),"/path/not/read/by/cap/preflight",id,l);ADD_FAILURE()<<"Invalid preflight accepted";
        } catch(const std::runtime_error& error) {
            const std::string reason=error.what();
            EXPECT_TRUE(reason=="Invalid vehicle source plan limits"||
                reason=="Vehicle source count/content identity exceeds plan limits"||
                reason=="Vehicle source startup byte cap exceeded")<<reason;
        }
    }
    EXPECT_THROW(VehicleSourcePlan::ReadBytes(Canonical(),Bytes()+" ",identity),std::runtime_error);
    auto bytes=Bytes();bytes.back()=' ';
    EXPECT_THROW(VehicleSourcePlan::ReadBytes(Canonical(),bytes,identity),std::runtime_error);
    EXPECT_EQ(Plan().counts().supported_parents,278301);
}
} // namespace crash::modelio::vehicle::test
