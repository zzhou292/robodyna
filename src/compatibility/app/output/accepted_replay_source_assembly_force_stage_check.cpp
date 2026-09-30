#include "AcceptedReplay.h"
#include "ReplayBundleTestSupport.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace crash::output {
namespace {
using test_support::ModifiedReplayBundle;
std::string FrameName(std::uint64_t epoch) {std::ostringstream s;s<<"accepted-"<<std::setw(6)<<std::setfill('0')<<epoch<<".fields.json";return s.str();}
class AcceptedReplayForceStage:public ::testing::Test {
  protected:
    void SetUp() override {
        const auto* p=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_FORCE_STAGE_REPLAY_FIXTURE");
        if(!p||!*p)GTEST_SKIP()<<"Set ROBO_DYNA_SOURCE_ASSEMBLY_FORCE_STAGE_REPLAY_FIXTURE to an enabled actual owner archive";
        source=p;
    }
    std::filesystem::path source;
};
TEST_F(AcceptedReplayForceStage,ActualEnabledSourceArchiveAuthenticatesEverySavedObservation) {
    AcceptedReplay reader;const auto opened=reader.Open(source);ASSERT_EQ(opened.status,ReplayStatus::Ok)<<opened.diagnostic;
    ASSERT_TRUE(reader.info()->source_assembly);EXPECT_TRUE(reader.info()->source_assembly->observe_force_stage);
    ASSERT_GT(reader.info()->frame_count,1u);
    for(std::size_t i=0;i<reader.info()->frame_count;++i) {
        SCOPED_TRACE(i);const auto loaded=reader.Load(i);ASSERT_EQ(loaded.status,ReplayStatus::Ok)<<loaded.diagnostic;
        const auto d=replay_detail::Json(ReadBounded(source/FrameName(reader.frame()->epoch),32*1024*1024));
        ASSERT_TRUE(d.HasMember("force_stage_kinetic"));const auto& f=d["force_stage_kinetic"];
        if(!i)EXPECT_TRUE(f.IsNull());else {
            EXPECT_EQ(f["enclosing_epoch"].GetUint64(),reader.frame()->epoch);
            EXPECT_EQ(f["base_epoch"].GetUint64()+1,reader.frame()->epoch);
            EXPECT_EQ(Bits(f["phase"]["force_time_s"].GetDouble()),Bits(d["stamp"]["reaction_time"].GetDouble()));
            EXPECT_LT(f["phase"]["force_time_s"].GetDouble(),reader.frame()->time);
        }
    }
}
TEST_F(AcceptedReplayForceStage,RehashedLateRecordIdentityPhaseAndPartitionsCannotBeRelabelled) {
    for(unsigned fault=0;fault<14;++fault) {
        SCOPED_TRACE(fault);ModifiedReplayBundle b(source);const auto manifest=b.Read("manifest.json");
        const auto file=FrameName(manifest["accepted_epoch"].GetUint64());auto frame=b.Read(file);auto& f=frame["force_stage_kinetic"];
        if(fault==0)f["owner_id"].SetUint64(0);if(fault==1)f["source"]["member_count"].SetUint64(75);
        if(fault==2)f["base_epoch"].SetUint64(f["enclosing_epoch"].GetUint64());if(fault==3)f["attempt"].SetUint64(0);
        if(fault==4)f["enclosing_time_s"].SetDouble(0);if(fault==5)f["phase"]["force_time_s"].SetDouble(frame["accepted_time_s"].GetDouble());
        if(fault==6)f["phase"]["previous_drift_dt_s"].SetDouble(-1);if(fault==7)f["phase"]["kick_dt_s"].SetDouble(0);
        if(fault==8)f["ordinary_native_nodes"][0u].SetDouble(-1);if(fault==9)f["grouped_native_members"][3u].SetDouble(1);
        if(fault==10)f["aggregate_groups"][4u].SetDouble(1);if(fault==11)f["native_total_J"].SetDouble(-1);
        if(fault==12)f["replacement_J"].SetDouble(1e100);if(fault==13)frame.RemoveMember("force_stage_kinetic");
        b.Replace(file,frame);b.Rehash(file);AcceptedReplay reader;
        EXPECT_EQ(reader.Open(b.directory).status,ReplayStatus::InvalidBundle);
    }
}
TEST_F(AcceptedReplayForceStage,OptionalDeclarationAndInitialAbsenceAreEnforced) {
    for(unsigned fault=0;fault<4;++fault) {
        SCOPED_TRACE(fault);ModifiedReplayBundle b(source);
        if(fault<2) {auto c=b.Read("configuration.json");if(!fault)c.RemoveMember("observe_force_stage");else c["observe_force_stage"].SetBool(false);
            b.Replace("configuration.json",c);b.Rehash("configuration.json");}
        else {const auto file=FrameName(0);auto f=b.Read(file);
            if(fault==2)f.RemoveMember("force_stage_kinetic");else f["force_stage_kinetic"].SetObject();b.Replace(file,f);b.Rehash(file);}
        AcceptedReplay reader;EXPECT_EQ(reader.Open(b.directory).status,ReplayStatus::InvalidBundle);
    }
}
TEST(AcceptedReplaySourceAssembly,DisabledFixtureRejectsUndeclaredForceStageRecord) {
    const auto* source=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_REPLAY_FIXTURE");if(!source||!*source)GTEST_SKIP()<<"Requires existing disabled archive";
    ModifiedReplayBundle b(source);auto c=b.Read("configuration.json");if(c.HasMember("observe_force_stage"))GTEST_SKIP()<<"Fixture enables optional force-stage observations";
    const auto file=FrameName(0);auto f=b.Read(file);Value value;f.AddMember("force_stage_kinetic",value,f.GetAllocator());
    b.Replace(file,f);b.Rehash(file);AcceptedReplay reader;EXPECT_EQ(reader.Open(b.directory).status,ReplayStatus::InvalidBundle);
}
}
} // namespace crash::output
