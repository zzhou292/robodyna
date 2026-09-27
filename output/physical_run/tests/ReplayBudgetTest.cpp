#include "Support.h"
#include "../ReplayBudget.h"
#include <algorithm>
namespace crash::output::physical_run::test {
TEST(PhysicalReplayBudget,SequentialPhasesUseLargestWorkspaceAndRetainAllFixedTerms) {
    const auto c=Context();auto config=Config(c);const std::size_t source=1u<<20;
    for(unsigned environment=0;environment<3;++environment) {
        config.wall=environment==1;config.environment=environment==2;
        const auto v=replay_detail::Budget(c,config,source,512u<<20);
        EXPECT_EQ(v.interval_workspace,IntervalReadStagingBytes(config.profile,4,config.request.file_byte_cap));
        EXPECT_EQ(v.frame_workspace,3*sizeof(double)*(3*c.nodes()+c.points()));
        const auto external=environment==1?WallWorkspaceBytes:environment==2?EnvironmentWorkspaceBytes:0;
        const auto retained=environment==1?WallMeshRetainedBytes:environment==2?EnvironmentRetainedBytes:0;
        EXPECT_EQ(v.environment_workspace,external);
        EXPECT_EQ(v.sequential_workspace,std::max({v.interval_workspace,v.frame_workspace,external}));
        EXPECT_EQ(v.peak_host_bytes,source+c.retained_payload_bytes()+32*MetadataCap+v.sequential_workspace+retained);
    }
}
TEST(PhysicalReplayBudget,ExactAccountingCapAndOneByteShortPreserveReaderEnvelope) {
    const auto c=Context();auto config=Config(c);config.environment=true;
    const auto expected=replay_detail::Budget(c,config,1u<<20,512u<<20);
    EXPECT_EQ(replay_detail::Budget(c,config,1u<<20,expected.peak_host_bytes).peak_host_bytes,expected.peak_host_bytes);
    EXPECT_THROW(replay_detail::Budget(c,config,1u<<20,expected.peak_host_bytes-1),std::exception);
    EXPECT_THROW(replay_detail::Budget(c,config,1u<<20,0),std::exception);
    EXPECT_THROW(replay_detail::Budget(c,config,1u<<20,(512u<<20)+1),std::exception);
    config.wall=true;EXPECT_THROW(replay_detail::Budget(c,config,1u<<20,512u<<20),std::exception);
}
TEST(PhysicalReplayBudget,GrowingNativeGroupHorizonChargesChunkPeakWithoutSummingFramePhase) {
    const auto c=Context();auto config=Config(c);config.profile={true,true,true};config.profile.native_group=true;
    config.environment=true;config.request.extra_interval_bytes=ExtraIntervalBytes(config.profile);
    std::size_t previous=0;
    for(std::uint64_t planned:{UINT64_C(25001),UINT64_C(50001),UINT64_C(200001)}) {
        config.request.intervals=planned;
        const auto v=replay_detail::Budget(c,config,384u<<20,512u<<20);
        EXPECT_GE(v.interval_workspace,previous);previous=v.interval_workspace;
        EXPECT_EQ(v.sequential_workspace,std::max({v.interval_workspace,v.frame_workspace,EnvironmentWorkspaceBytes}));
        EXPECT_LE(v.peak_host_bytes,512u<<20);
        if(planned==50001)EXPECT_EQ(v.interval_workspace,UINT64_C(43200864));
    }
    config.request.intervals=UINT64_MAX;
    EXPECT_THROW(replay_detail::Budget(c,config,384u<<20,512u<<20),std::exception);
}
TEST(PhysicalReplayBudget,ExistingReaderLimitsRemainUnchangedAndRejectBeforeFileReads) {
    ReplayLimits limits;EXPECT_EQ(limits.host_bytes,512u<<20);EXPECT_EQ(limits.source.host_bytes,384u<<20);
    limits.host_bytes=(512u<<20)+1;
    try { (void)Replay::Open("/absent-physical-replay-budget",{}, {},std::string(64,'a'),limits);FAIL(); }
    catch(const std::exception& error) {EXPECT_STREQ(error.what(),"Physical replay source and record workspace exceed host cap");}
}
}
