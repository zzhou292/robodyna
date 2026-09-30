#include "Support.h"
#include "../ReplayBudget.h"
#include <functional>
namespace crash::output::physical_run::test {
namespace {
void Rejects(const std::function<void()>& operation,const char* message) {
    try { operation(); FAIL() << "Expected admission rejection"; }
    catch(const std::exception& error) { EXPECT_STREQ(error.what(),message); }
}
Profile NativeGroup() {Profile p;p.type45=true;p.structural_limit=true;p.beam18=true;p.native_group=true;return p;}
records::Context LargeRecordShape() {
    // Count-only Context owns a small parent table, not large frame arrays.
    // Actual record limits permit64 points per parent and4M points total.
    std::vector<records::ParentPoints> parents(65536);
    for(std::size_t i=0;i<parents.size();++i)
        parents[i]={i+1,201,2,1,64,records::PlasticField::NativeEquivalentPlasticStrain};
    return records::Context::Create(ft::Id(),1048576,parents.data(),parents.size(),.125);
}
}
TEST(PhysicalReplayPreflight,ActualPublicBoundaryUsesSharedBudgetForAllStaticProfiles) {
    const auto c=Context();
    for(const auto p:{Profile{},Profile{true,true,true},NativeGroup()})for(unsigned stat=0;stat<3;++stat) {
        auto config=Config(c,p);config.wall=stat==1;config.environment=stat==2;
        config.request.extra_interval_bytes=ExtraIntervalBytes(p);
        const ReplayLimits limits;
        const auto reference=replay_detail::Budget(c,config,limits.source.host_bytes,limits.host_bytes);
        EXPECT_EQ(Replay::Preflight(c,config,limits),reference.peak_host_bytes);
    }
}
TEST(PhysicalReplayPreflight,DefaultEnvelopeExactAndOneByteShortAreEarlyLimitErrors) {
    const auto c=Context();const auto config=Config(c);ReplayLimits limits;
    EXPECT_EQ(limits.host_bytes,512u<<20);EXPECT_EQ(limits.source.host_bytes,384u<<20);
    EXPECT_NO_THROW(Replay::Preflight(c,config,limits));
    --limits.host_bytes;
    Rejects([&]{(void)Replay::Preflight(c,config,limits);},"Physical replay source and record workspace exceed host cap");
    // The original Open rejection must still precede its first file access.
    Rejects([&]{(void)Replay::Open("/absent-native-replay-preflight",{}, {},std::string(64,'a'),limits);},
        "Physical replay source and record workspace exceed host cap");
}
TEST(PhysicalReplayPreflight,RetainedPeakExactAndOneByteShortExerciseTheBudgetNotEnvelope) {
    const auto c=LargeRecordShape();auto config=Config(c,NativeGroup());config.environment=true;
    ReplayLimits limits;limits.source.host_bytes=1u<<20;
    const auto exact=Replay::Preflight(c,config,limits);
    ASSERT_GT(exact,limits.source.host_bytes+(128u<<20));
    ASSERT_LE(exact,512u<<20);
    limits.host_bytes=exact;
    EXPECT_EQ(Replay::Preflight(c,config,limits),exact);
    --limits.host_bytes;
    Rejects([&]{(void)Replay::Preflight(c,config,limits);},"Physical replay retained/peak buffers exceed host cap");
    // Same valid count-only shape exceeds the unchanged default512MiB reader.
    Rejects([&]{(void)Replay::Preflight(c,config);},"Physical replay retained/peak buffers exceed host cap");
}
TEST(PhysicalReplayPreflight,HorizonAndChunkTransitionsUseTheRealReaderStagingPlan) {
    const auto c=Context();auto config=Config(c,NativeGroup());config.environment=true;
    config.request.extra_interval_bytes=ExtraIntervalBytes(config.profile);
    const auto chunk=interval::PlanChunks(1,config.request.file_byte_cap,
        kArtifactMaximumTotalCap,config.request.extra_interval_bytes).rows_per_chunk;
    const ReplayLimits limits;
    std::size_t full=0;
    for(std::uint64_t count:{UINT64_C(1),UINT64_C(25001),UINT64_C(50001),chunk-1,chunk,chunk+1,2*chunk+1}) {
        config.request.intervals=count;
        const auto reference=replay_detail::Budget(c,config,limits.source.host_bytes,limits.host_bytes);
        EXPECT_EQ(Replay::Preflight(c,config,limits),reference.peak_host_bytes);
        if(count==chunk)full=reference.interval_workspace;
        if(count>chunk)EXPECT_EQ(reference.interval_workspace,full);
    }
    config.request.intervals=UINT64_MAX;
    EXPECT_THROW(Replay::Preflight(c,config,limits),std::exception);
}
TEST(PhysicalReplayPreflight,ReaderSharesExactWorkspaceAdmissionAndPreservesCoverageFirst) {
    const auto c=Context();const Profile profile{false,true};ft::Directory dir;
    IntervalWriter writer(dir.path,c,profile,5,624,4096);
    for(unsigned epoch=1;epoch<=5;++epoch)writer.Append(Row(c,epoch));
    const auto segments=writer.Finish();ASSERT_EQ(segments.size(),3u);
    const auto bytes=IntervalReadStagingBytes(profile,5,624);
    EXPECT_NO_THROW(CheckIntervalReadWorkspace(bytes,bytes));
    EXPECT_NO_THROW(ReadIntervals(dir.path,c,profile,5,5,segments,624,bytes,{}));
    Rejects([&]{(void)ReadIntervals(dir.path,c,profile,5,5,segments,624,bytes-1,{});},
        "Physical interval read staging exceeds host cap");
    auto incomplete=segments;incomplete.pop_back();
    Rejects([&]{(void)ReadIntervals(dir.path,c,profile,5,5,incomplete,624,0,{});},
        "Incomplete physical interval segment coverage");
    Rejects([&]{CheckIntervalReadWorkspace(bytes,0);},"Physical interval read staging exceeds host cap");
    Rejects([&]{CheckIntervalReadWorkspace(bytes,(256u<<20)+1);},"Physical interval read staging exceeds host cap");
}
TEST(PhysicalReplayPreflight,MalformedIndexStillPrecedesTheLaterIntervalWorkspaceError) {
    const auto c=Context();const auto config=Config(c);ft::Directory dir;
    const auto index=WriteRun(dir.path,c,config,4);
    auto invalid=index;++invalid.planned_intervals;
    Rejects([&]{ValidateRecords(dir.path,c,config,invalid,0);},"Physical run endpoint/source differs");
    Rejects([&]{ValidateRecords(dir.path,c,config,index,0);},"Physical interval read staging exceeds host cap");
    EXPECT_NO_THROW(ValidateRecords(dir.path,c,config,index,64u<<20));
}
TEST(PhysicalReplayPreflight,InvalidStaticEnvelopeRejectsWithoutCreatingOutputAuthority) {
    const auto c=Context();auto config=Config(c);config.wall=config.environment=true;
    Rejects([&]{(void)Replay::Preflight(c,config);},"Invalid physical replay memory envelope");
    EXPECT_EQ(c.nodes(),5u);EXPECT_EQ(c.points(),8u);
}
} // namespace crash::output::physical_run::test
