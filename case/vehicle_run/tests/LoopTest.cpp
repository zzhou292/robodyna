#include "../Loop.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <stdexcept>
namespace crash::cases::vehicle_run::test {
struct Fake final : detail::Operations {
    Endpoint accepted;
    std::uint64_t logged=0,saved=UINT64_MAX,fail_prepare=UINT64_MAX,fail_append=UINT64_MAX;
    std::uint64_t fail_capture=UINT64_MAX,fail_save=UINT64_MAX,fail_commit=UINT64_MAX;
    bool fail_finish=false;
    unsigned discarded=0,finished=0;
    bool completed=false;
    double time=0;
    std::vector<std::string> calls;
    Endpoint Accepted() const noexcept override {return accepted;}
    MechanicsTotals Mechanics() const noexcept override {
        MechanicsTotals result;
        result.available = logged != 0;
        result.intervals = logged;
        result.solids.metal_plastic_work.work.accepted_increment_sum_j = logged * .125;
        return result;
    }
    void Prepare() override {
        calls.push_back("prepare"+std::to_string(accepted.epoch+1));
        time+=2;
        if(accepted.epoch+1==fail_prepare) throw std::runtime_error("actual step rejection");
    }
    void Commit() override {
        if(accepted.epoch+1==fail_commit) throw std::runtime_error("common commit rejected");
        ++accepted.epoch;
        accepted.time_s+=.001;
        calls.push_back("commit"+std::to_string(accepted.epoch));
    }
    void Discard() noexcept override {++discarded;}
    void Append() override {
        if(accepted.epoch==fail_append) throw std::runtime_error("append I/O failure");
        EXPECT_EQ(accepted.epoch,logged+1);
        logged=accepted.epoch;
        calls.push_back("append"+std::to_string(logged));
    }
    void Capture() override {
        if(accepted.epoch==fail_capture) throw std::runtime_error("capture rejected");
        calls.push_back("capture"+std::to_string(accepted.epoch));
    }
    void SaveSample() override {
        if(accepted.epoch==fail_save) throw std::runtime_error("sample I/O failure");
        EXPECT_EQ(logged,accepted.epoch);
        saved=accepted.epoch;
        calls.push_back("sample"+std::to_string(saved));
    }
    void Finish(bool complete,const std::string&) override {
        if(fail_finish) throw std::runtime_error("manifest write failed");
        EXPECT_EQ(saved,accepted.epoch);
        EXPECT_EQ(logged,accepted.epoch);
        ++finished;
        completed=complete;
    }
};
LoopResult CheckRun(Fake& fake,Control control={}) {
    Config config;
    config.fixed_dt_s=.001;
    config.samples=3;
    return detail::RunLoop(fake,Plan(config),{0,2,5},control,[&] {return fake.time;});
}
TEST(VehicleRunLoop, CompleteOrderingIncludesOnlyScheduledSamplesAndActualOwnerTimes) {
    Fake fake;
    const auto result=CheckRun(fake);
    EXPECT_EQ(result.kind,StopKind::Completed);
    EXPECT_TRUE(result.valid_manifest);
    EXPECT_EQ(fake.calls,(std::vector<std::string>{"capture0","sample0",
        "prepare1","commit1","append1","prepare2","commit2","append2","capture2","sample2",
        "prepare3","commit3","append3","prepare4","commit4","append4",
        "prepare5","commit5","append5","capture5","sample5"}));
    EXPECT_EQ(result.progress.accepted.time_s,fake.accepted.time_s);
    EXPECT_EQ(result.progress.elapsed_s,10);
    EXPECT_EQ(result.progress.accepted_intervals_per_second,.5);
    EXPECT_EQ(fake.finished,1u);
    EXPECT_TRUE(fake.completed);
}
TEST(VehicleRunLoop, CooperativeOffCadenceAndInitialStopsExportExactPrefix) {
    for(std::uint64_t stop:{0u,3u}) {
        Fake fake;
        Control control;
        control.stop_requested=[&] {return fake.accepted.epoch==stop;};
        const auto result=CheckRun(fake,control);
        EXPECT_EQ(result.kind,StopKind::Requested);
        EXPECT_TRUE(result.valid_manifest);
        EXPECT_EQ(result.progress.accepted.epoch,stop);
        EXPECT_EQ(fake.saved,stop);
        EXPECT_EQ(fake.finished,1u);
        EXPECT_FALSE(fake.completed);
    }
}
TEST(VehicleRunLoop, RejectedAttemptDiscardsAndKeepsLoggedPrefixWithoutRetryOrClockChange) {
    Fake fake;
    fake.fail_prepare=3;
    const auto result=CheckRun(fake);
    EXPECT_EQ(result.kind,StopKind::PhysicsRejected);
    EXPECT_EQ(result.reason,"actual step rejection");
    EXPECT_TRUE(result.valid_manifest);
    EXPECT_EQ(fake.discarded,1u);
    EXPECT_EQ(fake.accepted.epoch,2u);
    EXPECT_EQ(fake.logged,2u);
    EXPECT_EQ(fake.saved,2u);
    EXPECT_EQ(fake.finished,1u);
    EXPECT_EQ(std::count(fake.calls.begin(),fake.calls.end(),"prepare3"),1);
    EXPECT_TRUE(result.progress.mechanics.available);
    EXPECT_EQ(result.progress.mechanics.intervals, 2u);
    EXPECT_EQ(result.progress.mechanics.solids.metal_plastic_work.work.accepted_increment_sum_j, .25);
}
TEST(VehicleRunLoop, OutputAndReadbackFailuresNeverPublishFalsePrefix) {
    for(unsigned stage=0;stage<3;++stage) {
        Fake fake;
        if(stage==0) fake.fail_append=3;
        if(stage==1) fake.fail_capture=2;
        if(stage==2) fake.fail_save=2;
        const auto result=CheckRun(fake);
        EXPECT_EQ(result.kind,stage==1?StopKind::CaptureFailure:StopKind::ArchiveFailure);
        EXPECT_FALSE(result.valid_manifest);
        EXPECT_EQ(fake.finished,0u);
    }
}
TEST(VehicleRunLoop, ElapsedLimitAndObserverFailuresStopAtAcceptedBoundary) {
    Fake fake;
    Control control;
    control.maximum_elapsed_s=3;
    const auto result=CheckRun(fake,control);
    EXPECT_EQ(result.kind,StopKind::TimeLimit);
    EXPECT_EQ(fake.accepted.epoch,2u);
    EXPECT_TRUE(result.valid_manifest);
    Fake observer;
    control={};
    control.progress_period_s=1;
    control.progress=[](const Progress&) {throw std::runtime_error("observer failed");};
    const auto failed=CheckRun(observer,control);
    EXPECT_EQ(failed.kind,StopKind::ObserverFailure);
    EXPECT_EQ(observer.accepted.epoch,1u);
    EXPECT_TRUE(failed.valid_manifest);
}
TEST(VehicleRunLoop, DiagnosticLimitCommitRejectionAndFinalWriteFailureHaveDistinctOutcomes) {
    Fake diagnostic;
    Control control;
    control.maximum_accepted_intervals=3;
    auto result=CheckRun(diagnostic,control);
    EXPECT_EQ(result.kind,StopKind::IntervalLimit);
    EXPECT_EQ(result.progress.accepted.epoch,3u);
    EXPECT_TRUE(result.valid_manifest);
    EXPECT_EQ(diagnostic.saved,3u);
    Fake rejected;
    rejected.fail_commit=3;
    result=CheckRun(rejected);
    EXPECT_EQ(result.kind,StopKind::PhysicsRejected);
    EXPECT_EQ(rejected.accepted.epoch,2u);
    EXPECT_EQ(rejected.logged,2u);
    EXPECT_EQ(rejected.discarded,1u);
    EXPECT_TRUE(result.valid_manifest);
    Fake failed;
    failed.fail_finish=true;
    result=CheckRun(failed);
    EXPECT_EQ(result.kind,StopKind::ArchiveFailure);
    EXPECT_EQ(result.progress.accepted.epoch,5u);
    EXPECT_FALSE(result.valid_manifest);
}
TEST(VehicleRunConfig, ExplicitHorizonsConditionalCapsAndInvalidInputs) {
    Config config;
    EXPECT_EQ(Plan(config).intervals,16667u);
    config.duration_s=.02;
    EXPECT_EQ(Plan(config).intervals,66667u);
    config.duration_s=.05;
    EXPECT_EQ(Plan(config).intervals,166667u);
    const auto normal=SelectCaps(ResourceProfile::ConditionalExpandedFull,19ull*1000*1000*1000,2ull<<30);
    EXPECT_FALSE(normal.expanded);
    EXPECT_EQ(normal.host_bytes,20ull*1000*1000*1000);
    EXPECT_EQ(normal.archive_bytes,2ull<<30);
    EXPECT_THROW(SelectCaps(ResourceProfile::Normal,1, (2ull<<30)+1),std::invalid_argument);
    const auto full=SelectCaps(ResourceProfile::ConditionalExpandedFull,1,(2ull<<30)+1);
    EXPECT_TRUE(full.expanded);
    EXPECT_EQ(full.host_bytes,normal.host_bytes);
    EXPECT_EQ(full.archive_bytes,6ull<<30);
    EXPECT_THROW(SelectCaps(ResourceProfile::ConditionalExpandedFull,60ull*1000*1000*1000+1,1),std::invalid_argument);
    config.duration_s=.006;
    EXPECT_THROW(Plan(config),std::invalid_argument);
}
} // namespace crash::cases::vehicle_run::test
