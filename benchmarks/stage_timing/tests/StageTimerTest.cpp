#include "benchmarks/stage_timing/StageTimer.h"
#include <gtest/gtest.h>
#include <array>
#include <stdexcept>

namespace crash::benchmarks {
namespace {
struct Fake {
    std::array<std::uint64_t,16> ticks{};
    std::size_t calls=0,fail_on=SIZE_MAX;
    static bool Read(void* context,std::uint64_t* output) noexcept {
        auto& f=*static_cast<Fake*>(context);const auto i=f.calls++;
        errno=ENOSPC;*output=f.ticks[i%f.ticks.size()];return i!=f.fail_on;
    }
    StageClock clock() {return {Read,this};}
};
struct Result {int code=0;const char* message=nullptr;explicit operator bool() const {return code==0;} };
TEST(StageTimer, DisabledCallsNoClockAndPreservesValueReferenceExceptionAndErrno) {
    Fake f;StageTimer<3> timer(false,f.clock());errno=ENOENT;unsigned invoked=0;
    Result value{7,"actual failure"};
    auto& returned=timer.Step([&]() -> Result& {++invoked;return timer.Measure<1>([&]() -> Result& {return value;});});
    EXPECT_EQ(&returned,&value);EXPECT_EQ(errno,ENOENT);EXPECT_EQ(invoked,1u);
    EXPECT_THROW(timer.Step([]() -> bool {throw std::runtime_error("same exception");}),std::runtime_error);
    EXPECT_EQ(f.calls,0u);EXPECT_EQ(timer.snapshot().total[0].calls,0u);
}
TEST(StageTimer, InclusiveStepAndDisjointStagesRetainFailureAndLastAttempt) {
    Fake f;f.ticks={100,110,130,140,170,200,300,350};StageTimer<3> timer(true,f.clock());
    auto r=timer.Step([&] {
        auto first=timer.Measure<1>([] {return Result{0,"first"};});
        EXPECT_TRUE(bool(first));return timer.Measure<2>([] {return Result{19,"late failure"};});
    });
    EXPECT_EQ(r.code,19);EXPECT_STREQ(r.message,"late failure");
    auto s=timer.snapshot();EXPECT_EQ(s.total[0].wall_ns,100u);EXPECT_EQ(s.total[1].wall_ns,20u);
    EXPECT_EQ(s.total[2].wall_ns,30u);EXPECT_EQ(s.total[0].failures,1u);EXPECT_EQ(s.total[2].failures,1u);
    EXPECT_TRUE(timer.Step([] {return true;}));s=timer.snapshot();
    EXPECT_EQ(s.total[0].calls,2u);EXPECT_EQ(s.total[0].wall_ns,150u);EXPECT_EQ(s.total[0].maximum_ns,100u);
    EXPECT_EQ(s.last_step[0].calls,1u);EXPECT_EQ(s.last_step[0].wall_ns,50u);EXPECT_EQ(s.last_step[1].calls,0u);
}
TEST(StageTimer, EnabledClockCannotChangeOperationErrnoAndExceptionsPropagate) {
    Fake f;f.ticks={1,3,5,9};StageTimer<2> timer(true,f.clock());errno=EDOM;
    EXPECT_TRUE(timer.Measure<1>([] {EXPECT_EQ(errno,EDOM);errno=EAGAIN;return true;}));EXPECT_EQ(errno,EAGAIN);
    try {timer.Step([]() -> bool {errno=EIO;throw std::runtime_error("original error");});FAIL();}
    catch(const std::runtime_error& e) {EXPECT_STREQ(e.what(),"original error");EXPECT_EQ(errno,EIO);}
    EXPECT_EQ(timer.snapshot().total[0].failures,1u);EXPECT_EQ(timer.snapshot().total[0].calls,1u);
}
TEST(StageTimer, InvalidAndBackwardClockSamplesNeverRejectOrFabricateDurations) {
    Fake f;f.ticks={10,20,50,40};f.fail_on=1;StageTimer<2> timer(true,f.clock());
    EXPECT_TRUE(timer.Measure<1>([] {return true;}));EXPECT_TRUE(timer.Measure<1>([] {return true;}));
    const auto s=timer.snapshot();EXPECT_EQ(s.clock_failures,1u);EXPECT_EQ(s.backward_samples,1u);
    EXPECT_EQ(s.total[1].calls,2u);EXPECT_EQ(s.total[1].valid_samples,0u);EXPECT_EQ(s.total[1].wall_ns,0u);
    EXPECT_EQ(s.total[1].failures,0u);
}
TEST(StageTimer, AccumulationSaturatesExplicitlyWithoutChangingResults) {
    Fake f;f.ticks={0,UINT64_MAX,0,1};StageTimer<2> timer(true,f.clock());
    EXPECT_TRUE(timer.Measure<1>([] {return true;}));EXPECT_FALSE(timer.Measure<1>([] {return false;}));
    const auto s=timer.snapshot();EXPECT_TRUE(s.counter_saturated);EXPECT_EQ(s.total[1].wall_ns,UINT64_MAX);
    EXPECT_EQ(s.total[1].maximum_ns,UINT64_MAX);EXPECT_EQ(s.total[1].valid_samples,2u);EXPECT_EQ(s.total[1].failures,1u);
}
TEST(StageTimer, InstancesKeepIndependentClocksAndByValueSnapshots) {
    Fake a,b;a.ticks={1,2,4,8};b.ticks={10,40};StageTimer<2> first(true,a.clock()),second(true,b.clock());
    first.Step([] {return true;});auto frozen=first.snapshot();second.Step([] {return true;});first.Step([] {return true;});
    EXPECT_EQ(frozen.total[0].wall_ns,1u);EXPECT_EQ(first.snapshot().total[0].wall_ns,5u);
    EXPECT_EQ(second.snapshot().total[0].wall_ns,30u);EXPECT_EQ(a.calls,4u);EXPECT_EQ(b.calls,2u);
}
}
}
