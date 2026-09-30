#include "../TimingWindow.h"
#include "../Loop.h"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
namespace crash::cases::vehicle_run::test {
TEST(VehicleTimingWindow, ExactInteriorEndpointsUseAccumulatedClockAndFreezeCompletedResult) {
    TimingWindow window({10,100},101,{0,101},1000);double time=0;
    for(unsigned epoch=0;epoch<101;++epoch){Progress p;p.planned_intervals=101;p.accepted={epoch,time};p.elapsed_s=100+epoch*.25;
        window.Observe(p);time+=150e-9;
        EXPECT_EQ(window.result().started,epoch>=10);EXPECT_EQ(window.result().complete,epoch>=100);}
    const auto& r=window.result();EXPECT_EQ(r.measured_intervals,90u);EXPECT_EQ(r.wall_s,22.5);
    EXPECT_EQ(r.first.epoch,10u);EXPECT_EQ(r.last.epoch,100u);EXPECT_EQ(r.first_elapsed_s,102.5);EXPECT_EQ(r.last_elapsed_s,125);
    double first=0,last=0;for(unsigned i=0;i<100;++i){last+=150e-9;if(i==9)first=last;}
    EXPECT_EQ(r.first.time_s,first);EXPECT_EQ(r.last.time_s,last);
}
TEST(VehicleTimingWindow, PartialWindowNeverInventsDuration) {
    TimingWindow window({2,4},6,{0,6},100);
    for(unsigned epoch=0;epoch<4;++epoch){Progress p;p.planned_intervals=6;p.accepted={epoch,double(epoch)};p.elapsed_s=double(epoch);window.Observe(p);}
    EXPECT_TRUE(window.result().started);EXPECT_FALSE(window.result().complete);EXPECT_EQ(window.result().wall_s,0);EXPECT_EQ(window.result().measured_intervals,0u);
}
TEST(VehicleTimingWindow, ArchivePreflightRejectsInteriorFramesAndChunkFlushes) {
    EXPECT_NO_THROW(TimingWindow({10,100},101,{0,10,101},101));
    EXPECT_NO_THROW(TimingWindow({10,19},21,{0,21},10)); // First-boundary flush is already complete.
    for(auto samples:{std::vector<std::uint64_t>{0,11,101},{0,100,101},{0,10,10,101},{0,102,101},{1,101}})
        EXPECT_THROW(TimingWindow({10,100},101,samples,1000),std::invalid_argument);
    for(auto chunk:{0u,1u,50u,100u})EXPECT_THROW(TimingWindow({10,100},101,{0,101},chunk),std::invalid_argument);
    for(auto request:{TimingWindowRequest{0,100},{10,10},{100,10},{10,101}})
        EXPECT_THROW(TimingWindow(request,101,{0,101},1000),std::invalid_argument);
}
TEST(VehicleTimingWindow, InvalidBoundaryPreservesStateThenAllowsCorrectRetry) {
    TimingWindow window({1,3},5,{0,5},100);Progress p;p.planned_intervals=5;window.Observe(p);
    p.accepted={1,.5};p.elapsed_s=.25;window.Observe(p);const auto saved=window.result();
    for(unsigned field=0;field<6;++field){auto bad=p;bad.accepted={2,1};bad.elapsed_s=.5;
        if(field==0)bad.accepted.epoch=3;if(field==1)bad.accepted.time_s=.5;if(field==2)bad.elapsed_s=.1;
        if(field==3)bad.elapsed_s=std::numeric_limits<double>::infinity();if(field==4)bad.planned_intervals=6;if(field==5)bad.accepted.time_s=std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW(window.Observe(bad),std::invalid_argument);EXPECT_EQ(window.result().first_elapsed_s,saved.first_elapsed_s);EXPECT_FALSE(window.result().complete);}
    p.accepted={2,1};p.elapsed_s=.5;window.Observe(p);p.accepted={3,1.5};p.elapsed_s=.75;window.Observe(p);EXPECT_EQ(window.result().wall_s,.5);
}
struct WindowOps:detail::Operations {
    Endpoint endpoint;double clock=0;unsigned captured=0,saved=0,appended=0,discarded=0;bool finished=false;
    Endpoint Accepted()const noexcept override{return endpoint;}
    void Prepare()override{clock+=2;}
    void Commit()override{++endpoint.epoch;endpoint.time_s+=150e-9;clock+=3;}
    void Discard()noexcept override{++discarded;}
    void Append()override{++appended;clock+=5;}
    void Capture()override{++captured;clock+=100;}
    void SaveSample()override{++saved;clock+=200;}
    void Finish(bool,const std::string&)override{finished=true;clock+=400;}
};
TEST(VehicleTimingWindow, ExistingLoopCallbackIncludesBufferedAppendAndExcludesEndpointArchives) {
    WindowOps ops;TimingWindow window({10,100},101,{0,101},1000);Control control;unsigned calls=0;
    control.accepted_boundary=[&](const Progress& p){EXPECT_EQ(p.accepted.epoch,ops.appended);++calls;window.Observe(p);};
    Horizon plan{101,150e-9*101,150e-9,150e-9L*101};
    const auto result=detail::RunLoop(ops,plan,{0,101},control,[&]{return ops.clock;});
    EXPECT_EQ(result.kind,StopKind::Completed);EXPECT_EQ(calls,101u);EXPECT_EQ(ops.captured,2u);EXPECT_EQ(ops.saved,2u);
    EXPECT_TRUE(window.result().complete);EXPECT_EQ(window.result().wall_s,90*10.);EXPECT_EQ(result.progress.elapsed_s,1010+1000.);
}
TEST(VehicleTimingWindow, ObserverFailureStopsBeforeAnotherPrepareAndKeepsAcceptedPrefix) {
    WindowOps ops;Control control;control.accepted_boundary=[](const Progress& p){if(p.accepted.epoch==2)throw std::runtime_error("observer rejected");};
    Horizon plan{5,750e-9,150e-9,750e-9L};const auto result=detail::RunLoop(ops,plan,{0,5},control,[&]{return ops.clock;});
    EXPECT_EQ(result.kind,StopKind::ObserverFailure);EXPECT_EQ(result.progress.accepted.epoch,2u);EXPECT_EQ(ops.appended,2u);EXPECT_EQ(ops.discarded,0u);EXPECT_TRUE(result.valid_manifest);
}
} // namespace crash::cases::vehicle_run::test
