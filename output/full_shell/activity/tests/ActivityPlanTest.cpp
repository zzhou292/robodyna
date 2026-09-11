#include "Support.h"

namespace crash::output::full_shell::activity::test {
namespace {
PlanRequest FullRequest(const Context& c) {
    PlanRequest r;r.nodes=c.nodes();r.parents=c.parents().size();r.plastic_points=c.points();
    r.frames=100;r.intervals=16384;r.fixed_dt=c.fixed_dt();r.requested_duration=r.intervals*r.fixed_dt;
    // Known canonical static package totals144417279 bytes. These bounded slots
    // are forecast fixtures, not a new source bundle or actual file inventory.
    for(unsigned i=0;i<4;++i)r.static_files.push_back({"canonical-reservation-"+std::to_string(i)+".bin",33554432});
    r.static_files.push_back({"canonical-reservation-tail.bin",10199551});
    r.static_files.push_back({"yaris-vehicle-declarations-1.json",3648589});
    r.static_files.push_back({"yaris-type13-startup-declaration-1.json",5150841});
    r.static_files.push_back({"wall.mesh.json",1048576});
    r.static_files.push_back({"configuration.json",65536});
    r.static_files.push_back({"manifest.json",16384});
    r.static_files.push_back({"frame-index.json",65536});return r;
}
}
TEST(ParentActivity, CompleteFrameAndPrefixForecastIncludesBothFilesAndExplicitSidecar) {
    const auto c=Sized(349645,359785);const auto request=FullRequest(c);
    const auto legacy=PlanArchive(request);const auto added=PlanWithActivity(c,request,"parent-activity.json");
    EXPECT_EQ(added.packed_frame_bytes,43712);EXPECT_EQ(added.archive.frame_capacity,101);
    EXPECT_EQ(added.archive.static_declared_bytes,legacy.static_declared_bytes+MetadataByteCap);
    EXPECT_EQ(added.archive.frame_bytes,legacy.frame_bytes+43712+MetadataByteCap);
    EXPECT_EQ(added.archive.forecast_bytes,legacy.forecast_bytes+101*(43712+MetadataByteCap));
    EXPECT_EQ(added.archive.forecast_files,legacy.forecast_files+2*101+1);
    EXPECT_EQ(added.archive.frame_epochs,legacy.frame_epochs);
    EXPECT_LT(added.archive.forecast_bytes,TotalByteCap);
    auto cap=request;cap.total_byte_cap=legacy.forecast_bytes;
    EXPECT_NO_THROW(PlanArchive(cap));EXPECT_THROW(PlanWithActivity(c,cap,"parent-activity.json"),std::runtime_error);
}
TEST(ParentActivity, OptionalProfileRejectsMissingReserveCollisionAndUnrelatedExtras) {
    const auto c=Sized(349645,359785);const auto original=FullRequest(c);
    for(unsigned fault=0;fault<5;++fault) {
        auto r=original;std::string name="parent-activity.json";
        if(fault==0)r.extra_frame_bytes=1;if(fault==1)name="manifest.json";
        if(fault==2)r.parents--;if(fault==3)r.fixed_dt*=2;
        if(fault==4)r.static_byte_reserve=PlanArchive(r).static_declared_bytes+r.frames*FrameMetadataByteCap;
        EXPECT_THROW(PlanWithActivity(c,r,name),std::runtime_error)<<fault;
    }
}
TEST(ParentActivity, SeparateMetadataAndValueFilesUseIndependentFileCaps) {
    const auto c=base::MakeContext();PlanRequest r;
    r.nodes=c.nodes();r.parents=c.parents().size();r.plastic_points=c.points();
    r.frames=2;r.intervals=2;r.fixed_dt=.125;r.requested_duration=.25;
    r.file_byte_cap=MetadataByteCap;r.static_byte_reserve=1024*1024;
    r.static_files={{"manifest.json",MetadataByteCap},{"configuration.json",MetadataByteCap},{"frame-index.json",MetadataByteCap}};
    const auto plan=PlanWithActivity(c,r,"parent-activity.json");
    EXPECT_EQ(plan.packed_frame_bytes,8);EXPECT_EQ(plan.frame_metadata_bytes,MetadataByteCap);
    EXPECT_EQ(plan.archive.frame_capacity,3);
}
TEST(ParentActivity, TwentyMillisecondVehicleForecastKeepsIntegrationStepAndCompleteLedger) {
    // Forecast only: neither this step nor this duration is vehicle dynamics
    // admission. Reduce saved frames to fit storage, never change integration h.
    const auto c=Sized(349645,359785,0x1p-26);
    auto request=FullRequest(c);
    request.frames=88;request.requested_duration=.020;request.intervals=1342178;
    const auto plan=PlanWithActivity(c,request,"parent-activity.json");
    EXPECT_EQ(plan.archive.frame_capacity,89);
    RecordProperty("forecast_bytes",std::to_string(plan.archive.forecast_bytes));
    RecordProperty("forecast_files",std::to_string(plan.archive.forecast_files));
    RecordProperty("frame_bytes",std::to_string(plan.archive.frame_bytes));
    RecordProperty("interval_bytes",std::to_string(plan.archive.interval_bytes));
    EXPECT_EQ(plan.archive.frame_epochs.front(),0);
    EXPECT_EQ(plan.archive.frame_epochs.back(),request.intervals);
    EXPECT_EQ(plan.archive.position_bytes,359785*3*8);
    EXPECT_EQ(plan.archive.plastic_bytes,349645*3*8);
    EXPECT_EQ(plan.archive.interval_bytes,request.intervals*IntervalCoreBytes);
    EXPECT_LT(plan.archive.forecast_bytes,TotalByteCap);
    // A single additional saved frame exceeds this conservative all-NIP3
    // profile, including activity and a separate last-accepted prefix reserve.
    ++request.frames;
    EXPECT_THROW(PlanWithActivity(c,request,"parent-activity.json"),std::runtime_error);
}
} // namespace crash::output::full_shell::activity::test
