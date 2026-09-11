#include "Support.h"
namespace crash::output::physical_run::test {
TEST(PhysicalRunValues, ProfileOmitsUnavailableColumnsAndRejectsInventedAvailability) {
    for(bool joint:{false,true})for(bool bound:{false,true}) {
        const Profile p{joint,bound};const auto doc=ProfileDocument(p);
        EXPECT_TRUE(SameProfile(ReadProfile(doc),p));
        EXPECT_EQ(RealFields(p).size(),bound?5u:4u);
        EXPECT_EQ(array_json::Text(doc["kinetic_energy"]),"unavailable");
        EXPECT_EQ(array_json::Text(doc["joint_work"]),"unavailable");
    }
    auto bad=ProfileDocument({});bad["kinetic_energy"].SetString("native_J",bad.GetAllocator());
    EXPECT_THROW(ReadProfile(bad),std::exception);
    const auto c=Context();auto row=Row(c,1);
    EXPECT_THROW(CheckValues(c,{},row),std::exception);
    row.structural_limit_s=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(CheckValues(c,{false,true},row),std::exception);
    row.structural_limit_s=.1;EXPECT_THROW(CheckValues(c,{false,true},row),std::exception);
}
TEST(PhysicalRunValues, AcceptedContiguityAllowsRejectedAttemptsButNeverGapsOrTrialTimes) {
    const auto c=Context();const Profile p{false,true};Sequence s;
    s=Advance(c,p,4,s,Row(c,1,3));const auto previous=s;
    auto bad=Row(c,3,9);EXPECT_THROW(Advance(c,p,4,s,bad),std::exception);
    bad=Row(c,2,3);EXPECT_THROW(Advance(c,p,4,s,bad),std::exception);
    bad=Row(c,2,8);bad.stamp.time=std::nextafter(bad.stamp.time,1.);
    EXPECT_THROW(Advance(c,p,4,s,bad),std::exception);
    bad=Row(c,2,8);bad.owner=991;EXPECT_THROW(Advance(c,p,4,s,bad),std::exception);
    EXPECT_TRUE(records::SameStamp(previous.last,s.last));
    s=Advance(c,p,4,s,Row(c,2,8));EXPECT_EQ(s.last.epoch,2u);EXPECT_EQ(s.last.attempt,8u);
}
TEST(PhysicalRunValues, DefaultTwoGiBAndExplicitSixGiBUse64BitWholeRunForecast) {
    auto c=Context();auto config=Config(c);EXPECT_EQ(config.request.total_byte_cap,UINT64_C(2147483648));
    auto r=config.request;r.nodes=359785;r.parents=349645;r.plastic_points=1037877;
    r.intervals=1000000;r.fixed_dt=0x1p-24;r.requested_duration=r.intervals*r.fixed_dt;r.frames=300;
    r.static_byte_reserve=records::StaticReserveBytes;
    EXPECT_THROW(records::PlanArchive(r),std::exception);
    r.total_byte_cap=records::FullRunByteCap;
    const auto plan=records::PlanArchive(r);
    EXPECT_GT(plan.forecast_bytes,UINT64_C(4294967295));EXPECT_LE(plan.forecast_bytes,UINT64_C(6442450944));
    config.request=r;
    const auto metadata=ConfigurationDocument(config);
    EXPECT_EQ(ReadConfiguration(metadata).request.total_byte_cap,UINT64_C(6442450944));
    r.total_byte_cap=plan.forecast_bytes;EXPECT_EQ(records::PlanArchive(r).forecast_bytes,plan.forecast_bytes);
    --r.total_byte_cap;EXPECT_THROW(records::PlanArchive(r),std::exception);
    r.total_byte_cap=records::FullRunByteCap+1;EXPECT_THROW(records::PlanArchive(r),std::exception);
    EXPECT_THROW(interval::PlanChunks(UINT64_MAX,kArtifactFileCap,records::FullRunByteCap),std::exception);
    EXPECT_THROW(interval::PlanChunks(UINT64_C(10000000),kArtifactFileCap,records::FullRunByteCap),std::exception);
}
} // namespace crash::output::physical_run::test
