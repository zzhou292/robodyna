#include "TestSupport.h"
#include "output/full_shell/FullShellVisualizationPlan.h"
#include <limits>

namespace crash::output::full_shell::test {
PlanRequest Request(double h,std::uint64_t steps) {
    PlanRequest r;r.nodes=359785;r.parents=349645;r.plastic_points=3*r.parents;r.fixed_dt=h;
    r.intervals=steps;r.requested_duration=.02;
    r.static_files={{"manifest.json",1024*1024},{"frame-index.json",256*1024},{"configuration.json",8*1024*1024},
        {"source-member-0.bin",33554432},{"source-member-1.bin",9292321},{"scope.json",13175122}};
    return r;
}
TEST(FullShellPlan,ActualCountsExactForecastAndLowerDtRejects) {
    for(auto item:{std::pair<double,std::uint64_t>{0x1p-23,167773},{0x1p-24,335545}}) {
        const auto r=Request(item.first,item.second);const auto p=PlanArchive(r);
        EXPECT_EQ(p.position_bytes,8634840u);EXPECT_EQ(p.plastic_bytes,8391480u);
        EXPECT_EQ(p.frame_capacity,101u);EXPECT_EQ(p.interval_bytes,312*r.intervals);
        EXPECT_EQ(p.forecast_bytes,1719658320u+201326592u+312*r.intervals);
        ASSERT_EQ(p.frame_epochs.size(),100u);EXPECT_EQ(p.frame_epochs.front(),0u);EXPECT_EQ(p.frame_epochs.back(),r.intervals);
        for(std::size_t i=1;i<p.frame_epochs.size();++i)EXPECT_GT(p.frame_epochs[i],p.frame_epochs[i-1]);
        EXPECT_LE(p.rows_per_chunk*p.interval_row_bytes,kArtifactFileCap);
        auto exact=r;exact.total_byte_cap=p.forecast_bytes;EXPECT_NO_THROW(PlanArchive(exact));
        --exact.total_byte_cap;EXPECT_THROW(PlanArchive(exact),std::exception);
    }
    EXPECT_THROW(PlanArchive(Request(0x1p-26,1342178)),std::exception);
    // Explicit lower output cadence preserves every native point and interval.
    // The planner neither increases h nor silently changes the requested profile.
    auto lower_cadence=Request(0x1p-26,1342178);lower_cadence.frames=88;
    const auto smaller=PlanArchive(lower_cadence);
    EXPECT_EQ(smaller.frame_capacity,89u);EXPECT_EQ(smaller.forecast_bytes,2135428608u);
    EXPECT_EQ(TotalByteCap-smaller.forecast_bytes,12055040u);
}
TEST(FullShellPlan,ChecksDeclaredFilesOptionalChannelsAndRealTimestep) {
    const auto good=Request(0x1p-24,335545);auto r=good;
    --r.intervals;EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.fixed_dt*=.5;EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.static_files.back().bytes=kArtifactFileCap+1;EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.static_files.push_back(r.static_files.back());EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.static_files.front().file="other.json";EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.static_byte_reserve=64*1024*1024;EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.file_byte_cap=1;EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.extra_interval_bytes=400;EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.extra_frame_bytes=1024;EXPECT_EQ(PlanArchive(r).forecast_bytes,PlanArchive(good).forecast_bytes+101*1024);
    r=good;r.frames=150;EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.plastic_points=4194305;EXPECT_THROW(PlanArchive(r),std::exception);
    r=good;r.intervals=2;r.frames=2;
    r.fixed_dt=.75*std::numeric_limits<double>::max();r.requested_duration=std::numeric_limits<double>::max();
    EXPECT_THROW(PlanArchive(r),std::exception);
}
} // namespace crash::output::full_shell::test
