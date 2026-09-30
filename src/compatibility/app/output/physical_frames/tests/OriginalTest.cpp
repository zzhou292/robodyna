#include "../PhysicalAcceptedFrames.h"
#include "../Archive.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include "output/full_shell/tests/TestSupport.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace crash::output::physical_frames::test {
namespace {
const cases::vehicle_runtime::Attachments& Source() {
    return cases::vehicle_startup::physical_attachments::test::Actual();
}
const Execution& Shells() {
    static const auto source=Execution::Prepare(Source().physical());return source;
}
const Mapping& Map() {static const auto value=Mapping::Prepare(Shells());return value;}
records::Identity Identity() {
    records::Identity id;id.run=0x43415054555245ULL;id.topology=0x5941524953ULL;return id;
}
source::BundleRequest Request(const records::Context& c) {
    source::BundleRequest request;
    auto& plan=request.archive;
    plan.nodes=c.nodes();plan.parents=c.parents().size();plan.plastic_points=c.points();
    // Capacity forecast only: the gate emits one actual initial frame and no
    // interval record or run-completion marker for this reserved future step.
    plan.frames=2;plan.intervals=1;plan.fixed_dt=c.fixed_dt();plan.requested_duration=c.fixed_dt();
    plan.static_files={{"manifest.json",16384},{"frame-index.json",16384},{"configuration.json",16384},
                       {"wall.json",16384}};
    return request;
}
}
TEST(PhysicalCaptureOriginal, MappingAndArchiveForecast) {
    auto id=Identity();id.owner=1;id.source_instance=Shells().physical().domain()->source_instance_id();
    id.configuration=cases::vehicle_runtime::Config{}.configuration_id;
    id.qualification=cases::vehicle_runtime::Config{}.qualification_id;
    const auto context=Map().source_mapping().MakeFrameContext(id,1e-8);
    EXPECT_EQ(context.nodes(),359785u);EXPECT_EQ(context.parents().size(),349645u);
    EXPECT_EQ(context.points(),956346u);
    std::size_t points=0,skin=0,one=0,four=0;
    for(const auto& p:context.parents()) {
        points+=p.native_points;skin+=p.native_points==0;one+=p.native_points==1;four+=p.native_points==4;
    }
    EXPECT_EQ(points,1037877u);EXPECT_EQ(skin,5102u);EXPECT_EQ(one,1u);EXPECT_EQ(four,4250u);
    const auto forecast=PhysicalAcceptedFrames::Preflight(Map(),context);
    RecordProperty("capture_peak_bytes",std::to_string(forecast.peak_bytes));
    RecordProperty("mapping_payload_bytes",std::to_string(Map().payload_bytes()));
    Limits exact;exact.host_bytes=forecast.peak_bytes;
    EXPECT_EQ(PhysicalAcceptedFrames::Preflight(Map(),context,exact).peak_bytes,forecast.peak_bytes);
    --exact.host_bytes;
    EXPECT_THROW(PhysicalAcceptedFrames::Preflight(Map(),context,exact),std::exception);
    const auto archive=Archive::Prepare(Map().source_mapping(),context,Request(context));
    EXPECT_LE(archive.plan().archive.forecast_bytes,records::TotalByteCap);
    RecordProperty("complete_archive_forecast_bytes",std::to_string(archive.plan().archive.forecast_bytes));
    RecordProperty("archive_startup_bytes",std::to_string(archive.startup_host_bytes()));
}
TEST(PhysicalCaptureOriginal, InitialCompleteSnapshotAndBinaryRoundTrip) {
    auto run=Run::Prepare(Shells(),Source());
    const auto initial=run.accepted();
    PhysicalAcceptedFrames capture(Map(),run,Identity());
    EXPECT_EQ(capture.frame(),nullptr);EXPECT_EQ(capture.activity(),nullptr);
    capture.Capture(run);
    ASSERT_NE(capture.frame(),nullptr);ASSERT_NE(capture.activity(),nullptr);
    const auto& c=capture.context();const auto& frame=*capture.frame();
    EXPECT_EQ(frame.stamp.epoch,0u);EXPECT_EQ(frame.stamp.time,0);
    EXPECT_EQ(capture.activity()->active_count(),349645u);
    EXPECT_EQ(c.points(),956346u);
    const auto& domain=Shells().physical().domain()->nodes();
    for(std::size_t n=0;n<Map().physical_nodes().size();++n) {
        const auto x=domain[Map().physical_nodes()[n]].position;
        ASSERT_EQ(Bits(frame.position_xyz[3*n]),Bits(x.x));
        ASSERT_EQ(Bits(frame.position_xyz[3*n+1]),Bits(x.y));
        ASSERT_EQ(Bits(frame.position_xyz[3*n+2]),Bits(x.z));
    }
    for(double p:frame.plastic_points) ASSERT_EQ(p,0);
    auto archive=Archive::Prepare(Map().source_mapping(),c,Request(c));
    records::test::Directory directory;
    std::filesystem::create_directory(directory.path/"arrays");
    const auto bundle=source::WriteSourceBundle(directory.path,archive.source_bundle());
    const auto restored_mapping=source::ReadSourceBundle(directory.path,bundle,
        Map().source_mapping().source().data().inputs,Map().source_mapping().digest());
    EXPECT_EQ(restored_mapping.digest(),Map().source_mapping().digest());
    const auto restored_context=restored_mapping.MakeFrameContext(c.identity(),c.fixed_dt(),c.limits());
    WriteBytes(directory.path/"initial.activity.json","existing");
    EXPECT_THROW(archive.Write(directory.path,"initial",frame,*capture.activity()),std::exception);
    EXPECT_EQ(archive.written_frames(),0u);
    EXPECT_FALSE(std::filesystem::exists(directory.path/"initial.positions.bin"));
    std::filesystem::remove(directory.path/"initial.activity.json");
    const auto files=archive.Write(directory.path,"initial",frame,*capture.activity());
    const auto restored=records::ReadFrame(directory.path,restored_context,files.frame,frame.stamp);
    const auto active=records::activity::ReadActivity(directory.path,c,files.activity,frame.stamp);
    ASSERT_EQ(restored.position_xyz.size(),frame.position_xyz.size());
    for(std::size_t i=0;i<restored.position_xyz.size();++i)ASSERT_EQ(Bits(restored.position_xyz[i]),Bits(frame.position_xyz[i]));
    ASSERT_EQ(restored.plastic_points,frame.plastic_points);
    ASSERT_EQ(active.words(),capture.activity()->words());
    EXPECT_EQ(archive.written_frames(),1u);
    EXPECT_THROW(archive.Write(directory.path,"duplicate",frame,*capture.activity()),std::exception);
    EXPECT_FALSE(std::filesystem::exists(directory.path/"manifest.json"));
    capture.Capture(run);
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(initial,run.accepted()));
    RecordProperty("captured_epoch",0);RecordProperty("shell_nodes",c.nodes());
    RecordProperty("parents",c.parents().size());RecordProperty("native_plastic_values",c.points());
    RecordProperty("explicit_device_bytes",run.allocations().device_bytes);
}
} // namespace crash::output::physical_frames::test
