#include "../VehicleAcceptedFrames.h"
#include "output/physical_frames/Archive.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include "output/full_shell/tests/TestSupport.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace crash::cases::vehicle_dynamics::capture::test {
namespace {
namespace records=crash::output::full_shell;
namespace physical=crash::output::physical_frames;
const vehicle_runtime::Attachments& Source() {
    return vehicle_startup::physical_attachments::test::Actual();
}
const vehicle_runtime::Execution& Shells() {
    static const auto value=vehicle_runtime::Execution::Prepare(Source().physical());return value;
}
void SameFrame(const records::FrameRecord& a,const records::FrameRecord& b) {
    ASSERT_TRUE(records::SameStamp(a.stamp,b.stamp));
    ASSERT_EQ(a.position_xyz,b.position_xyz);
    ASSERT_EQ(a.plastic_points,b.plastic_points);
}
}
TEST(VehicleDynamicsCaptureOriginal, AcceptedEpochReadbackDiscardAndBinaryRoundTrip) {
    auto run=VehiclePhysicalDynamics::Prepare(Shells(),Source());
    const auto map=physical::Mapping::Prepare(Shells());
    records::Identity id;id.run=0x44594e434150ULL;id.topology=0x5941524953ULL;
    VehicleAcceptedFrames capture(map,run,id);
    capture.Capture(run);
    const auto initial=*capture.frames().frame();
    const auto initial_stamp=run.accepted();
    const auto allocations=run.allocations();
    ASSERT_EQ(initial.stamp.epoch,0u);
    const auto observation=run.PrepareStep();
    capture.Capture(run);
    ASSERT_NO_FATAL_FAILURE(SameFrame(initial,*capture.frames().frame()));
    run.DiscardStep();
    capture.Capture(run);
    ASSERT_NO_FATAL_FAILURE(SameFrame(initial,*capture.frames().frame()));
    ASSERT_TRUE(tl::fea::trial_identity::SameStamp(initial_stamp,run.accepted()));
    const auto retry=run.PrepareStep();
    ASSERT_EQ(observation.proposed_time,retry.proposed_time);
    run.CommitStep();
    capture.Capture(run);
    const auto& producer=capture.frames();
    const auto& frame=*producer.frame();
    ASSERT_EQ(frame.stamp.epoch,1u);
    ASSERT_EQ(frame.stamp.time,run.accepted().time);
    ASSERT_EQ(producer.activity()->stamp().epoch,1u);
    ASSERT_EQ(producer.activity()->active_count(),349645u);
    ASSERT_EQ(frame.plastic_points.size(),956346u);
    const double displacement=15.6464*frame.stamp.time;
    for(std::size_t n=0;n<map.physical_nodes().size();++n) {
        ASSERT_NEAR(frame.position_xyz[3*n]-initial.position_xyz[3*n],displacement,2e-15);
        ASSERT_NEAR(frame.position_xyz[3*n+1],initial.position_xyz[3*n+1],2e-15);
        ASSERT_NEAR(frame.position_xyz[3*n+2],initial.position_xyz[3*n+2],2e-15);
    }
    for(double p:frame.plastic_points) ASSERT_EQ(p,0);
    records::source::BundleRequest request;
    auto& plan=request.archive;
    plan.nodes=producer.context().nodes();plan.parents=producer.context().parents().size();
    plan.plastic_points=producer.context().points();plan.frames=2;plan.intervals=1;
    plan.fixed_dt=producer.context().fixed_dt();plan.requested_duration=plan.fixed_dt;
    plan.static_files={{"manifest.json",16384},{"frame-index.json",16384},
                       {"configuration.json",16384},{"wall.json",16384}};
    auto archive=physical::Archive::Prepare(map.source_mapping(),producer.context(),request);
    records::test::Directory directory;
    const auto files=archive.Write(directory.path,"accepted",frame,*producer.activity());
    const auto restored=records::ReadFrame(directory.path,producer.context(),files.frame,frame.stamp);
    const auto active=records::activity::ReadActivity(directory.path,producer.context(),files.activity,frame.stamp);
    ASSERT_NO_FATAL_FAILURE(SameFrame(restored,frame));
    ASSERT_EQ(active.words(),producer.activity()->words());
    ASSERT_EQ(run.allocations().device_bytes,allocations.device_bytes);
    ASSERT_EQ(run.allocations().device_allocations,allocations.device_allocations);
    RecordProperty("accepted_epoch",frame.stamp.epoch);
    RecordProperty("captured_shell_nodes",producer.context().nodes());
    RecordProperty("captured_native_plastic_values",producer.context().points());
    RecordProperty("capture_peak_bytes",std::to_string(producer.forecast().peak_bytes));
}
} // namespace crash::cases::vehicle_dynamics::capture::test
