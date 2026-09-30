#include "../Run.h"
#include "OriginalFixture.h"
#include "case/vehicle_runtime/VehiclePhysicalStartup.h"
#include "case/vehicle_runtime/CaptureAccess.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
namespace crash::cases::vehicle_run::test {
TEST(VehicleRunSupports, FullSupportForecastUsesActualBeamModelAndInclusiveCaps) {
    const auto source=Source(PhysicalProfile::VehicleSupportsV5);
    const auto& setup=source.setup;
    const auto& model=setup.execution().model();
    ASSERT_TRUE(model.structural_beams());
    EXPECT_EQ(model.structural_beams()->parents().size(),142u);
    EXPECT_EQ(model.source_domain().domain().node_count(),376930u);
    EXPECT_EQ(model.source_domain().source().solid_source().data().rows.size(),4980u);
    EXPECT_EQ(model.solids().solid18_law44().size(),386u);
    EXPECT_EQ(model.solids().solid18_law90().size(),1345u);
    Config config;
    config.physical_profile=PhysicalProfile::VehicleSupportsV5;
    config.fixed_dt_s=2e-7;
    const auto plan=PreparedRun::Prepare(setup,source.joints,config,Identity());
    EXPECT_EQ(plan.forecast().joint_count,44u);
    EXPECT_FALSE(plan.forecast().caps.expanded);
    EXPECT_LE(plan.forecast().complete_host_bytes,20ull*1000*1000*1000);
    EXPECT_LE(plan.forecast().complete_archive_bytes,2ull<<30);
    EXPECT_THROW(PreparedRun::Prepare(setup,source.joints,{},Identity()),std::exception);
    config.physical_profile=PhysicalProfile::ExtendedSolidsV4;
    EXPECT_THROW(PreparedRun::Prepare(setup,source.joints,config,Identity()),std::exception);

    namespace vr=vehicle_runtime;
    vr::Config exact;
    exact.reserved_step_s=2e-7;
    const auto forecast=vr::VehiclePhysicalStartup::Preflight(setup.execution(),setup.attachments(),exact,&source.joints);
    EXPECT_TRUE(forecast.has_beam18);
    EXPECT_GT(forecast.structural_beams.device_bytes,0u);
    EXPECT_GT(forecast.structural_beam_incremental_host_bytes,0u);
    exact.limits.host_bytes=forecast.peak_host_upper_bound;
    exact.limits.device_bytes=forecast.device_bytes;
    EXPECT_EQ(vr::VehiclePhysicalStartup::Preflight(setup.execution(),setup.attachments(),exact,&source.joints).device_bytes,
        forecast.device_bytes);
    auto short_cap=exact;
    --short_cap.limits.host_bytes;
    EXPECT_THROW(vr::VehiclePhysicalStartup::Prepare(setup.execution(),setup.attachments(),short_cap,&source.joints),std::exception);
    short_cap=exact; --short_cap.limits.device_bytes;
    EXPECT_THROW(vr::VehiclePhysicalStartup::Prepare(setup.execution(),setup.attachments(),short_cap,&source.joints),std::exception);
    short_cap=exact; short_cap.limits.structural_beams.max_parents=141;
    EXPECT_THROW(vr::VehiclePhysicalStartup::Preflight(setup.execution(),setup.attachments(),short_cap,&source.joints),std::exception);
    RecordProperty("runtime_host_upper_bound",std::to_string(forecast.peak_host_upper_bound));
    RecordProperty("runtime_device_bytes",std::to_string(forecast.device_bytes));
    RecordProperty("beam_device_bytes",std::to_string(forecast.structural_beams.device_bytes));
    RecordProperty("complete_archive_bytes",std::to_string(plan.forecast().complete_archive_bytes));
}
TEST(VehicleRunSupports, CompleteInitialOwnerReadsEightParticipantsAndPreservesFailedReplacement) {
    const auto source=Source(PhysicalProfile::VehicleSupportsV5);
    const auto& setup=source.setup;
    namespace vr=vehicle_runtime;
    vr::Config exact;
    exact.reserved_step_s=2e-7;
    const auto forecast=vr::VehiclePhysicalStartup::Preflight(setup.execution(),setup.attachments(),exact,&source.joints);
    exact.limits.host_bytes=forecast.peak_host_upper_bound;
    exact.limits.device_bytes=forecast.device_bytes;
    auto owner=vr::VehiclePhysicalStartup::Prepare(setup.execution(),setup.attachments(),exact,&source.joints);
    const auto before=owner.accepted();
    const auto view=owner.InspectInitial();
    EXPECT_EQ(view.nodes,376930u);
    EXPECT_EQ(view.shell_parents,349645u);
    EXPECT_EQ(view.material_points,1037877u);
    EXPECT_EQ(view.rigid_skins,5102u);
    EXPECT_EQ(view.solid_parents,4980u);
    EXPECT_EQ(view.structural_beams,142u);
    EXPECT_EQ(view.type45_joints,44u);
    EXPECT_EQ(view.type13_connections,4442u);
    EXPECT_EQ(view.type25_connections,2828u);
    EXPECT_GT(view.absent_rotations,0u);
    EXPECT_EQ(view.allocations.device_bytes,forecast.device_bytes);
    EXPECT_EQ(view.stamp.epoch,0u);
    const auto capture=vr::detail::CaptureAccess::Scope(owner);
    EXPECT_TRUE(capture.diagnostics.has_beam18);
    EXPECT_TRUE(capture.diagnostics.has_type45);
    EXPECT_EQ(capture.beam18_parent_count,142u);
    EXPECT_EQ(capture.beam18_source_instance_id,setup.execution().model().structural_beams()->source_instance_id());
    EXPECT_FALSE(capture.diagnostics.beam18.has_completed_interval);
    EXPECT_FALSE(capture.diagnostics.type45.automatic_stiffness_initialized);
    --exact.limits.device_bytes;
    EXPECT_THROW(owner=vr::VehiclePhysicalStartup::Prepare(setup.execution(),setup.attachments(),exact,&source.joints),std::exception);
    EXPECT_EQ(owner.accepted().owner_id,before.owner_id);
    EXPECT_EQ(owner.accepted().epoch,0u);
    const auto repeated=owner.InspectInitial();
    EXPECT_EQ(repeated.structural_beams,view.structural_beams);
    EXPECT_EQ(repeated.allocations.device_allocations,view.allocations.device_allocations);
    RecordProperty("explicit_device_bytes",std::to_string(view.allocations.device_bytes));
    RecordProperty("explicit_device_allocations",std::to_string(view.allocations.device_allocations));
}
TEST(VehicleRunSupports, TwoLoadedIntervalsKeepCompleteSupportsAndAuthenticatedReplay) {
    CheckLoadedPrefix(PhysicalProfile::VehicleSupportsV5);
}
} // namespace crash::cases::vehicle_run::test
