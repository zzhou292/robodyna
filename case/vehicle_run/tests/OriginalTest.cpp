#include "../Run.h"
#include "OriginalFixture.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
#include "../source/OriginalYaris.h"
#include "case/CanonicalWallArtifacts.h"
#include "output/full_shell/tests/TestSupport.h"
#include <cstdlib>
#include <iomanip>
#include <sstream>
namespace crash::cases::vehicle_run::test {
TEST(VehicleRunOriginal, ForecastRetainsCompleteSourceAndRejectsStaleRunIdentityBeforeOwner) {
    const auto source=Source();
    const auto& setup=source.setup;
    const auto plan=PreparedRun::Prepare(setup,source.joints,{},Identity());
    EXPECT_EQ(plan.horizon().intervals,16667u);
    EXPECT_LE(plan.forecast().complete_host_bytes,20ull*1000*1000*1000);
    EXPECT_LE(plan.forecast().complete_archive_bytes,2ull<<30);
    EXPECT_FALSE(plan.forecast().caps.expanded);
    EXPECT_EQ(plan.forecast().joint_count,38u);
    Config wrong_profile;
    wrong_profile.physical_profile=PhysicalProfile::ExtendedSolidsV4;
    EXPECT_THROW(PreparedRun::Prepare(setup,source.joints,wrong_profile,Identity()),std::exception);
    auto wrong=Identity();
    wrong.source_instance=source.setup.execution().physical().domain()->source_instance_id()+1;
    EXPECT_THROW(PreparedRun::Prepare(setup,source.joints,{},wrong),std::exception);
    ::testing::Test::RecordProperty("complete_host_upper_bound",std::to_string(plan.forecast().complete_host_bytes));
    ::testing::Test::RecordProperty("complete_archive_upper_bound",std::to_string(plan.forecast().complete_archive_bytes));
    ::testing::Test::RecordProperty("device_bytes",std::to_string(plan.forecast().wall.device_bytes));
}
TEST(VehicleRunExtended, Full4900SourceForecastKeepsOriginalConnectionsAndNormalResourceCaps) {
    const auto source=Source(PhysicalProfile::ExtendedSolidsV4);
    Config config;
    config.physical_profile=PhysicalProfile::ExtendedSolidsV4;
    config.fixed_dt_s=2e-7;
    const auto plan=PreparedRun::Prepare(source.setup,source.joints,config,Identity());
    const auto& model=source.setup.execution().model();
    EXPECT_EQ(model.source_domain().source().solid_source().data().rows.size(),4900u);
    EXPECT_EQ(model.solids().solid18_law44().size(),306u);
    EXPECT_EQ(model.solids().solid18_law90().size(),1345u);
    EXPECT_EQ(plan.forecast().joint_count,40u);
    EXPECT_FALSE(plan.forecast().caps.expanded);
    EXPECT_LE(plan.forecast().complete_host_bytes,20ull*1000*1000*1000);
    EXPECT_LE(plan.forecast().complete_archive_bytes,2ull<<30);
    EXPECT_THROW(PreparedRun::Prepare(source.setup,source.joints,{},Identity()),std::exception);
    ::testing::Test::RecordProperty("physical_nodes",model.source_domain().domain().node_count());
    ::testing::Test::RecordProperty("complete_host_upper_bound",std::to_string(plan.forecast().complete_host_bytes));
    ::testing::Test::RecordProperty("complete_archive_upper_bound",std::to_string(plan.forecast().complete_archive_bytes));
    ::testing::Test::RecordProperty("device_bytes",std::to_string(plan.forecast().wall.device_bytes));
}
void CheckLoadedPrefix(PhysicalProfile profile) {
    const auto source=Source(profile);
    Config config;
    config.physical_profile=profile;
    // Complete V4 assembly measures a 2.2934672909685784e-7 s post-CIN
    // startup limit. V5 initially tests the same step with its own actual
    // beam stiffness and post-CIN screen; no V5 bound is assumed here.
    if(profile!=PhysicalProfile::RetainedShellAssembliesV1) config.fixed_dt_s=2e-7;
    const auto plan=PreparedRun::Prepare(source.setup,source.joints,config,Identity());
    records::test::Directory temporary;
    const auto requested=std::getenv("ROBO_VEHICLE_RUN_OUTPUT");
    const auto destination=requested && *requested?std::filesystem::path(requested):temporary.path;
    Control control;
    control.maximum_accepted_intervals=2;
    const auto result=plan.Execute(destination,control);
    ASSERT_TRUE(result.session_initialized)<<result.loop.reason;
    ASSERT_EQ(result.loop.kind,StopKind::IntervalLimit)<<result.loop.reason;
    ASSERT_TRUE(result.loop.valid_manifest)<<result.loop.reason;
    ASSERT_TRUE(result.archive_manifest);
    ASSERT_TRUE(result.viewer_input)<<result.viewer_input_error;
    ASSERT_TRUE(result.summary)<<result.summary_error;
    EXPECT_EQ(result.loop.progress.accepted.epoch,2u);
    EXPECT_EQ(result.loop.progress.accepted.time_s,2*config.fixed_dt_s);
    EXPECT_TRUE(result.loop.progress.contact.available);
    EXPECT_GT(result.loop.progress.contact.peak_observed_force_n,0);
    EXPECT_GT(result.loop.progress.contact.peak_observed_penetration_m,0);
    EXPECT_EQ(result.loop.progress.contact.intervals,2u);
    const auto descriptor=output::physical_run::ReadViewerInput(destination,*result.viewer_input);
    const auto replay=output::physical_run::Replay::Open(
        output::physical_run::ViewerArchivePath(destination,descriptor),descriptor.manifest,
        descriptor.source,descriptor.mapping_sha256);
    EXPECT_EQ(replay.context().parents().size(),349645u);
    EXPECT_EQ(replay.context().nodes(),359785u);
    EXPECT_TRUE(replay.configuration().profile.type45);
    EXPECT_TRUE(replay.wall());
    ASSERT_TRUE(replay.wall_composition());
    const auto& composition=*replay.wall_composition();
    const bool supports=profile==PhysicalProfile::VehicleSupportsV5;
    const bool extended=profile!=PhysicalProfile::RetainedShellAssembliesV1;
    EXPECT_EQ(composition.profile,supports ? output::physical_run::CompositionProfile::VehicleSupportsV5
        : extended ? output::physical_run::CompositionProfile::ExtendedSolidsV4
        : output::physical_run::CompositionProfile::RetainedV1);
    EXPECT_EQ(composition.solid_parents,(supports ? std::array<std::uint64_t,5>{908,1991,350,386,1345}
        : extended ? std::array<std::uint64_t,5>{908,1991,350,306,1345}
        : std::array<std::uint64_t,5>{908,1309,195,0,0}));
    EXPECT_EQ(composition.physical_nodes,source.setup.execution().model().source_domain().domain().node_count());
    EXPECT_EQ(composition.point_mass_records,supports ? 154u : extended ? 150u : 148u);
    EXPECT_EQ(replay.configuration().profile.beam18,supports);
    EXPECT_EQ(composition.structural_beam_parents,supports ? 142u : 0u);
    EXPECT_EQ(composition.structural_beam_parts,supports ? 4u : 0u);
    if(supports) EXPECT_EQ(composition.solid_parts,17u);
    EXPECT_EQ(output::Bits(composition.initial_mass_kg),
        output::Bits(source.setup.execution().model().coefficients().totals().mass));
    EXPECT_EQ(replay.index().accepted_intervals,2u);
    EXPECT_FALSE(replay.index().horizon_complete);
    ASSERT_EQ(replay.index().frames.size(),2u);
    const auto saved=replay.ReadSample(1);
    EXPECT_EQ(saved.frame.stamp.epoch,2u);
    EXPECT_EQ(saved.frame.stamp.time,result.loop.progress.accepted.time_s);
    EXPECT_EQ(saved.activity.stamp().epoch,2u);
    ::testing::Test::RecordProperty("accepted_intervals",std::to_string(result.loop.progress.accepted.epoch));
    ::testing::Test::RecordProperty("archive_manifest_sha256",result.archive_manifest->sha256);
    ::testing::Test::RecordProperty("viewer_input_sha256",result.viewer_input->sha256);
    std::ostringstream completed;
    completed<<std::setprecision(17)<<result.loop.progress.accepted.time_s;
    ::testing::Test::RecordProperty("completed_seconds",completed.str());
    ::testing::Test::RecordProperty("scope","two accepted loaded intervals through actual controller; no complete5ms claim");
}
TEST(VehicleRunOriginal, TwoActualLoadedIntervalsExportAuthenticAcceptedPrefixAndViewerInput) {
    CheckLoadedPrefix(PhysicalProfile::RetainedShellAssembliesV1);
}
TEST(VehicleRunExtended, TwoActualLoadedIntervalsKeep4900SolidOwnerAndAuthenticReplay) {
    CheckLoadedPrefix(PhysicalProfile::ExtendedSolidsV4);
}
} // namespace crash::cases::vehicle_run::test
