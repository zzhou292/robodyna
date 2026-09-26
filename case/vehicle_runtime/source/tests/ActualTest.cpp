#include "ActualFixture.h"
#include "../../ParticipantConfigs.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_runtime::source_test {
TEST(EnvelopeRuntimeActual, CompleteSourceForecastBeforeOwnerAllocation) {
    const auto source=Source::WithEnvironment(OwnerSource());
    const auto forecast=VehiclePhysicalStartup::Preflight(source,RuntimeConfig());
    const auto config=detail::ConfigureParticipants(RuntimeConfig(),source,detail::DescriptiveStamp(RuntimeConfig(),source));
    EXPECT_EQ(source.kind(),SourceKind::VehicleWithEnvironment);
    EXPECT_EQ(source.vehicle_nodes(),376930u);
    EXPECT_EQ(source.physical().domain()->node_count(),376934u);
    EXPECT_EQ(config.qeph.startup.kind,source.startup().kind);
    EXPECT_EQ(config.solids.startup.kind,source.startup().kind);
    EXPECT_EQ(config.publication.startup.kind,source.startup().kind);
    EXPECT_TRUE(forecast.has_type45);EXPECT_TRUE(forecast.has_beam18);
    EXPECT_THROW(source.original_execution(),std::exception);
    EXPECT_THROW(source.original_attachments(),std::exception);
    auto roles=source.roles();const auto wall=OwnerSource().execution_source().mechanical().environment_parent().domain_nodes[0];
    roles.node[wall]^=Shell;
    EXPECT_THROW(source.PackOwner(roles,forecast.packing_bytes),std::exception);
    auto short_cap=RuntimeConfig();short_cap.limits.device_bytes=forecast.device_bytes-1;
    EXPECT_THROW(VehiclePhysicalStartup::Preflight(source,short_cap),std::exception);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.combined_runtime_forecast.v1");
    ForecastFields(doc,forecast);output::WriteJson(Destination()/"forecast.json",doc);
}
TEST(EnvelopeRuntimeActual, OneActualOwnerRetainsVehicleAndFixedWallWithAllParticipants) {
    const auto source=Source::WithEnvironment(OwnerSource());
    const auto forecast=VehiclePhysicalStartup::Preflight(source,RuntimeConfig());
    ASSERT_LE(Qualification(forecast),GuardBytes);
    auto config=RuntimeConfig();config.limits.host_bytes=forecast.peak_host_upper_bound;config.limits.device_bytes=forecast.device_bytes;
    auto owner=VehiclePhysicalStartup::Prepare(source,config);
    EXPECT_EQ(owner.source().kind(),SourceKind::VehicleWithEnvironment);
    EXPECT_EQ(owner.allocations().device_bytes,forecast.device_bytes);
    const auto before=owner.accepted();const auto initial=owner.InspectInitial();
    EXPECT_EQ(initial.nodes,376934u);EXPECT_EQ(initial.shell_parents,349646u);
    EXPECT_EQ(initial.zero_point_parents,27178u);EXPECT_EQ(initial.rigid_skins,5102u);
    EXPECT_EQ(initial.cin_secondaries,11165u);EXPECT_EQ(initial.type45_joints,44u);
    EXPECT_EQ(initial.solid_parents,4980u);EXPECT_GT(initial.structural_beams,0u);
    EXPECT_EQ(owner.accepted().owner_id,before.owner_id);EXPECT_EQ(owner.accepted().epoch,0u);
    EXPECT_EQ(owner.accepted().time,0.);EXPECT_EQ(owner.accepted().fixed_dt,2e-7);
    auto short_cap=config;--short_cap.limits.device_bytes;
    EXPECT_THROW(owner=VehiclePhysicalStartup::Prepare(source,short_cap),std::exception);
    EXPECT_EQ(owner.accepted().owner_id,before.owner_id);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.combined_runtime_initial.v1");
    ForecastFields(doc,forecast);output::Integer(doc,"nodes",initial.nodes);
    output::Integer(doc,"shell_parents",initial.shell_parents);output::Integer(doc,"zero_point_parents",initial.zero_point_parents);
    output::Integer(doc,"cin_rows",initial.cin_secondaries);output::Integer(doc,"joints",initial.type45_joints);
    output::Integer(doc,"solid_parents",initial.solid_parents);output::Integer(doc,"structural_beams",initial.structural_beams);
    output::Integer(doc,"owner_id",before.owner_id);output::Integer(doc,"epoch",owner.accepted().epoch);
    output::WriteJson(Destination()/"initial.json",doc);
}
TEST(EnvelopeRuntimeActual, ExistingDynamicsAdvancesOneClockAndDiscardsPreparedAttempt) {
    const auto source=Source::WithEnvironment(OwnerSource());const auto config=DynamicsConfig();
    const auto forecast=vehicle_dynamics::VehiclePhysicalDynamics::Preflight(source,config);
    ASSERT_LE(Add(forecast.peak_host_upper_bound,ExportBytes),GuardBytes);
    auto run=vehicle_dynamics::VehiclePhysicalDynamics::Prepare(source,config);
    const auto initial=run.accepted();
    const auto first=run.PrepareStep();
    EXPECT_TRUE(first.motion_includes_fixed_environment);EXPECT_EQ(run.accepted().epoch,0u);
    EXPECT_TRUE(first.structural_limiter.values.kind!=tl::fea::NodalCinLimitKind::Unavailable);
    run.DiscardStep();EXPECT_EQ(run.accepted().epoch,0u);EXPECT_EQ(run.accepted().owner_id,initial.owner_id);
    for(unsigned step=0;step<2;++step) {
        const auto& trial=run.PrepareStep();EXPECT_EQ(trial.base.epoch,step);
        run.CommitStep();EXPECT_EQ(run.accepted().epoch,step+1);
    }
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.combined_runtime_structural_smoke.v1");
    output::Integer(doc,"owner_id",run.accepted().owner_id);output::Integer(doc,"accepted_steps",run.accepted().epoch);
    output::Number(doc,"accepted_time_s",run.accepted().time);output::Number(doc,"fixed_dt_s",run.accepted().fixed_dt);
    output::Boolean(doc,"contact_enabled",false);output::WriteJson(Destination()/"steps.json",doc);
}
}
