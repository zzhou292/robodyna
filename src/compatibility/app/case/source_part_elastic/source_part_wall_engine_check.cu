#include "SourcePartWallEngineTestSupport.h"
#include "SourcePartUniformFlightTestSupport.h"
#include <iomanip>
#include <iostream>

namespace crash::cases::source_part_elastic::test {
namespace {
TEST(SourcePartWallEngine, ExplicitWallStartupRejectsContradictionsWithoutPublishing) {
    const auto f=std::make_unique<wall_test::Fixture>();
    auto config=MeshWallConfig(1); auto settings=MeshWallSettings(config);
    SourcePartElasticCase run;
    EXPECT_EQ(run.Initialize(f->source,config).status,Status::InvalidInput);
    auto bad=settings; bad.initial_velocity[0]=2;
    EXPECT_EQ(run.Initialize(f->source,config,f->wall,f->wall_bytes,bad).status,Status::InvalidInput);
    bad=settings; ++bad.configuration_id;
    EXPECT_EQ(run.Initialize(f->source,config,f->wall,f->wall_bytes,bad).status,Status::InvalidInput);
    EXPECT_EQ(run.Initialize(f->source,UniformFlightConfig(1),f->wall,f->wall_bytes,settings).status,Status::InvalidInput);
    auto invalid=config; invalid.relative_energy_residual=.1; EXPECT_FALSE(ValidConfig(invalid));
    invalid=config; invalid.initial_velocity={-1,0,0}; EXPECT_FALSE(ValidConfig(invalid));
    EXPECT_FALSE(run.initialized()); EXPECT_EQ(run.allocations().device_bytes,0);
    EXPECT_EQ(run.wall_setup(),nullptr); EXPECT_EQ(run.accepted_contact(),nullptr); EXPECT_EQ(run.wall_metrics(),nullptr);
    const auto initialized=run.Initialize(f->source,config,f->wall,f->wall_bytes,settings);
    ASSERT_TRUE(initialized)<<initialized.message;
    ASSERT_NE(run.wall_setup(),nullptr); ASSERT_NE(run.wall_metrics(),nullptr);
    EXPECT_EQ(run.accepted_contact(),nullptr); EXPECT_GT(run.initial_kinetic_energy(),0);
    EXPECT_EQ(run.wall_setup()->certificate()->measured_initial_kinetic,run.initial_kinetic_energy());
    EXPECT_EQ(run.wall_setup()->settings()->leading_gap,.0005);
    EXPECT_GT(run.wall_setup()->certificate()->leading_gap.lower,0);
    EXPECT_EQ(run.wall_metrics()->wall_kick_impulse,0); EXPECT_EQ(run.wall_metrics()->first_contact_epoch,0);
    Snapshot initial; ASSERT_TRUE(run.Capture(&initial));
    EXPECT_EQ(initial.position,f->source.coordinates()); EXPECT_EQ(initial.stamp.epoch,0);
    EXPECT_EQ(initial.velocity,initial.synchronized_velocity);
    EXPECT_EQ(run.wall_metrics()->carried_angular_momentum,CarriedAngularMomentum(run.binding(),initial));
    const auto& p=SourcePartElasticTestAccess::Internal(run);
    EXPECT_EQ(p.device_pulse,nullptr);
    EXPECT_LE(p.wall->contributor.step_rate_upper(),settings.maximum_step_rate);
    EXPECT_EQ(run.Initialize(f->source,config,f->wall,f->wall_bytes,settings).status,Status::AlreadyInitialized);
}
TEST(SourcePartWallEngine, ActualGap8448IntervalNativeOnsetAndLateFailureRetry) {
    const auto f=std::make_unique<wall_test::Fixture>();
    SourcePartElasticCase run;
    const auto config=MeshWallConfig(1); const auto settings=MeshWallSettings(config);
    const auto initialized=run.Initialize(f->source,config,f->wall,f->wall_bytes,settings);
    ASSERT_TRUE(initialized)<<initialized.message;
    auto native=std::make_unique<NativeSequence>(); ASSERT_NO_FATAL_FAILURE(native->Initialize(run));
    FlightOracle flight; ASSERT_NO_FATAL_FAILURE(flight.Initialize(run,*native));
    const auto allocation=run.allocations();
    auto host=std::make_unique<wall_contact::NodalWallResult>();
    std::array<long double,3*NodeCount> force{};
    Snapshot base,endpoint; ASSERT_TRUE(run.Capture(&base));
    double peak_reaction=0,peak_penetration=0,peak_energy_upper=0,peak_chord=0;
    for(unsigned step=0;step<WallOnsetPrefix;++step) {
        SCOPED_TRACE(step);
        ASSERT_NO_FATAL_FAILURE(HostWall(run,base,base.stamp.epoch,1,*host));
        ASSERT_NO_FATAL_FAILURE(HostForce(*host,force));
        const auto report=run.Step();
        ASSERT_TRUE(report)<<report.message<<" source="<<report.source_parent_id
            <<" measured="<<report.measured<<" limit="<<report.limit;
        ASSERT_TRUE(run.Capture(&endpoint)); ASSERT_NE(run.accepted_contact(),nullptr);
        const auto& contact=*run.accepted_contact(); const auto& d=contact.diagnostics;
        ASSERT_NO_FATAL_FAILURE(native->Check(run,base,endpoint,&force));
        ASSERT_NO_FATAL_FAILURE(HostWall(run,endpoint,d.base_epoch,d.attempt,*host));
        ASSERT_NO_FATAL_FAILURE(HostAgreement(contact,*host));
        EXPECT_EQ(d.phase,wall_contact::NodalWallDevicePhase::PreparedCandidate);
        EXPECT_EQ(d.owner_id,endpoint.stamp.owner_id); EXPECT_EQ(d.base_epoch,base.stamp.epoch);
        EXPECT_EQ(d.time,endpoint.stamp.time); EXPECT_EQ(d.velocity_time,endpoint.stamp.velocity_time);
        EXPECT_EQ(d.kick_dt,step?config.dt:config.dt/2);
        EXPECT_EQ(endpoint.stamp.velocity_phase,fe::NodalVelocityPhase::PreviousMidpoint);
        const auto& metrics=*run.wall_metrics();
        EXPECT_LE(std::abs(endpoint.diagnostics.energy_residual)+metrics.physical_energy_uncertainty,metrics.energy_allowance);
        EXPECT_GE(endpoint.diagnostics.total_internal_work,-.01*run.initial_kinetic_energy());
        EXPECT_LE(d.maximum_penetration,settings.penetration_cap);
        if(step+1<=8388) {
            EXPECT_EQ(d.resultant.value,0); EXPECT_EQ(d.potential.value,0); EXPECT_EQ(metrics.wall_kick_impulse,0);
            if((step+1)%128==0||step+1==8388) ASSERT_NO_FATAL_FAILURE(flight.Check(run,endpoint,step+1));
        }
        if(step+1==8389) {
            EXPECT_GT(d.resultant.lower,0); EXPECT_GT(d.potential.lower,0);
            EXPECT_EQ(metrics.first_contact_epoch,8389); EXPECT_EQ(d.wall_kick_impulse,0);
        }
        if(step+1==8390) EXPECT_GT(d.wall_kick_impulse,0);
        peak_reaction=std::max(peak_reaction,d.wall_reaction.x);
        peak_penetration=std::max(peak_penetration,d.maximum_penetration);
        peak_energy_upper=std::max(peak_energy_upper,std::abs(endpoint.diagnostics.energy_residual)+metrics.physical_energy_uncertainty);
        peak_chord=std::max(peak_chord,endpoint.diagnostics.maximum_chord_change);
        ASSERT_FALSE(::testing::Test::HasFailure());
        native->Accept(); base=endpoint;
    }
    EXPECT_GT(run.accepted_contact()->diagnostics.resultant.lower,0);
    EXPECT_GT(run.accepted_contact()->diagnostics.potential.lower,0);
    EXPECT_GT(run.wall_metrics()->wall_kick_impulse,0); EXPECT_GT(peak_chord,0);
    EXPECT_EQ(run.wall_metrics()->first_contact_epoch,8389);
    ASSERT_NO_FATAL_FAILURE(CheckWallRollbackAndRetry(run,*native));
    EXPECT_EQ(run.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(run.allocations().device_allocations,allocation.device_allocations);
    RecordProperty("native_onset_prefix_intervals",WallOnsetPrefix);
    RecordProperty("native_qeph_onset_intervals",88*(WallOnsetPrefix+1));
    RecordProperty("native_t3_onset_intervals",6*(WallOnsetPrefix+1));
    RecordProperty("source_readiness_sha256",source::ReadinessSha256);
    const auto record=[&](const char* key,double value) {
        std::ostringstream text; text<<std::scientific<<std::setprecision(17)<<value; RecordProperty(key,text.str());
    };
    record("peak_contact_reaction_N",peak_reaction); record("peak_penetration_m",peak_penetration);
    record("peak_energy_error_upper_J",peak_energy_upper); record("peak_chord_change_m",peak_chord);
    record("initial_kinetic_J",run.initial_kinetic_energy());
    record("precontact_max_position_error_m",static_cast<double>(flight.maximum_position_error));
    record("precontact_max_velocity_error_m_s",static_cast<double>(flight.maximum_raw_velocity_error));
}
} // namespace
} // namespace crash::cases::source_part_elastic::test
int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if(argc!=3) { std::cerr<<"Usage: source_part_wall_engine_check READINESS WALL [gtest options]\n"; return 2; }
    crash::cases::source_part_wall::check::SourcePath=argv[1];
    crash::cases::source_part_wall::check::WallPath=argv[2];
    return RUN_ALL_TESTS();
}
