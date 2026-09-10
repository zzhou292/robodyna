#include "SourcePartUniformFlightTestSupport.h"
#include "case/SourcePartElasticArtifacts.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace crash::cases::source_part_elastic::test {
namespace {
class SourcePartUniformFlight : public ::testing::Test {
  protected:
    source::SourcePartContactFixture source;
    void SetUp() override {
        const auto loaded=source::LoadPinnedSourcePartContact(SourceReadinessPath(),&source);
        ASSERT_EQ(loaded.status,source::FixtureStatus::Ok)<<loaded.diagnostic;
        int devices=0; ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess); ASSERT_GT(devices,0);
    }
};
TEST_F(SourcePartUniformFlight, ExplicitModeValidationAndMeasuredInitialStateExcludePulseArchives) {
    auto config=UniformFlightConfig(1);
    ASSERT_TRUE(ValidConfig(config));
    EXPECT_EQ(config.experiment,Experiment::UniformFlight);
    EXPECT_EQ(config.initial_velocity,(std::array<double,3>{1,0,0}));
    EXPECT_EQ(UniformFlightConfig(2).dt,config.dt/2); EXPECT_EQ(UniformFlightConfig(4).dt,config.dt/4);
    SourcePartElasticCase run;
    auto reject=[&](const Config& bad) {
        EXPECT_FALSE(ValidConfig(bad));
        EXPECT_EQ(run.Initialize(source,bad).status,Status::InvalidInput);
        EXPECT_FALSE(run.initialized()); EXPECT_EQ(run.allocations().device_bytes,0);
        EXPECT_EQ(run.initial_kinetic_energy(),0);
    };
    auto bad=config; bad.experiment=static_cast<Experiment>(999); reject(bad);
    bad=config; bad.initial_velocity={}; reject(bad);
    bad=config; bad.initial_velocity[2]=std::numeric_limits<double>::infinity(); reject(bad);
    bad=config; bad.pulse_duration=config.dt; reject(bad);
    bad=config; bad.acceleration=1; reject(bad);
    bad=config; bad.spatial_axis=2; reject(bad);
    bad=config; bad.direction={1,0,0}; reject(bad);
    bad=config; bad.acceleration=std::numeric_limits<double>::quiet_NaN(); reject(bad);
    bad=config; bad.pulse_duration=std::numeric_limits<double>::quiet_NaN(); reject(bad);
    bad=PilotConfig(1); bad.initial_velocity={1,0,0}; reject(bad);
    bad=config; bad.experiment=Experiment::ElasticPulse; reject(bad);
    ASSERT_TRUE(run.Initialize(source,config));
    Snapshot initial,actual; ASSERT_TRUE(run.Capture(&initial)); actual=initial;
    ASSERT_EQ(run.owner().CopyAccepted({actual.position.data(),actual.velocity.data(),NodeCount,
        actual.orientation.data(),actual.omega.data()},&actual.stamp).status,fe::NodalStatus::Ok);
    SameSnapshot(initial,actual);
    EXPECT_EQ(initial.position,source.coordinates()); EXPECT_EQ(initial.stamp.epoch,0);
    EXPECT_EQ(initial.stamp.velocity_phase,fe::NodalVelocityPhase::Collocated);
    EXPECT_EQ(initial.synchronized_velocity,initial.velocity); EXPECT_EQ(initial.synchronized_omega,initial.omega);
    for(std::size_t n=0;n<NodeCount;++n) for(unsigned a=0;a<3;++a) {
        EXPECT_EQ(initial.velocity[3*n+a],config.initial_velocity[a]); EXPECT_EQ(initial.omega[3*n+a],0);
    }
    const auto& d=initial.diagnostics;
    EXPECT_GT(run.initial_kinetic_energy(),0);
    EXPECT_EQ(run.initial_kinetic_energy(),d.shells.kinetic.translation+d.shells.kinetic.rotation);
    EXPECT_EQ(d.synchronized_kinetic,run.initial_kinetic_energy());
    EXPECT_EQ(d.shells.base_kinetic.translation,0); EXPECT_EQ(d.shells.base_kinetic.rotation,0);
    EXPECT_FALSE(d.shells.qeph.kinetic_available); EXPECT_FALSE(d.shells.t3.kinetic_available);
    EXPECT_EQ(d.shells.qeph.kinetic_translation,0); EXPECT_EQ(d.shells.t3.kinetic_translation,0);
    EXPECT_EQ(d.shells.qeph.kinetic_rotation,0); EXPECT_EQ(d.shells.t3.kinetic_rotation,0);
    EXPECT_FALSE(d.shells.qeph.has_completed_interval); EXPECT_FALSE(d.shells.t3.has_completed_interval);
    auto& p=SourcePartElasticTestAccess::Internal(run);
    EXPECT_EQ(p.device_pulse,nullptr);
    for(double force:p.pulse_force) EXPECT_EQ(force,0);
    const auto path=std::filesystem::temp_directory_path()/
        ("robo-dyna-uniform-reject-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    ASSERT_FALSE(std::filesystem::exists(path));
    EXPECT_THROW((SourcePartElasticArtifacts(path.string(),run,64,64,1,1)),std::runtime_error);
    EXPECT_FALSE(std::filesystem::exists(path));
    ASSERT_TRUE(run.Capture(&actual)); SameSnapshot(initial,actual);
}
TEST_F(SourcePartUniformFlight, All94OriginalShellsAgreeWithNativeThrough64RigidFlightIntervals) {
    SourcePartElasticCase run;
    ASSERT_TRUE(run.Initialize(source,UniformFlightConfig(1)));
    auto native=std::make_unique<NativeSequence>();
    ASSERT_NO_FATAL_FAILURE(native->Initialize(run.binding()));
    FlightOracle flight; ASSERT_NO_FATAL_FAILURE(flight.Initialize(run,*native));
    const auto allocation=run.allocations();
    Snapshot base,endpoint; ASSERT_TRUE(run.Capture(&base));
    ASSERT_NO_FATAL_FAILURE(flight.Check(run,base,0));
    for(unsigned step=0;step<64;++step) {
        SCOPED_TRACE(step);
        const auto report=run.Step();
        ASSERT_EQ(report.status,Status::Ok)<<report.message<<" source="<<report.source_parent_id
            <<" measured="<<report.measured<<" limit="<<report.limit;
        ASSERT_TRUE(run.Capture(&endpoint));
        ASSERT_NO_FATAL_FAILURE(native->Check(run,base,endpoint));
        ASSERT_NO_FATAL_FAILURE(flight.Check(run,endpoint,step+1));
        ASSERT_FALSE(::testing::Test::HasFailure());
        native->Accept();
        EXPECT_EQ(endpoint.stamp.epoch,step+1);
        EXPECT_EQ(endpoint.stamp.velocity_phase,fe::NodalVelocityPhase::PreviousMidpoint);
        EXPECT_EQ(endpoint.stamp.velocity_time,endpoint.stamp.time-run.config().dt/2);
        EXPECT_EQ(endpoint.stamp.reaction_kick_dt,step?run.config().dt:run.config().dt/2);
        if(!step) EXPECT_EQ(endpoint.diagnostics.shells.base_kinetic.translation,run.initial_kinetic_energy());
        base=endpoint;
    }
    EXPECT_EQ(run.allocations().device_allocations,allocation.device_allocations);
    EXPECT_EQ(run.allocations().device_bytes,allocation.device_bytes);
    RecordProperty("native_qeph_uniform_intervals",88*64); RecordProperty("native_t3_uniform_intervals",6*64);
    RecordProperty("source_readiness_sha256",source::ReadinessSha256);
    const auto record=[&](const char* name,long double value) {
        std::ostringstream text; text<<std::scientific<<std::setprecision(17)<<value;
        RecordProperty(name,text.str());
    };
    record("uniform_native_mass_kg",flight.total_mass); record("uniform_initial_kinetic_J",run.initial_kinetic_energy());
    record("uniform_max_position_error_m",flight.maximum_position_error);
    record("uniform_max_raw_velocity_error_m_s",flight.maximum_raw_velocity_error);
    record("uniform_max_synchronized_velocity_error_m_s",flight.maximum_synchronized_velocity_error);
    record("uniform_max_angular_velocity_rad_s",flight.maximum_angular_velocity);
    record("uniform_max_chord_change_m",flight.maximum_chord_change);
    record("uniform_max_energy_residual_J",flight.maximum_energy_residual);
}
TEST_F(SourcePartUniformFlight, RejectedFirstMovingTrialPreservesMeasuredK0AndRetriesExactly) {
    SourcePartElasticCase run,clean;
    ASSERT_TRUE(run.Initialize(source,UniformFlightConfig(1)));
    ASSERT_TRUE(clean.Initialize(source,UniformFlightConfig(1)));
    Snapshot before,after,expected; ASSERT_TRUE(run.Capture(&before));
    const auto initial_kinetic=run.initial_kinetic_energy();
    auto original=std::make_unique<Results>(),observed=std::make_unique<Results>(),clean_results=std::make_unique<Results>();
    ASSERT_NO_FATAL_FAILURE(ReadResults(run,*original));
    auto& p=SourcePartElasticTestAccess::Internal(run);
    const auto q_magnitudes=p.accepted_q_work_magnitude,t_magnitudes=p.accepted_t_work_magnitude;
    ASSERT_TRUE(p.Prepare()); ASSERT_TRUE(p.Evaluate());
    EXPECT_EQ(p.prepared.kick_dt,run.config().dt/2);
    // Fault after both native families and common K have prepared, before commit.
    p.trial.orientation.back()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(p.Observe().status,Status::EnvelopeFailure); p.Discard();
    ASSERT_TRUE(run.Capture(&after)); SameSnapshot(before,after);
    ASSERT_EQ(run.owner().CopyAccepted({after.position.data(),after.velocity.data(),NodeCount,
        after.orientation.data(),after.omega.data()},&after.stamp).status,fe::NodalStatus::Ok);
    SameSnapshot(before,after);
    EXPECT_EQ(run.initial_kinetic_energy(),initial_kinetic);
    EXPECT_EQ(after.diagnostics.shells.kinetic.translation,initial_kinetic);
    EXPECT_EQ(p.accepted_q_work_magnitude,q_magnitudes); EXPECT_EQ(p.accepted_t_work_magnitude,t_magnitudes);
    ASSERT_NO_FATAL_FAILURE(ReadResults(run,*observed)); SameResults(*original,*observed);
    auto native=std::make_unique<NativeSequence>(); ASSERT_NO_FATAL_FAILURE(native->Initialize(run.binding()));
    ASSERT_TRUE(run.Step()); ASSERT_TRUE(clean.Step());
    ASSERT_TRUE(run.Capture(&after)); ASSERT_TRUE(clean.Capture(&expected)); SameSnapshot(after,expected,false);
    ASSERT_NO_FATAL_FAILURE(ReadResults(run,*observed));
    ASSERT_NO_FATAL_FAILURE(ReadResults(clean,*clean_results)); SameResults(*observed,*clean_results);
    ASSERT_NO_FATAL_FAILURE(native->Check(run,before,after));
    EXPECT_EQ(after.diagnostics.shells.base_kinetic.translation,initial_kinetic);
    EXPECT_EQ(run.initial_kinetic_energy(),initial_kinetic);
}
} // namespace
} // namespace crash::cases::source_part_elastic::test
