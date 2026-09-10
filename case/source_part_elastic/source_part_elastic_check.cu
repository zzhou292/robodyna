#include "SourcePartElasticTestSupport.h"
#include <iostream>
#include <limits>

namespace crash::cases::source_part_elastic::test {
namespace {
std::filesystem::path readiness;
class SourcePartElastic : public ::testing::Test {
  protected:
    source::SourcePartContactFixture source;
    void SetUp() override {
        const auto loaded=source::LoadPinnedSourcePartContact(readiness,&source);
        ASSERT_EQ(loaded.status,source::FixtureStatus::Ok)<<loaded.diagnostic;
        int devices=0; ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess); ASSERT_GT(devices,0);
    }
};
__global__ void CollapseTriangle(fe::NodalPreparedView view,std::size_t a,std::size_t b,std::size_t c) {
    auto* x=const_cast<double*>(view.kinematics.position_xyz);
    for(unsigned i=0;i<3;++i) { x[3*b+i]=x[3*a+i]; x[3*c+i]=x[3*a+i]; }
}
TEST_F(SourcePartElastic, InvalidConfigurationIsAtomicAndInitialCaptureRetainsOriginalRestState) {
    SourcePartElasticCase run;
    Snapshot sentinel; sentinel.position[0]=42;
    EXPECT_EQ(run.Capture(&sentinel).status,Status::NotInitialized); EXPECT_EQ(sentinel.position[0],42);
    auto config=PilotConfig(1);
    ASSERT_TRUE(ValidConfig(config));
    auto invalid=config; invalid.dt=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(run.Initialize(source,invalid).status,Status::InvalidInput);
    EXPECT_FALSE(run.initialized()); EXPECT_EQ(run.allocations().device_bytes,0);
    ASSERT_TRUE(run.Initialize(source,config));
    Snapshot initial; ASSERT_TRUE(run.Capture(&initial));
    EXPECT_EQ(initial.position,source.coordinates()); EXPECT_EQ(initial.stamp.epoch,0);
    EXPECT_EQ(initial.stamp.velocity_phase,fe::NodalVelocityPhase::Collocated);
    for(double v:initial.velocity) EXPECT_EQ(v,0);
    for(double w:initial.omega) EXPECT_EQ(w,0);
    EXPECT_EQ(run.binding().node_count(),117); EXPECT_EQ(run.binding().qeph_count(),88); EXPECT_EQ(run.binding().t3_count(),6);
    EXPECT_EQ(run.Initialize(source,config).status,Status::AlreadyInitialized);
    EXPECT_EQ(PulseScale(0,config.pulse_duration),0); EXPECT_EQ(PulseScale(config.pulse_duration,config.pulse_duration),0);
    EXPECT_EQ(PulseScale(config.pulse_duration/2,config.pulse_duration),1);
    EXPECT_EQ(PulseScale(config.pulse_duration*2,config.pulse_duration),0);
    auto invalid_axis=config; invalid_axis.spatial_axis=3; EXPECT_FALSE(ValidConfig(invalid_axis));
    auto invalid_direction=config; invalid_direction.direction={1,1,0}; EXPECT_FALSE(ValidConfig(invalid_direction));
}
TEST_F(SourcePartElastic, All94OriginalShellsAgreeWithNativeThrough64AcceptedPulseIntervals) {
    SourcePartElasticCase run;
    ASSERT_TRUE(run.Initialize(source,PilotConfig(1)));
    auto native=std::make_unique<NativeSequence>();
    ASSERT_NO_FATAL_FAILURE(native->Initialize(run.binding()));
    const auto allocation=run.allocations();
    Snapshot base,endpoint; ASSERT_TRUE(run.Capture(&base));
    for(unsigned step=0;step<64;++step) {
        SCOPED_TRACE(step);
        const auto report=run.Step();
        ASSERT_EQ(report.status,Status::Ok)<<report.message<<" source="<<report.source_parent_id
            <<" measured="<<report.measured<<" limit="<<report.limit;
        ASSERT_TRUE(run.Capture(&endpoint));
        ASSERT_NO_FATAL_FAILURE(native->Check(run,base,endpoint));
        ASSERT_FALSE(::testing::Test::HasFailure());
        native->Accept();
        EXPECT_EQ(endpoint.stamp.epoch,step+1);
        EXPECT_EQ(endpoint.stamp.velocity_phase,fe::NodalVelocityPhase::PreviousMidpoint);
        EXPECT_EQ(endpoint.stamp.velocity_time,endpoint.stamp.time-run.config().dt/2);
        EXPECT_EQ(endpoint.stamp.reaction_kick_dt,step?run.config().dt:run.config().dt/2);
        base=endpoint;
    }
    EXPECT_GT(endpoint.diagnostics.maximum_chord_change,0);
    EXPECT_GT(endpoint.diagnostics.external_drift_work,0);
    EXPECT_GT(endpoint.diagnostics.shells.qeph.internal_work[0]+endpoint.diagnostics.shells.qeph.internal_work[1],0);
    EXPECT_GT(endpoint.diagnostics.shells.t3.internal_work[0]+endpoint.diagnostics.shells.t3.internal_work[1],0);
    EXPECT_EQ(run.allocations().device_allocations,allocation.device_allocations);
    EXPECT_EQ(run.allocations().device_bytes,allocation.device_bytes);
    RecordProperty("native_qeph_intervals",88*64); RecordProperty("native_t3_intervals",6*64);
    RecordProperty("owned_device_bytes",std::to_string(allocation.device_bytes));
    RecordProperty("source_readiness_sha256",source::ReadinessSha256);
}
TEST_F(SourcePartElastic, LateT3AndObservationFailuresPreserveAcceptedStateAndRetryExactly) {
    SourcePartElasticCase run,clean;
    ASSERT_TRUE(run.Initialize(source,PilotConfig(1)));
    ASSERT_TRUE(clean.Initialize(source,PilotConfig(1)));
    for(unsigned i=0;i<16;++i) {
        ASSERT_TRUE(run.Step());
        ASSERT_TRUE(clean.Step());
    }
    Snapshot before,after,expected;
    ASSERT_TRUE(run.Capture(&before));
    auto original=std::make_unique<Results>(),observed=std::make_unique<Results>(),clean_results=std::make_unique<Results>();
    ASSERT_NO_FATAL_FAILURE(ReadResults(run,*original));
    auto& p=SourcePartElasticTestAccess::Internal(run);
    const auto saved_q_magnitudes=p.accepted_q_work_magnitude;
    const auto saved_t_magnitudes=p.accepted_t_work_magnitude;
    ASSERT_TRUE(p.Prepare());
    ASSERT_EQ(p.qeph.EvaluateCandidate(p.prepared,&p.trial.diagnostics.shells.qeph).status,q::BatchStatus::Success);
    const auto& cell=run.binding().t3_nodes(source::T3Count-1);
    CollapseTriangle<<<1,1,0,p.prepared.stream>>>(p.prepared,cell[0],cell[1],cell[2]);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess); ASSERT_EQ(cudaStreamSynchronize(p.prepared.stream),cudaSuccess);
    const auto rejected=p.t3.EvaluateCandidate(p.prepared,&p.trial.diagnostics.shells.t3);
    EXPECT_EQ(rejected.status,t::BatchStatus::ElementFailure);
    EXPECT_EQ(rejected.element,source::T3Count-1);
    p.Discard();
    ASSERT_TRUE(run.Capture(&after)); SameSnapshot(before,after);
    ASSERT_EQ(run.owner().CopyAccepted({after.position.data(),after.velocity.data(),NodeCount,
        after.orientation.data(),after.omega.data()},&after.stamp).status,fe::NodalStatus::Ok);
    SameSnapshot(before,after);
    ASSERT_NO_FATAL_FAILURE(ReadResults(run,*observed)); SameResults(*original,*observed);
    ASSERT_TRUE(p.Prepare());
    ASSERT_TRUE(p.Evaluate());
    p.trial.orientation[0]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(p.Observe().status,Status::EnvelopeFailure);
    p.Discard();
    ASSERT_TRUE(run.Capture(&after)); SameSnapshot(before,after);
    ASSERT_EQ(run.owner().CopyAccepted({after.position.data(),after.velocity.data(),NodeCount,
        after.orientation.data(),after.omega.data()},&after.stamp).status,fe::NodalStatus::Ok);
    SameSnapshot(before,after);
    EXPECT_EQ(p.accepted_q_work_magnitude,saved_q_magnitudes);
    EXPECT_EQ(p.accepted_t_work_magnitude,saved_t_magnitudes);
    ASSERT_NO_FATAL_FAILURE(ReadResults(run,*observed)); SameResults(*original,*observed);
    ASSERT_TRUE(run.Step());
    ASSERT_TRUE(clean.Step());
    ASSERT_TRUE(run.Capture(&after));
    ASSERT_TRUE(clean.Capture(&expected)); SameSnapshot(after,expected,false);
    ASSERT_NO_FATAL_FAILURE(ReadResults(run,*observed));
    ASSERT_NO_FATAL_FAILURE(ReadResults(clean,*clean_results)); SameResults(*observed,*clean_results);
}
} // namespace
} // namespace crash::cases::source_part_elastic::test
int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if(argc!=2) { std::cerr<<"Usage: source_part_elastic_check READINESS [gtest options]\n"; return 2; }
    crash::cases::source_part_elastic::test::readiness=argv[1];
    return RUN_ALL_TESTS();
}
