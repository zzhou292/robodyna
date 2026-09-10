#include "SourcePartWallEngineTestSupport.h"

namespace crash::cases::source_part_elastic::test {
void CheckWallRollbackAndRetry(SourcePartElasticCase& run,NativeSequence& native) {
    Snapshot before,after; ASSERT_TRUE(run.Capture(&before));
    const auto initial_kinetic=run.initial_kinetic_energy();
    const auto saved_contact=std::make_unique<wall_contact::NodalWallDeviceResults>(*run.accepted_contact());
    const auto saved_metrics=*run.wall_metrics();
    auto original=std::make_unique<Results>(),observed=std::make_unique<Results>();
    ASSERT_NO_FATAL_FAILURE(ReadResults(run,*original));
    auto& p=SourcePartElasticTestAccess::Internal(run);
    const auto q_magnitude=p.accepted_q_work_magnitude,t_magnitude=p.accepted_t_work_magnitude;
    ASSERT_TRUE(p.Prepare()); ASSERT_TRUE(p.Evaluate()); ASSERT_TRUE(p.Observe());
    const auto clean_snapshot=p.trial;
    const auto clean_shells=std::make_unique<Results>(); clean_shells->qeph=p.qresult; clean_shells->t3=p.tresult;
    const auto clean_contact=std::make_unique<wall_contact::NodalWallDeviceResults>(p.wall->trial);
    const auto clean_metrics=p.wall->trial_metrics;
    p.Discard();
    for(unsigned fault=0;fault<2;++fault) {
        SCOPED_TRACE(fault);
        ASSERT_TRUE(p.Prepare());
        if(fault==0) {
            ASSERT_TRUE(p.EvaluateShells());
            // Last original node leaves the finite mesh after both material
            // candidates exist. Only prepared device scratch is modified.
            const double outside=2;
            ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(p.prepared.kinematics.position_xyz)+3*(NodeCount-1)+1,
                &outside,sizeof(outside),cudaMemcpyHostToDevice,p.prepared.stream),cudaSuccess);
            ASSERT_EQ(cudaStreamSynchronize(p.prepared.stream),cudaSuccess);
            EXPECT_EQ(p.EvaluateWall().status,Status::ComponentFailure);
        } else {
            ASSERT_TRUE(p.Evaluate());
            p.wall->trial.nodes[NodeCount-1].force_world.x=std::numeric_limits<double>::quiet_NaN();
            EXPECT_FALSE(p.Observe());
        }
        p.Discard();
        ASSERT_TRUE(run.Capture(&after)); SameSnapshot(before,after);
        ASSERT_EQ(run.owner().CopyAccepted({after.position.data(),after.velocity.data(),NodeCount,
            after.orientation.data(),after.omega.data()},&after.stamp).status,fe::NodalStatus::Ok);
        SameSnapshot(before,after);
        EXPECT_EQ(wall_test::Bytes(*run.accepted_contact()),wall_test::Bytes(*saved_contact));
        SameWallMetrics(saved_metrics,*run.wall_metrics());
        ASSERT_NO_FATAL_FAILURE(ReadResults(run,*observed)); SameResults(*original,*observed);
        EXPECT_EQ(p.accepted_q_work_magnitude,q_magnitude); EXPECT_EQ(p.accepted_t_work_magnitude,t_magnitude);
        EXPECT_EQ(run.initial_kinetic_energy(),initial_kinetic);
    }
    auto host=std::make_unique<wall_contact::NodalWallResult>();
    ASSERT_NO_FATAL_FAILURE(HostWall(run,before,before.stamp.epoch,1,*host));
    std::array<long double,3*NodeCount> force; ASSERT_NO_FATAL_FAILURE(HostForce(*host,force));
    ASSERT_TRUE(run.Step()); ASSERT_TRUE(run.Capture(&after));
    EXPECT_EQ(after.position,clean_snapshot.position); EXPECT_EQ(after.velocity,clean_snapshot.velocity);
    EXPECT_EQ(after.orientation,clean_snapshot.orientation); EXPECT_EQ(after.omega,clean_snapshot.omega);
    EXPECT_EQ(after.synchronized_velocity,clean_snapshot.synchronized_velocity);
    EXPECT_EQ(after.synchronized_omega,clean_snapshot.synchronized_omega);
    EXPECT_EQ(after.diagnostics.energy_residual,clean_snapshot.diagnostics.energy_residual);
    ASSERT_NO_FATAL_FAILURE(ReadResults(run,*observed)); SameResults(*clean_shells,*observed);
    wall_test::SameScientificResult(*clean_contact,*run.accepted_contact());
    SameWallMetrics(clean_metrics,*run.wall_metrics());
    ASSERT_NO_FATAL_FAILURE(native.Check(run,before,after,&force));
    native.Accept();
}
} // namespace crash::cases::source_part_elastic::test
