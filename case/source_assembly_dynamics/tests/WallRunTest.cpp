#include "Fixture.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include "lib_src/elements/t3/T3History.h"
#include <cmath>

namespace crash::cases::source_assembly_dynamics::test {
namespace {
void CheckContactHorizon(unsigned steps) {
    const auto bindings=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto setup=PrepareWall(bindings);SourceAssemblyWallCase run;auto r=run.Initialize(bindings,setup,SmokeConfig());ASSERT_TRUE(r)<<r.message;
    SourceAssemblyAcceptedOutput output;
    ASSERT_EQ(output.Initialize(*run.owner(),bindings,{run.owner()->accepted().owner_id,17,29},31).status,visual::Status::Ok);
    ASSERT_NO_FATAL_FAILURE(CheckOutput(run,output));const auto allocation=run.allocations();const auto host=run.host_payload_bytes();
    const auto output_host=output.section_host_bytes(),nodal_host=output.nodal()->capture_bytes();
    const auto qlast=bindings.shells().qeph_count()-1,tlast=bindings.shells().t3_count()-1;
    for(unsigned i=0;i<steps;++i) {
        r=run.Step();ASSERT_TRUE(r)<<"interval="<<i+1<<" "<<r.message<<" source_parent="<<r.source_parent<<" node="<<r.node
                                  <<" measured="<<r.measured<<" limit="<<r.limit;
        const auto& sample=SourceAssemblyDynamicsTestAccess::Accepted(run);const auto& d=*run.diagnostics();
        EXPECT_EQ(d.stamp.epoch,i+1u);EXPECT_EQ(d.stamp.time,(i+1)*Dt);EXPECT_EQ(d.stamp.rigid_groups.group_count,6u);
        EXPECT_EQ(d.stamp.rigid_groups.member_count,76u);EXPECT_EQ(d.stamp.reaction_kick_dt,i?Dt:Dt/2);
        EXPECT_EQ(d.motion.after.phase.position_time,d.stamp.time);EXPECT_EQ(d.motion.after.phase.velocity_time,d.stamp.velocity_time);
        EXPECT_EQ(d.motion.after.phase.frame_time,d.stamp.reaction_time);
        EXPECT_TRUE(std::isfinite(d.motion.after.effective_total));EXPECT_LE(std::abs(d.motion.native_residual),d.motion.roundoff_budget);
        EXPECT_EQ(sample.parents.qeph[qlast].proposed_history.stamp().sample_index,i+1u);
        EXPECT_EQ(sample.parents.t3[tlast].proposed_history.stamp().sample_index,i+1u);
        EXPECT_TRUE(sample.parents.qeph[qlast].proposed_history.matches_reference(bindings.shells().qeph_reference(qlast)));
        EXPECT_TRUE(sample.parents.t3[tlast].proposed_history.matches_reference(bindings.shells().t3_reference(tlast)));
        if(i==0||i+1==steps||(i+1)%64==0)ASSERT_NO_FATAL_FAILURE(CheckOutput(run,output));
        SameAllocations(run,allocation,host);EXPECT_EQ(output.section_host_bytes(),output_host);EXPECT_EQ(output.nodal()->capture_bytes(),nodal_host);
    }
    const auto& d=*run.diagnostics();ASSERT_NE(run.accepted_contact().diagnostics,nullptr);
    EXPECT_GT(d.first_contact_epoch,0u);EXPECT_LT(d.first_contact_epoch,steps/2);EXPECT_GT(d.contact_intervals,0u);
    EXPECT_GT(run.accepted_contact().diagnostics->maximum_penetration,0);
    EXPECT_GT(run.accepted_contact().diagnostics->resultant.value,0);
    ::testing::Test::RecordProperty("scope","Actual 915-parent/1030-node source, six internal groups, finite mesh wall; short integration horizon, not an energy-validated crash or video");
    ::testing::Test::RecordProperty("accepted_intervals",int(steps));
    ::testing::Test::RecordProperty("accepted_time_s",source::test::Number(d.stamp.time));
    ::testing::Test::RecordProperty("first_contact_epoch",int(d.first_contact_epoch));
    ::testing::Test::RecordProperty("wall_resultant_N",source::test::Number(run.accepted_contact().diagnostics->resultant.value));
    ::testing::Test::RecordProperty("maximum_plastic_strain",source::test::Number(d.maximum_plastic_strain));
    ::testing::Test::RecordProperty("owned_device_bytes",int(allocation.device_bytes));
}
}
TEST_F(SourceAssemblyDynamicsCheck, Actual256IntervalMeshWallSmoke) { CheckContactHorizon(256); }
TEST_F(SourceAssemblyDynamicsCheck, Actual1024IntervalMeshWallSmoke) { CheckContactHorizon(1024); }
} // namespace crash::cases::source_assembly_dynamics::test
