#include "Fixture.h"
#include "lib_utest/qualification/nodal_rigid_group/PreparedSnapshotCudaProbe.h"
#include <cstring>

namespace crash::cases::source_assembly_dynamics::test {
using Access=SourceAssemblyDynamicsTestAccess;
TEST_F(SourceAssemblyDynamicsCheck, LateForceStageInputsPreserveAcceptedSummaryAndAllowExactRetry) {
    const auto bindings=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto setup=PrepareWall(bindings);auto config=SmokeConfig();config.observe_force_stage=true;
    SourceAssemblyWallCase run,clean;ASSERT_TRUE(run.Initialize(bindings,setup,config));ASSERT_TRUE(clean.Initialize(bindings,setup,config));
    for(unsigned i=0;i<64;++i) {auto r=run.Step();ASSERT_TRUE(r)<<r.message;r=clean.Step();ASSERT_TRUE(r)<<r.message;}
    ASSERT_GT(run.diagnostics()->active_contact_nodes,0u);const auto* visible=run.accepted_force_stage();ASSERT_NE(visible,nullptr);
    const auto before=Access::Accepted(run);const auto storage=Access::CaptureStorage(run);
    const auto allocations=run.allocations();const auto host=run.host_payload_bytes();
    for(auto fault:{Access::ForceFault::LastNode,Access::ForceFault::LastGroup,Access::ForceFault::CaptureIdentity,
                    Access::ForceFault::GroupIdentity,Access::ForceFault::CaptureAssociation}) {
        SCOPED_TRACE(static_cast<unsigned>(fault));const auto r=Access::RejectForceStage(run,fault);
        EXPECT_EQ(r.status,fault==Access::ForceFault::CaptureAssociation?Status::ComponentFailure:Status::ObservationFailure)<<r.message;
        SameAccepted(before,Access::Accepted(run));EXPECT_EQ(run.accepted_force_stage(),visible);
        EXPECT_TRUE(fe::trial_identity::SameStamp(run.owner()->accepted(),before.diagnostics.stamp));
        SameAllocations(run,allocations,host);EXPECT_EQ(Access::CaptureStorage(run),storage);
    }
    // A fresh accepted-output consumer also reads the retained TL state/history
    // after rejection; duplicate capture into an old consumer is not required.
    SourceAssemblyAcceptedOutput output;
    ASSERT_EQ(output.Initialize(*run.owner(),bindings,{run.owner()->accepted().owner_id,7,9},11).status,visual::Status::Ok);
    CheckOutput(run,output);
    auto r=run.Step();ASSERT_TRUE(r)<<r.message;r=clean.Step();ASSERT_TRUE(r)<<r.message;
    SameFields(Access::Accepted(run).fields,Access::Accepted(clean).fields);
    SameParents(Access::Accepted(run).parents,Access::Accepted(clean).parents);
    const auto* actual=run.accepted_force_stage();const auto* expected=clean.accepted_force_stage();
    ASSERT_NE(actual,nullptr);ASSERT_NE(expected,nullptr);EXPECT_EQ(actual->enclosing_epoch,65u);
    EXPECT_GT(actual->attempt,expected->attempt);EXPECT_EQ(actual->base_epoch,expected->base_epoch);
    EXPECT_EQ(actual->enclosing_time,expected->enclosing_time);
    EXPECT_EQ(std::memcmp(&actual->phase,&expected->phase,sizeof(actual->phase)),0);
    EXPECT_EQ(std::memcmp(&actual->ordinary,&expected->ordinary,sizeof(actual->ordinary)),0);
    EXPECT_EQ(std::memcmp(&actual->grouped_members,&expected->grouped_members,sizeof(actual->grouped_members)),0);
    EXPECT_EQ(std::memcmp(&actual->groups,&expected->groups,sizeof(actual->groups)),0);
    EXPECT_EQ(actual->native_total,expected->native_total);EXPECT_EQ(actual->effective_total,expected->effective_total);
    EXPECT_EQ(actual->replacement,expected->replacement);
    EXPECT_EQ(run.accepted_contact().diagnostics->resultant.value,clean.accepted_contact().diagnostics->resultant.value);
    SameAllocations(run,allocations,host);EXPECT_EQ(Access::CaptureStorage(run),storage);
}
TEST_F(SourceAssemblyDynamicsCheck, CompletedForceStageReadThenDeviceFailurePoisonsWithoutChangingAcceptedSummary) {
    const auto bindings=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto setup=PrepareWall(bindings);auto config=SmokeConfig();config.observe_force_stage=true;SourceAssemblyWallCase run;
    ASSERT_TRUE(run.Initialize(bindings,setup,config));ASSERT_TRUE(run.Step());
    const auto before=Access::Accepted(run);const auto* visible=run.accepted_force_stage();ASSERT_NE(visible,nullptr);
    const auto allocations=run.allocations();const auto host=run.host_payload_bytes();const auto storage=Access::CaptureStorage(run);
    const auto r=Access::RejectCaptureDevice(run);EXPECT_EQ(r.status,Status::DeviceFailure)<<r.message;
    EXPECT_TRUE(prepared_snapshot_probe::CompletedReadBeforeFailure());
    SameAccepted(before,Access::Accepted(run));EXPECT_EQ(run.accepted_force_stage(),visible);
    EXPECT_TRUE(fe::trial_identity::SameStamp(run.owner()->accepted(),before.diagnostics.stamp));
    EXPECT_EQ(run.Step().status,Status::DeviceFailure);SameAccepted(before,Access::Accepted(run));
    EXPECT_EQ(run.accepted_force_stage(),visible);SameAllocations(run,allocations,host);
    EXPECT_EQ(Access::CaptureStorage(run),storage);
}
} // namespace crash::cases::source_assembly_dynamics::test
