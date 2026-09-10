#include "Fixture.h"
#include "case/source_assembly_observation/tests/ForceStageOracle.h"
#include <cstring>
#include <sstream>

namespace crash::cases::source_assembly_dynamics::test {
namespace value_test=source_assembly_observation::test;
using Access=SourceAssemblyDynamicsTestAccess;
TEST_F(SourceAssemblyDynamicsCheck, ForceStageStartupChargesExactOptionalHostAndDevicePayload) {
    const auto bindings=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto setup=PrepareWall(bindings);auto config=SmokeConfig();SourceAssemblyWallCase plain,observed;
    EXPECT_FALSE(config.observe_force_stage);EXPECT_EQ(observed.accepted_force_stage(),nullptr);
    ASSERT_TRUE(plain.Initialize(bindings,setup,config));config.observe_force_stage=true;
    const auto r=observed.Initialize(bindings,setup,config);ASSERT_TRUE(r)<<r.message;
    EXPECT_EQ(plain.accepted_force_stage(),nullptr);EXPECT_EQ(observed.accepted_force_stage(),nullptr);
    EXPECT_EQ(Access::CaptureStorage(plain)[2],0u);EXPECT_EQ(Access::CaptureStorage(plain)[3],0u);
    const auto extra_host=6*1030*sizeof(double)+6*sizeof(fe::NodalRigidGroupAccelerationSnapshot);
    EXPECT_EQ(observed.host_payload_bytes()-plain.host_payload_bytes(),extra_host);
    EXPECT_EQ(observed.allocations().device_bytes-plain.allocations().device_bytes,49728u);
    EXPECT_EQ(observed.allocations().device_allocations,plain.allocations().device_allocations);
    const auto required_host=observed.host_payload_bytes();SourceAssemblyWallCase small;
    config.storage.max_host_bytes=required_host-1;
    EXPECT_EQ(small.Initialize(bindings,setup,config).status,Status::ResourceLimit);
    EXPECT_FALSE(small.initialized());EXPECT_EQ(small.allocations().device_bytes,0u);
    EXPECT_EQ(small.accepted_force_stage(),nullptr);
    config.storage.max_host_bytes=required_host;ASSERT_TRUE(small.Initialize(bindings,setup,config));
    EXPECT_EQ(small.host_payload_bytes(),required_host);
    // Participant owner admission independently includes the capture tail.
    SourceAssemblyWallCase small_device;config.storage.owner_device_bytes=observed.owner()->allocations().device_bytes-1;
    EXPECT_EQ(small_device.Initialize(bindings,setup,config).status,Status::ResourceLimit);
    EXPECT_EQ(small_device.owner(),nullptr);EXPECT_EQ(small_device.accepted_force_stage(),nullptr);
}
TEST_F(SourceAssemblyDynamicsCheck, ForceStageActual128ContactIntervalsPreserveStateHistoryAndPhaseExactly) {
    const auto bindings=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto setup=PrepareWall(bindings);auto config=SmokeConfig();SourceAssemblyWallCase plain,observed;
    ASSERT_TRUE(plain.Initialize(bindings,setup,config));config.observe_force_stage=true;
    ASSERT_TRUE(observed.Initialize(bindings,setup,config));
    const auto allocations=observed.allocations();const auto host=observed.host_payload_bytes();
    const auto workspace=Access::CaptureStorage(observed);
    const auto initial=observed.diagnostics()->motion.after;
    for(unsigned step=0;step<128;++step) {
        SCOPED_TRACE(step);const auto before=Access::Accepted(observed);
        auto r=plain.Step();ASSERT_TRUE(r)<<r.message;r=observed.Step();ASSERT_TRUE(r)<<r.message;
        const auto& p=Access::Accepted(plain);const auto& o=Access::Accepted(observed);
        SameFields(p.fields,o.fields);SameParents(p.parents,o.parents);
        EXPECT_EQ(p.diagnostics.maximum_plastic_strain,o.diagnostics.maximum_plastic_strain);
        EXPECT_EQ(p.diagnostics.cumulative_plastic_work,o.diagnostics.cumulative_plastic_work);
        EXPECT_EQ(p.diagnostics.native_internal_work,o.diagnostics.native_internal_work);
        EXPECT_EQ(p.diagnostics.motion.native_residual,o.diagnostics.motion.native_residual);
        EXPECT_EQ(p.diagnostics.motion.after.effective_total,o.diagnostics.motion.after.effective_total);
        EXPECT_EQ(p.wall.diagnostics.resultant.value,o.wall.diagnostics.resultant.value);
        EXPECT_EQ(p.wall.diagnostics.potential.value,o.wall.diagnostics.potential.value);
        EXPECT_EQ(p.wall.wall_face,o.wall.wall_face);
        EXPECT_EQ(std::memcmp(p.wall.parents.data(),o.wall.parents.data(),p.wall.parents.size()*sizeof(contact::NodalWallParentResult)),0);
        EXPECT_EQ(std::memcmp(p.wall.nodes.data(),o.wall.nodes.data(),p.wall.nodes.size()*sizeof(contact::NodalWallPointResult)),0);
        EXPECT_EQ(plain.accepted_force_stage(),nullptr);const auto* result=observed.accepted_force_stage();ASSERT_NE(result,nullptr);
        EXPECT_EQ(result->owner_id,observed.owner()->accepted().owner_id);
        EXPECT_EQ(result->base_epoch,step);EXPECT_EQ(result->enclosing_epoch,step+1);
        EXPECT_EQ(result->attempt,observed.diagnostics()->shells.qeph.attempt);
        EXPECT_EQ(result->enclosing_time,(step+1)*Dt);EXPECT_EQ(result->phase.force_time,step*Dt);
        EXPECT_EQ(result->phase.input_velocity_time,step?(step-.5)*Dt:0);
        EXPECT_EQ(result->phase.previous_frame_time,step?(step-1)*Dt:0);
        EXPECT_EQ(result->phase.durations.previous_drift_dt,step?Dt:0);
        EXPECT_EQ(result->phase.durations.kick_dt,step?Dt:.5*Dt);EXPECT_EQ(result->phase.durations.drift_dt,Dt);
        EXPECT_EQ(result->source.source_instance_id,bindings.source_instance_id());
        EXPECT_EQ(result->source.group_count,6u);EXPECT_EQ(result->source.member_count,76u);
        const auto input=Access::CapturedInput(observed,before);
        EXPECT_TRUE(fe::trial_identity::SamePrepared(input.prepared,input.frame_prepared));
        EXPECT_TRUE(fe::trial_identity::SamePrepared(input.prepared,input.capture_prepared));
        value_test::Check(*result,value_test::Oracle(input));
        if(!step) {
            EXPECT_EQ(result->native_total,initial.native_total);EXPECT_EQ(result->effective_total,initial.effective_total);
            EXPECT_EQ(result->groups.primary_translation,initial.groups.primary_translation);
            EXPECT_GT(result->groups.primary_translation,0);
        }
        SameAllocations(observed,allocations,host);EXPECT_EQ(Access::CaptureStorage(observed),workspace);
    }
    EXPECT_EQ(observed.diagnostics()->first_contact_epoch,42u);EXPECT_GT(observed.accepted_contact().diagnostics->resultant.value,0);
    std::ostringstream maximum;maximum.precision(17);maximum<<observed.diagnostics()->maximum_plastic_strain;
    RecordProperty("maximum_plastic_strain",maximum.str());
    // This baseline short gate proves contact and exact existing material
    // histories. It does not assert that 128 baseline steps reach first yield.
}
} // namespace crash::cases::source_assembly_dynamics::test
