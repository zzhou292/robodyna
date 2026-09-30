#include "Fixture.h"
#include <cstring>
#include <limits>

namespace crash::cases::source_assembly_dynamics {
Report SourceAssemblyDynamicsTestAccess::RejectSpin(SourceAssemblyWallCase& c,bool bad_phase) {
    auto& s=*c.impl_;auto r=s.Prepare();if(!r)return s.Stop(r);
    r=s.Evaluate();if(!r)return s.Stop(r);
    r=s.Check();if(!r)return s.Stop(r);
    // Test-only failure after the ordinary checks, in the optional observer.
    // The old native packets and the accepted owner are never mutated.
    const auto prepared=s.prepared;const auto value=s.applied_couple[3*459];
    if(bad_phase)++s.prepared.kinematics.base_epoch;
    else s.applied_couple[3*459]=std::numeric_limits<double>::quiet_NaN();
    r=s.CheckQephSpin();s.prepared=prepared;s.applied_couple[3*459]=value;
    if(r)return s.Stop(Failure(Status::ComponentFailure,"Injected spin observer fault unexpectedly passed"));
    return s.Stop(r);
}
namespace test {
using Access=SourceAssemblyDynamicsTestAccess;
TEST_F(SourceAssemblyDynamicsCheck, OptionalSpinActual64IntervalsPreserveMechanicsAndRetainCompleteBasePackets) {
    const auto b=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto setup=PrepareWall(b);auto cfg=SmokeConfig();SourceAssemblyWallCase plain,probe;
    EXPECT_EQ(cfg.observe_qeph_spin_node,0u);ASSERT_TRUE(plain.Initialize(b,setup,cfg));
    cfg.observe_qeph_spin_node=2181592;ASSERT_TRUE(probe.Initialize(b,setup,cfg));
    EXPECT_EQ(probe.accepted_qeph_spin(),nullptr);EXPECT_EQ(plain.accepted_qeph_spin(),nullptr);
    EXPECT_EQ(probe.host_payload_bytes()-plain.host_payload_bytes(),2*sizeof(observation::QephSpinObservation));
    EXPECT_EQ(probe.allocations().device_bytes,plain.allocations().device_bytes);
    EXPECT_EQ(probe.allocations().device_allocations,plain.allocations().device_allocations);
    const auto allocation=probe.allocations();const auto host=probe.host_payload_bytes();
    SourceAssemblyWallCase small;cfg.storage.max_host_bytes=host-1;
    EXPECT_EQ(small.Initialize(b,setup,cfg).status,Status::ResourceLimit);EXPECT_EQ(small.allocations().device_bytes,0u);
    for(unsigned step=0;step<64;++step) {
        SCOPED_TRACE(step);const auto before=Access::Accepted(probe);
        auto r=plain.Step();ASSERT_TRUE(r)<<r.message;r=probe.Step();ASSERT_TRUE(r)<<r.message;
        SameFields(Access::Accepted(plain).fields,Access::Accepted(probe).fields);
        SameParents(Access::Accepted(plain).parents,Access::Accepted(probe).parents);
        EXPECT_EQ(plain.accepted_contact().diagnostics->resultant.value,probe.accepted_contact().diagnostics->resultant.value);
        const auto* record=probe.accepted_qeph_spin();ASSERT_NE(record,nullptr);
        EXPECT_TRUE(fe::trial_identity::SameStamp(record->base,before.diagnostics.stamp));
        EXPECT_TRUE(fe::trial_identity::SameStamp(record->enclosing,probe.owner()->accepted()));
        EXPECT_EQ(record->attempt,probe.diagnostics()->shells.qeph.attempt);EXPECT_EQ(record->base.epoch,step);
        EXPECT_EQ(record->source_node,2181592u);EXPECT_EQ(record->global_node,459u);ASSERT_EQ(record->parent_count,2u);
        for(unsigned i=0;i<2;++i) {
            const auto& p=record->parents[i];EXPECT_EQ(p.source_parent,2214871u+i);EXPECT_EQ(p.family_index,392u+i);
            EXPECT_EQ(p.has_native_kinematics,step!=0);
            const auto& paired=record->candidate_parents[i];EXPECT_TRUE(paired.has_native_kinematics);
            EXPECT_EQ(paired.force.kinematics.base_time,record->base.time);
            EXPECT_EQ(paired.force.proposed_history.stamp().sample_index,record->enclosing.epoch);
            EXPECT_EQ(paired.force.internal_couple[p.local_node].x,Access::Accepted(probe).parents.qeph[p.family_index].internal_couple[p.local_node].x);
            const auto& original=before.parents.qeph[p.family_index];
            for(unsigned n=0;n<4;++n) {
                EXPECT_EQ(p.force.internal_couple[n].x,original.internal_couple[n].x);
                EXPECT_EQ(p.force.internal_couple[n].y,original.internal_couple[n].y);
                EXPECT_EQ(p.force.internal_couple[n].z,original.internal_couple[n].z);
            }
            for(unsigned k=0;k<12;++k)EXPECT_EQ(p.force.proposed_history.data().stabilization[k],original.proposed_history.data().stabilization[k]);
            EXPECT_EQ(p.section.cumulative_plastic_work_J,before.parents.qsection[p.family_index].cumulative_plastic_work_J);
        }
        EXPECT_EQ(record->assembly_couple_residual.x,0);EXPECT_EQ(record->assembly_couple_residual.y,0);EXPECT_EQ(record->assembly_couple_residual.z,0);
        SameAllocations(probe,allocation,host);EXPECT_EQ(plain.accepted_qeph_spin(),nullptr);
    }
    EXPECT_EQ(probe.diagnostics()->first_contact_epoch,42u);EXPECT_GT(probe.accepted_contact().diagnostics->resultant.value,0);
    const auto before=Access::Accepted(probe);const auto* retained=probe.accepted_qeph_spin();std::array<unsigned char,sizeof(*retained)> bytes;std::memcpy(bytes.data(),retained,bytes.size());
    for(bool phase:{false,true}) {
        const auto r=Access::RejectSpin(probe,phase);EXPECT_FALSE(r)<<r.message;
        SameAccepted(before,Access::Accepted(probe));EXPECT_EQ(probe.accepted_qeph_spin(),retained);
        EXPECT_EQ(std::memcmp(retained,bytes.data(),bytes.size()),0);SameAllocations(probe,allocation,host);
    }
    auto r=probe.Step();ASSERT_TRUE(r)<<r.message;r=plain.Step();ASSERT_TRUE(r)<<r.message;
    SameFields(Access::Accepted(plain).fields,Access::Accepted(probe).fields);
    SameParents(Access::Accepted(plain).parents,Access::Accepted(probe).parents);
    EXPECT_GT(probe.accepted_qeph_spin()->attempt,plain.diagnostics()->shells.qeph.attempt);
    EXPECT_EQ(probe.accepted_qeph_spin()->enclosing.epoch,65u);SameAllocations(probe,allocation,host);
}
}
}
