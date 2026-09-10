#include "Fixture.h"
#include "case/source_assembly/tests/SourceBracketTestSupport.h"
#include "lib_utest/qualification/type25/EvaluationValues.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::source_assembly_dynamics::test {
namespace {
using Spring=fe::type25::Evaluation;
void SameSpring(const Spring& a,const Spring& b) {
    const auto x=type25_test::EvaluationValues(a),y=type25_test::EvaluationValues(b);
    for(std::size_t i=0;i<x.size();++i)EXPECT_EQ(output::Bits(x[i]),output::Bits(y[i]));
    EXPECT_EQ(a.history.active,b.history.active);
}
}
TEST_F(SourceAssemblyDynamicsCheck, ActualSevenPartCommonPublicationRetainsWeldAndOptionalKineticCapture) {
    const auto bindings=source::test::BracketBindings();const auto setup=PrepareWall(bindings);
    auto config=SmokeConfig();config.fixed_dt=4*Dt;config.observe_force_stage=true;
    SourceAssemblyWallCase run;const auto initialized=run.Initialize(bindings,setup,config);
    ASSERT_TRUE(initialized)<<initialized.message;
    const auto allocations=run.allocations();const auto host=run.host_payload_bytes();
    ASSERT_EQ(run.diagnostics()->stamp.node_count,1093u);ASSERT_EQ(run.accepted_connectors().count,1u);
    const auto* initial=run.accepted_connectors().diagnostics;ASSERT_NE(initial,nullptr);
    EXPECT_EQ(initial->phase,fe::type25::BatchPhase::Accepted);EXPECT_EQ(initial->epoch,0u);
    EXPECT_EQ(initial->active_count,1u);EXPECT_EQ(run.diagnostics()->motion.after.connector.translation,.032);
    double peak_force=0,peak_couple=0;
    for(unsigned step=0;step<512;++step) {
        const auto r=run.Step();ASSERT_TRUE(r)<<"step="<<step<<" "<<r.message<<" source="<<r.source_parent;
        const auto view=run.accepted_connectors();ASSERT_EQ(view.count,1u);ASSERT_NE(view.diagnostics,nullptr);
        const auto& d=*view.diagnostics;const auto& stamp=run.diagnostics()->stamp;
        EXPECT_EQ(d.phase,fe::type25::BatchPhase::Accepted);EXPECT_EQ(d.epoch,stamp.epoch);EXPECT_EQ(d.time,stamp.time);
        EXPECT_EQ(d.source_instance_id,bindings.source_instance_id());EXPECT_TRUE(d.has_completed_interval);
        EXPECT_TRUE(d.accepted_force_assembled);EXPECT_EQ(d.element_count,1u);EXPECT_EQ(d.active_count,1u);
        const auto& v=view.elements[0];
        for(const auto& w:v.endpoints) {
            peak_force=std::max(peak_force,std::hypot(std::hypot(w.force_N.x,w.force_N.y),w.force_N.z));
            peak_couple=std::max(peak_couple,std::hypot(std::hypot(w.couple_Nm.x,w.couple_Nm.y),w.couple_Nm.z));
        }
        EXPECT_GT(v.critical_dt_s,config.fixed_dt/config.deformation.maximum_native_dt_fraction);
        ASSERT_NE(run.accepted_force_stage(),nullptr);EXPECT_GT(run.accepted_force_stage()->connector.translation,0);
        EXPECT_GE(run.diagnostics()->motion.after.connector.rotation,0);SameAllocations(run,allocations,host);
    }
    EXPECT_GT(run.diagnostics()->contact_intervals,0u);EXPECT_GT(peak_force,0);
    RecordProperty("peak_original_weld_force_N",source::test::Number(peak_force));
    RecordProperty("peak_original_weld_couple_Nm",source::test::Number(peak_couple));
}
TEST_F(SourceAssemblyDynamicsCheck, ConnectorLateFailurePreservesEveryAcceptedParticipantAndExactRetry) {
    const auto bindings=source::test::BracketBindings();const auto setup=PrepareWall(bindings);
    SourceAssemblyWallCase tested,baseline;ASSERT_TRUE(tested.Initialize(bindings,setup,SmokeConfig()));
    ASSERT_TRUE(baseline.Initialize(bindings,setup,SmokeConfig()));
    for(unsigned step=0;step<64;++step){ASSERT_TRUE(tested.Step());ASSERT_TRUE(baseline.Step());}
    for(const auto fault:{SourceAssemblyDynamicsTestAccess::ConnectorFault::Work,
                         SourceAssemblyDynamicsTestAccess::ConnectorFault::Phase,
                         SourceAssemblyDynamicsTestAccess::ConnectorFault::LastForce}) {
        const auto saved=SourceAssemblyDynamicsTestAccess::Accepted(tested);const Spring spring=tested.accepted_connectors().elements[0];
        const auto stamp=tested.diagnostics()->stamp;const auto r=SourceAssemblyDynamicsTestAccess::RejectConnector(tested,fault);
        EXPECT_FALSE(r);EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,tested.diagnostics()->stamp));
        SameAccepted(saved,SourceAssemblyDynamicsTestAccess::Accepted(tested));SameSpring(spring,tested.accepted_connectors().elements[0]);
        ASSERT_TRUE(tested.Step());ASSERT_TRUE(baseline.Step());
        SameFields(SourceAssemblyDynamicsTestAccess::Accepted(tested).fields,SourceAssemblyDynamicsTestAccess::Accepted(baseline).fields);
        SameParents(SourceAssemblyDynamicsTestAccess::Accepted(tested).parents,SourceAssemblyDynamicsTestAccess::Accepted(baseline).parents);
        SameSpring(tested.accepted_connectors().elements[0],baseline.accepted_connectors().elements[0]);
    }
}
TEST_F(SourceAssemblyDynamicsCheck, ConnectorSourceAndBudgetRejectBeforeOwnerAllocation) {
    const auto bindings=source::test::BracketBindings();const auto setup=PrepareWall(bindings);
    auto options=source::test::BracketOptions();++options.spotweld->generated_property_id;
    const auto other=source::SourceAssemblyBindings::Prepare(bindings.source(),options);
    SourceAssemblyWallCase wrong;EXPECT_EQ(wrong.Initialize(other,setup,SmokeConfig()).status,Status::SourceMismatch);
    EXPECT_EQ(wrong.owner(),nullptr);EXPECT_EQ(wrong.allocations().device_allocations,0u);
    auto config=SmokeConfig();config.storage.connector.max_connections=0;
    SourceAssemblyWallCase capped;EXPECT_EQ(capped.Initialize(bindings,setup,config).status,Status::ResourceLimit);
    EXPECT_EQ(capped.owner(),nullptr);EXPECT_EQ(capped.allocations().device_allocations,0u);
    config=SmokeConfig();config.storage.max_host_bytes=1024;
    SourceAssemblyWallCase host;EXPECT_EQ(host.Initialize(bindings,setup,config).status,Status::ResourceLimit);
    EXPECT_EQ(host.owner(),nullptr);EXPECT_EQ(host.allocations().device_allocations,0u);
}
} // namespace crash::cases::source_assembly_dynamics::test
