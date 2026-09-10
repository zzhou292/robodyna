#include "Fixture.h"
#include "qualification/source_assembly/SourceAssemblyFlightFixture.h"
#include <cstring>

namespace crash::cases::source_assembly_dynamics::test {
TEST_F(SourceAssemblyDynamicsCheck, ActualStartupAndExactBudgetsRejectWithoutPublishing) {
    const auto bindings=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto setup=PrepareWall(bindings);const auto config=SmokeConfig();SourceAssemblyWallCase run;
    const auto r=run.Initialize(bindings,setup,config);ASSERT_TRUE(r)<<r.message;
    ASSERT_NE(run.owner(),nullptr);ASSERT_NE(run.diagnostics(),nullptr);
    EXPECT_EQ(run.owner()->accepted().node_count,1030u);EXPECT_EQ(run.owner()->accepted().rigid_groups.group_count,6u);
    EXPECT_EQ(run.owner()->accepted().rigid_groups.member_count,76u);EXPECT_EQ(run.owner()->accepted().epoch,0u);
    EXPECT_EQ(run.bindings()->materials().parent_count(),915u);EXPECT_EQ(run.bindings()->materials().material_count(),6u);
    EXPECT_EQ(run.bindings()->materials().curve_count(),2u);EXPECT_EQ(run.accepted_contact().diagnostics,nullptr);
    EXPECT_FALSE(run.diagnostics()->has_interval);EXPECT_GT(run.diagnostics()->motion.after.native_total,0);
    const auto& sample=SourceAssemblyDynamicsTestAccess::Accepted(run);
    for(std::size_t n=0;n<1030;++n) {
        const auto& node=bindings.shells().nodes()[n];const double x[]{node.position.x,node.position.y,node.position.z};
        for(unsigned a=0;a<3;++a) {EXPECT_EQ(sample.fields.x[3*n+a],x[a]);EXPECT_EQ(sample.fields.v[3*n+a],a==0?8:0);}
        EXPECT_EQ(sample.fields.orientation[4*n],1);EXPECT_EQ(sample.fields.w[3*n],0);
    }
    const auto allocations=run.allocations();const auto host=run.host_payload_bytes();ASSERT_GT(host,1u);
    SourceAssemblyWallCase small;auto limits=config;limits.storage.max_nodes=128;
    EXPECT_EQ(small.Initialize(bindings,setup,limits).status,Status::ResourceLimit);EXPECT_FALSE(small.initialized());EXPECT_EQ(small.owner(),nullptr);
    limits=config;limits.storage.max_host_bytes=host-1;
    EXPECT_EQ(small.Initialize(bindings,setup,limits).status,Status::ResourceLimit);EXPECT_EQ(small.host_payload_bytes(),0u);
    EXPECT_EQ(small.allocations().device_bytes,0u);
    limits.storage.max_host_bytes=host;ASSERT_TRUE(small.Initialize(bindings,setup,limits));
    EXPECT_EQ(small.host_payload_bytes(),host);SameAllocations(run,allocations,host);
    auto other_options=source::test::Options();++other_options.source_instance_id;
    const auto other=source::SourceAssemblyBindings::Prepare(bindings.source(),other_options);SourceAssemblyWallCase wrong;
    EXPECT_EQ(wrong.Initialize(other,setup,config).status,Status::SourceMismatch);EXPECT_EQ(wrong.owner(),nullptr);
}
TEST_F(SourceAssemblyDynamicsCheck, SeparatedWallMatchesTheQualifiedGroupedSourceFlight) {
    namespace flight=qualification::source_assembly;
    const auto source_model=source::test::Load();flight::Rig reference(source_model,true);ASSERT_TRUE(reference.Initialize());
    const auto setup=PrepareWall(reference.bindings,.02);SourceAssemblyWallCase run;
    const auto initialized=run.Initialize(reference.bindings,setup,SmokeConfig());ASSERT_TRUE(initialized)<<initialized.message;
    for(unsigned i=0;i<3;++i) {
        flight::Prepared trial(reference.nodes());flight::ShellFields shells(reference.quads(),reference.triangles());
        ASSERT_TRUE(flight::Prepare(reference,trial));ASSERT_TRUE(flight::Evaluate(reference,trial,shells));
        ASSERT_TRUE(flight::Publish(reference,trial,shells));const auto report=run.Step();ASSERT_TRUE(report)<<report.message;
        flight::Fields actual(reference.nodes());ASSERT_TRUE(flight::Capture(reference,actual,shells));
        const auto& accepted=SourceAssemblyDynamicsTestAccess::Accepted(run);
        EXPECT_EQ(accepted.fields.x,actual.x);EXPECT_EQ(accepted.fields.v,actual.v);EXPECT_EQ(accepted.fields.w,actual.w);
        EXPECT_EQ(accepted.fields.orientation,actual.orientation);EXPECT_EQ(accepted.fields.reaction,actual.reaction);EXPECT_EQ(accepted.fields.couple,actual.couple);
        EXPECT_EQ(std::memcmp(accepted.parents.qeph.data(),shells.quad.data(),shells.quad.size()*sizeof(q::ForceTrial)),0);
        EXPECT_EQ(std::memcmp(accepted.parents.t3.data(),shells.triangle.data(),shells.triangle.size()*sizeof(t::ForceTrial)),0);
        EXPECT_EQ(std::memcmp(accepted.parents.qsection.data(),shells.qsection.data(),shells.qsection.size()*sizeof(fe::ShellBatchSectionState)),0);
        EXPECT_EQ(std::memcmp(accepted.parents.tsection.data(),shells.tsection.data(),shells.tsection.size()*sizeof(fe::ShellBatchSectionState)),0);
        EXPECT_EQ(run.accepted_contact().diagnostics->resultant.value,0);EXPECT_EQ(run.diagnostics()->active_contact_nodes,0u);
    }
}
TEST_F(SourceAssemblyDynamicsCheck, LateContactAndObservationRejectionsPreserveEveryAcceptedParticipant) {
    const auto bindings=source::SourceAssemblyBindings::Prepare(source::test::Load(),source::test::Options());
    const auto setup=PrepareWall(bindings);SourceAssemblyWallCase run,clean;
    auto r=run.Initialize(bindings,setup,SmokeConfig());ASSERT_TRUE(r)<<r.message;
    r=clean.Initialize(bindings,setup,SmokeConfig());ASSERT_TRUE(r)<<r.message;
    for(unsigned i=0;i<64;++i) {r=run.Step();ASSERT_TRUE(r)<<r.message;r=clean.Step();ASSERT_TRUE(r)<<r.message;}
    ASSERT_GT(run.diagnostics()->active_contact_nodes,0u);
    SourceAssemblyAcceptedOutput visible;
    ASSERT_EQ(visible.Initialize(*run.owner(),bindings,{run.owner()->accepted().owner_id,7,9},11).status,visual::Status::Ok);
    CheckOutput(run,visible);const auto* positions=visible.nodal()->fields().position_xyz;const auto* sections=visible.qeph().values;
    const auto stamp=*visible.nodal()->stamp();const auto before=SourceAssemblyDynamicsTestAccess::Accepted(run);
    const auto allocation=run.allocations();const auto host=run.host_payload_bytes();
    using Access=SourceAssemblyDynamicsTestAccess;
    for(auto fault:{Access::Fault::LastWallFace,Access::Fault::AppliedLoad}) {
        r=Access::RejectLate(run,fault);EXPECT_EQ(r.status,fault==Access::Fault::LastWallFace?Status::ComponentFailure:Status::ObservationFailure)<<r.message;
        SameAccepted(before,Access::Accepted(run));SameAllocations(run,allocation,host);
        EXPECT_FALSE(run.CaptureAccepted(visible));EXPECT_EQ(visible.nodal()->fields().position_xyz,positions);EXPECT_EQ(visible.qeph().values,sections);
        EXPECT_TRUE(fe::trial_identity::SameStamp(*visible.nodal()->stamp(),stamp));
    }
    r=run.Step();ASSERT_TRUE(r)<<r.message;r=clean.Step();ASSERT_TRUE(r)<<r.message;
    SameFields(Access::Accepted(run).fields,Access::Accepted(clean).fields);
    SameParents(Access::Accepted(run).parents,Access::Accepted(clean).parents);
    EXPECT_EQ(run.diagnostics()->stamp.epoch,65u);EXPECT_EQ(run.accepted_contact().diagnostics->resultant.value,clean.accepted_contact().diagnostics->resultant.value);
    EXPECT_EQ(run.diagnostics()->motion.after.effective_total,clean.diagnostics()->motion.after.effective_total);
    CheckOutput(run,visible);SameAllocations(run,allocation,host);
}
} // namespace crash::cases::source_assembly_dynamics::test
