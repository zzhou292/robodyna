#include "output/source_assembly/SourceAssemblyAcceptedOutput.h"
#include "qualification/source_assembly/SourceAssemblyFlightFixture.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <cuda_runtime_api.h>
#include <cstring>

namespace crash::output::assembly::test {
namespace flight=crash::qualification::source_assembly;
namespace fe=tl::fea;
namespace {
class SourceAssemblyAcceptedOutputCheck:public ::testing::Test {
    void SetUp() override {int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);}
};
void Check(SourceAssemblyAcceptedOutput& output,flight::Rig& rig) {
    flight::Fields fields(rig.nodes());flight::ShellFields shells(rig.quads(),rig.triangles());
    ASSERT_TRUE(flight::Capture(rig,fields,shells));const auto* nodal=output.nodal();ASSERT_NE(nodal,nullptr);
    ASSERT_NE(nodal->stamp(),nullptr);EXPECT_TRUE(fe::trial_identity::SameStamp(*nodal->stamp(),fields.stamp));
    const auto actual=nodal->fields();ASSERT_EQ(actual.node_count,1030u);
    EXPECT_EQ(std::memcmp(actual.position_xyz,fields.x.data(),fields.x.size()*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(actual.velocity_xyz,fields.v.data(),fields.v.size()*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(actual.angular_velocity_xyz,fields.w.data(),fields.w.size()*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(actual.orientation_wxyz,fields.orientation.data(),fields.orientation.size()*sizeof(double)),0);
    ASSERT_EQ(output.qeph().count,804u);ASSERT_EQ(output.t3().count,111u);
    EXPECT_EQ(std::memcmp(output.qeph().values,shells.qsection.data(),shells.qsection.size()*sizeof(fe::ShellBatchSectionState)),0);
    EXPECT_EQ(std::memcmp(output.t3().values,shells.tsection.data(),shells.tsection.size()*sizeof(fe::ShellBatchSectionState)),0);
    for(std::size_t p=0;p<rig.quads();++p)EXPECT_EQ(output.qeph().reported_thickness_m[p],shells.quad[p].proposed_history.data().thickness);
    for(std::size_t p=0;p<rig.triangles();++p)EXPECT_EQ(output.t3().reported_thickness_m[p],shells.triangle[p].proposed_history.data().thickness);
    const auto& mesh=*nodal->surface().mesh();ASSERT_EQ(mesh.GetCoordsVertices().size(),1030u);ASSERT_EQ(mesh.GetIndicesVertices().size(),1719u);
    for(std::size_t n=0;n<1030;++n)for(unsigned c=0;c<3;++c)EXPECT_EQ(Bits(mesh.GetCoordsVertices()[n][c]),Bits(fields.x[3*n+c]));
    ASSERT_NE(output.parent_scalars(),nullptr);ASSERT_EQ(output.parent_scalars()->size(),915u);
    for(const auto& p:output.mapping()->parents()) {
        const auto s=p.family==source::ShellFamily::Qeph?output.qeph():output.t3();
        EXPECT_EQ(output.parent_scalars()->at(p.source_index).source_parent,p.element);
        EXPECT_EQ(output.parent_scalars()->at(p.source_index).value,s.values[p.family_index].diagnostics.maximum_plastic_strain);
    }
    ASSERT_NE(output.diagnostics(),nullptr);
    // Owner publication is the only energy producer; compare diagnostics as supplied.
    EXPECT_EQ(output.diagnostics()->kinetic.translation,shells.diagnostics.kinetic.translation);
    EXPECT_EQ(output.diagnostics()->kinetic.rotation,shells.diagnostics.kinetic.rotation);
    EXPECT_EQ(output.diagnostics()->qeph.internal_work[0],shells.diagnostics.qeph.internal_work[0]);
    EXPECT_EQ(output.diagnostics()->t3.internal_work[1],shells.diagnostics.t3.internal_work[1]);
}
}
TEST_F(SourceAssemblyAcceptedOutputCheck, ActualFullSourceAcceptedFieldsExcludeCompletedAndRejectedCandidates) {
    RecordProperty("scope","Actual source accepted-output/free-flight contract with internal groups inactive and active; no wall trajectory or video");
    for(bool grouped:{false,true}) {
        SCOPED_TRACE(grouped ? "six internal groups active" : "groups inactive");
        auto rig=std::make_unique<flight::Rig>(source::test::Load(),grouped);ASSERT_TRUE(rig->Initialize());
        const auto allocations=flight::Allocations(*rig);SourceAssemblyAcceptedOutput output;
        const visual::Identity identity{rig->owner.accepted().owner_id,7,19};
        ASSERT_EQ(output.Initialize(rig->owner,rig->bindings,identity,5).status,visual::Status::Ok);
        EXPECT_EQ(output.nodal()->stamp(),nullptr);EXPECT_EQ(output.qeph().values,nullptr);EXPECT_EQ(output.parent_scalars(),nullptr);
        ASSERT_EQ(output.Publish(rig->owner,rig->qeph,rig->t3,rig->publication).status,visual::Status::Ok);Check(output,*rig);
        const auto bytes=output.section_host_bytes(),nodal_bytes=output.nodal()->capture_bytes();
        const auto* first_nodes=output.nodal()->fields().position_xyz;const auto* first_sections=output.qeph().values;
        for(unsigned step=0;step<3;++step) {
            flight::Prepared candidate(rig->nodes());flight::ShellFields proposed(rig->quads(),rig->triangles());
            ASSERT_TRUE(flight::Prepare(*rig,candidate));ASSERT_TRUE(flight::Evaluate(*rig,candidate,proposed));
            const auto stamp=*output.nodal()->stamp();const auto* held=output.qeph().values;
            EXPECT_EQ(output.Publish(rig->owner,rig->qeph,rig->t3,rig->publication).status,visual::Status::StaleFrame);
            EXPECT_EQ(output.qeph().values,held);EXPECT_TRUE(fe::trial_identity::SameStamp(*output.nodal()->stamp(),stamp));
            if(step==0) {
                const auto& d=proposed.diagnostics.qeph;
                EXPECT_EQ(rig->publication.Commit(rig->owner,candidate.token,proposed.diagnostics,
                    {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,false}).status,fe::ShellPublicationStatus::StaleTrial);
                EXPECT_EQ(output.Publish(rig->owner,rig->qeph,rig->t3,rig->publication).status,visual::Status::StaleFrame);Check(output,*rig);
                ASSERT_TRUE(flight::Prepare(*rig,candidate));ASSERT_TRUE(flight::Evaluate(*rig,candidate,proposed));
            }
            ASSERT_TRUE(flight::Publish(*rig,candidate,proposed));
            if(step==0) {
                fe::t3::T3Batch missing;
                EXPECT_EQ(output.Publish(rig->owner,rig->qeph,missing,rig->publication).status,visual::Status::InvalidFrame);
                EXPECT_EQ(output.qeph().values,held);EXPECT_TRUE(fe::trial_identity::SameStamp(*output.nodal()->stamp(),stamp));
                EXPECT_EQ(output.nodal()->fields().position_xyz,first_nodes);
            }
            ASSERT_EQ(output.Publish(rig->owner,rig->qeph,rig->t3,rig->publication).status,visual::Status::Ok);Check(output,*rig);
            EXPECT_EQ(output.nodal()->stamp()->epoch,step+1u);EXPECT_EQ(output.section_host_bytes(),bytes);
            EXPECT_EQ(output.nodal()->capture_bytes(),nodal_bytes);flight::SameAllocations(*rig,allocations);
            if(step==1) {
                EXPECT_EQ(output.nodal()->fields().position_xyz,first_nodes);EXPECT_EQ(output.qeph().values,first_sections);
            }
        }
        fe::FENodalState foreign;const auto* fields=output.qeph().values;
        EXPECT_EQ(output.Publish(foreign,rig->qeph,rig->t3,rig->publication).status,visual::Status::WrongOwner);
        EXPECT_EQ(output.qeph().values,fields);EXPECT_EQ(output.nodal()->stamp()->epoch,3u);
    }
}
TEST_F(SourceAssemblyAcceptedOutputCheck, ExplicitCaptureCapsRejectBeforePublishingAndAllowCompleteRetry) {
    auto rig=std::make_unique<flight::Rig>(source::test::Load());ASSERT_TRUE(rig->Initialize());
    const visual::Identity identity{rig->owner.accepted().owner_id,7,19};
    const auto surface=SourceAssemblySurface::Prepare(rig->bindings.source(),identity,5,rig->bindings.source_instance_id());
    visual::NodalMeshOutput nodal;
    EXPECT_EQ(nodal.Initialize(rig->owner,surface.binding(),visual::NodalOutputTiming::StaggeredHalfKick).status,visual::Status::ResourceLimit);
    EXPECT_EQ(nodal.surface().binding(),nullptr);EXPECT_EQ(nodal.capture_bytes(),0u);
    const auto required=1030u*26*sizeof(double);
    EXPECT_EQ(nodal.Initialize(rig->owner,surface.binding(),visual::NodalOutputTiming::StaggeredHalfKick,{2048,required-1}).status,
        visual::Status::ResourceLimit);EXPECT_EQ(nodal.surface().binding(),nullptr);
    ASSERT_EQ(nodal.Initialize(rig->owner,surface.binding(),visual::NodalOutputTiming::StaggeredHalfKick,{2048,required}).status,visual::Status::Ok);
    ASSERT_EQ(nodal.Publish(rig->owner).status,visual::Status::Ok);EXPECT_EQ(nodal.capture_bytes(),required);
    SourceAssemblyAcceptedOutput output;AcceptedOutputLimits limits;limits.max_section_host_bytes=1;
    EXPECT_EQ(output.Initialize(rig->owner,rig->bindings,identity,5,limits).status,visual::Status::ResourceLimit);
    EXPECT_EQ(output.mapping(),nullptr);EXPECT_EQ(output.section_host_bytes(),0u);
    limits={};ASSERT_EQ(output.Initialize(rig->owner,rig->bindings,identity,5,limits).status,visual::Status::Ok);
    const auto exact=output.section_host_bytes();SourceAssemblyAcceptedOutput second;limits.max_section_host_bytes=exact-1;
    EXPECT_EQ(second.Initialize(rig->owner,rig->bindings,identity,5,limits).status,visual::Status::ResourceLimit);
    EXPECT_EQ(second.mapping(),nullptr);limits.max_section_host_bytes=exact;
    ASSERT_EQ(second.Initialize(rig->owner,rig->bindings,identity,5,limits).status,visual::Status::Ok);
    ASSERT_EQ(second.Publish(rig->owner,rig->qeph,rig->t3,rig->publication).status,visual::Status::Ok);Check(second,*rig);
    // Actual constrained owner startup with identical source geometry/native M/J
    // but a different immutable source instance cannot borrow this output scope.
    auto other_options=cases::source_assembly::test::Options();++other_options.source_instance_id;
    const auto other=cases::source_assembly::SourceAssemblyBindings::Prepare(rig->bindings.source(),other_options);
    ASSERT_NE(other.rigid_groups(),nullptr);fe::FENodalState constrained;
    fe::NodalStateConfig config;config.node_count=rig->nodes();config.fixed_dt=flight::TimeStep;
    config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    ASSERT_EQ(constrained.Initialize(config,{rig->initial.x.data(),rig->initial.v.data(),rig->initial.w.data(),
        rig->nodes(),rig->initial.orientation.data()},rig->inverse_mass.data(),
        {rig->free.data(),rig->free.data(),rig->inverse_inertia.data()},*other.rigid_groups()).status,fe::NodalStatus::Ok);
    EXPECT_EQ(constrained.accepted().rigid_groups.group_count,6u);EXPECT_EQ(constrained.accepted().rigid_groups.member_count,76u);
    SourceAssemblyAcceptedOutput mismatched;
    EXPECT_EQ(mismatched.Initialize(constrained,rig->bindings,{constrained.accepted().owner_id,7,19},5).status,
        visual::Status::InvalidBinding);
    EXPECT_EQ(mismatched.mapping(),nullptr);EXPECT_EQ(mismatched.nodal(),nullptr);
}
} // namespace crash::output::assembly::test
