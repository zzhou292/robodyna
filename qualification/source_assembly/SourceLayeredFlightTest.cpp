#include "SourceAssemblyFlightFixture.h"
#include "modelio/source_assembly/tests/SectionTestSupport.h"
#include <cuda_runtime_api.h>

namespace crash::qualification::source_assembly {
namespace {
class SourceLayeredFlight : public ::testing::TestWithParam<bool> {
    void SetUp() override { int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0); }
};
void SourceContract(const Rig& r,bool mixed) {
    const auto& s=r.bindings.source().data();const auto& c=r.bindings.materials();
    EXPECT_EQ(s.identity.sha256,src::test::section::Identity(mixed).sha256);EXPECT_EQ(s.schema,src::SectionInventorySchema);
    EXPECT_EQ(r.nodes(),mixed?683u:145u);EXPECT_EQ(r.quads(),mixed?581u:135u);EXPECT_EQ(r.triangles(),mixed?50u:14u);
    EXPECT_EQ(c.parent_count(),mixed?631u:149u);EXPECT_TRUE(c.heterogeneous_sections());EXPECT_EQ(c.curve_count(),mixed?1u:0u);
    EXPECT_EQ(r.bindings.rigid_groups(),nullptr);EXPECT_EQ(r.owner.rigid_groups().group_count,0u);
    EXPECT_EQ(s.boundary.policy,"released_external_connections");EXPECT_EQ(s.boundary.nodal_rigid_ids.size(),mixed?10u:2u);
    EXPECT_EQ(s.boundary.spotweld_ids.size(),mixed?33u:0u);EXPECT_EQ(s.boundary.external_node_ids.size(),mixed?102u:2u);
    EXPECT_EQ(s.boundary.auxiliary.source_node_ids,(std::vector<src::SourceId>{2406582,2411580}));
    EXPECT_EQ(s.boundary.auxiliary.point_masses.size(),2u);
    fe::ShellSectionCounts qn,tn;ASSERT_TRUE(c.Counts(fe::ShellBindingFamily::Qeph,&qn));ASSERT_TRUE(c.Counts(fe::ShellBindingFamily::T3,&tn));
    EXPECT_EQ(qn.law1,135u);EXPECT_EQ(tn.law1,14u);EXPECT_EQ(qn.law44,mixed?446u:0u);EXPECT_EQ(tn.law44,mixed?36u:0u);
    for(std::size_t n=0;n<r.nodes();++n) {
        EXPECT_EQ(r.bindings.shells().nodes()[n].source_id,s.nodes[n].source_id);
        EXPECT_EQ(r.inverse_mass[n],1/r.bindings.shells().nodes()[n].native.mass);
        EXPECT_EQ(r.inverse_inertia[n],1/r.bindings.shells().nodes()[n].native.isotropic_inertia);
    }
}
void UnavailableLegacyReadback(Rig& r) {
    std::vector<fe::ShellBatchSectionState> qold(r.quads()),told(r.triangles());
    for(auto& h:qold)h.cumulative_plastic_work_J=73;
    for(auto& h:told)h.cumulative_plastic_work_J=91;
    q::BatchDiagnostics qd;t::BatchDiagnostics td;qd.owner_id=17;td.owner_id=19;
    EXPECT_EQ(r.qeph.CopyAcceptedSectionHistory(r.owner.accepted(),qold.data(),qold.size(),&qd).status,q::BatchStatus::InvalidInput);
    EXPECT_EQ(r.t3.CopyAcceptedSectionHistory(r.owner.accepted(),told.data(),told.size(),&td).status,t::BatchStatus::InvalidInput);
    for(const auto& h:qold)EXPECT_EQ(h.cumulative_plastic_work_J,73);
    for(const auto& h:told)EXPECT_EQ(h.cumulative_plastic_work_J,91);
    EXPECT_EQ(qd.owner_id,17u);EXPECT_EQ(td.owner_id,19u);
}
}
TEST_P(SourceLayeredFlight, OriginalElasticAndThreeLawSourceCollectionsAdvanceWithExactRetry) {
    const bool mixed=GetParam();const auto source=src::test::section::Load(mixed);
    auto r=std::make_unique<Rig>(source);ASSERT_TRUE(r->Initialize());ASSERT_NO_FATAL_FAILURE(SourceContract(*r,mixed));
    ASSERT_NO_FATAL_FAILURE(UnavailableLegacyReadback(*r));const auto allocated=Allocations(*r);
    Fields state(r->nodes());LayeredShellFields base(r->quads(),r->triangles());
    ASSERT_TRUE(Capture(*r,state,base));ASSERT_NO_FATAL_FAILURE(CheckFlight(*r,state,base));
    for(unsigned step=0;step<4;++step) {
        SCOPED_TRACE(step);Prepared p(r->nodes());LayeredShellFields candidate(r->quads(),r->triangles());
        ASSERT_TRUE(Prepare(*r,p));ASSERT_TRUE(Evaluate(*r,p,candidate));
        ASSERT_NO_FATAL_FAILURE(CheckHostParents(*r,p,base,candidate));
        Fields still(r->nodes());LayeredShellFields held(r->quads(),r->triangles());
        ASSERT_TRUE(Capture(*r,still,held));SameFields(state,still);SameShells(base,held);
        if(step==1) {
            const auto& d=candidate.diagnostics.qeph;
            const auto rejected=r->publication.Commit(r->owner,p.token,candidate.diagnostics,
                {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,false});
            EXPECT_EQ(rejected.status,fe::ShellPublicationStatus::StaleTrial);
            ASSERT_TRUE(Capture(*r,still,held));SameFields(state,still);SameShells(base,held);
            Prepared retry(r->nodes());LayeredShellFields repeated(r->quads(),r->triangles());
            ASSERT_TRUE(Prepare(*r,retry));ASSERT_TRUE(Evaluate(*r,retry,repeated));SameShells(candidate,repeated);
            EXPECT_EQ(p.endpoint.x,retry.endpoint.x);EXPECT_EQ(p.endpoint.v,retry.endpoint.v);
            EXPECT_EQ(p.endpoint.w,retry.endpoint.w);EXPECT_EQ(p.endpoint.orientation,retry.endpoint.orientation);
            ASSERT_TRUE(Publish(*r,retry,repeated));
        } else ASSERT_TRUE(Publish(*r,p,candidate));
        ASSERT_TRUE(Capture(*r,state,held));SameShells(candidate,held);base=held;
        EXPECT_EQ(state.stamp.epoch,step+1u);ASSERT_NO_FATAL_FAILURE(CheckFlight(*r,state,base));SameAllocations(*r,allocated);
    }
    RecordProperty("source_inventory_sha256",source.data().identity.sha256);
    RecordProperty("scope","original V3 CUDA free flight; complete selected shells and explicitly released auxiliary interfaces; no wall");
    RecordProperty("source_parent_count",int(source.data().parents.size()));
    RecordProperty("host_parent_interval_checks",int(4*source.data().parents.size()));
}
INSTANTIATE_TEST_SUITE_P(ActualSource,SourceLayeredFlight,::testing::Values(false,true));
} // namespace crash::qualification::source_assembly
