#include "SourceAssemblyFlightFixture.h"
#include "modelio/source_assembly/tests/AnalyticTestSupport.h"
#include <cuda_runtime_api.h>

namespace crash::qualification::source_assembly {
namespace {
class SourceAnalyticFlight : public ::testing::TestWithParam<bool> {
    void SetUp() override {
        int devices=0; ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess); ASSERT_GT(devices,0);
    }
};
void SourceContract(const Rig& r,bool mixed) {
    const auto& s=r.bindings.source().data(); const auto& c=r.bindings.materials();
    EXPECT_EQ(s.identity.sha256,src::test::analytic::Identity(mixed).sha256);
    EXPECT_EQ(s.schema,src::Law44InventorySchema);
    EXPECT_EQ(r.nodes(),mixed?538u:421u); EXPECT_EQ(r.quads(),mixed?446u:358u);
    EXPECT_EQ(r.triangles(),mixed?36u:30u); EXPECT_EQ(c.parent_count(),mixed?482u:388u);
    EXPECT_EQ(c.curve_count(),mixed?1u:0u); EXPECT_EQ(r.bindings.rigid_groups(),nullptr);
    EXPECT_EQ(r.owner.rigid_groups().group_count,0u);
    EXPECT_EQ(s.boundary.policy,"released_external_connections");
    EXPECT_EQ(s.boundary.spotweld_ids.size(),33u);
    for(const auto& parent:s.parents) {
        const auto* declared=c.parent(parent.index); ASSERT_NE(declared,nullptr);
        EXPECT_EQ(declared->source_parent_id,parent.source_id);
        EXPECT_EQ(declared->source_part_id,parent.part_id);
        EXPECT_EQ(declared->material_id,parent.material_id);
        EXPECT_EQ(declared->section_id,parent.section_id);
        fe::sections::PointParameters p;
        ASSERT_TRUE(c.Parameters(declared->family,declared->family_index,&p));
        const bool linear=parent.part_id==2000064;
        EXPECT_EQ(p.hardening,linear?tl::material::ShellPlasticityHardeningKind::LinearLaw44:
                                    tl::material::ShellPlasticityHardeningKind::Tabulated);
        EXPECT_TRUE(p.rate.enabled); EXPECT_EQ(p.rate.cutoff_hz,10000.);
        if(linear) { EXPECT_EQ(p.curve.count,0u); EXPECT_EQ(p.curve.plastic_strain,nullptr); }
        else { EXPECT_GT(p.curve.count,1u); EXPECT_NE(p.curve.plastic_strain,nullptr); }
    }
}
}
TEST_P(SourceAnalyticFlight, OriginalPureAndMixedCatalogsAdvanceBothFamiliesWithExactRetry) {
    const bool mixed=GetParam(); const auto source=src::test::analytic::Load(mixed);
    auto r=std::make_unique<Rig>(source); ASSERT_TRUE(r->Initialize());
    ASSERT_NO_FATAL_FAILURE(SourceContract(*r,mixed)); const auto allocated=Allocations(*r);
    Fields state(r->nodes()); ShellFields base(r->quads(),r->triangles());
    ASSERT_TRUE(Capture(*r,state,base)); ASSERT_NO_FATAL_FAILURE(CheckFlight(*r,state,base));
    for(unsigned step=0;step<4;++step) {
        SCOPED_TRACE(step); Prepared p(r->nodes()); ShellFields proposed(r->quads(),r->triangles());
        ASSERT_TRUE(Prepare(*r,p)); ASSERT_TRUE(Evaluate(*r,p,proposed));
        ASSERT_NO_FATAL_FAILURE(CheckHostParents(*r,p,base,proposed));
        Fields still(r->nodes()); ShellFields held(r->quads(),r->triangles());
        ASSERT_TRUE(Capture(*r,still,held)); SameFields(state,still); SameShells(base,held);
        if(step==1) {
            const auto& d=proposed.diagnostics.qeph;
            const auto rejected=r->publication.Commit(r->owner,p.token,proposed.diagnostics,
                {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,false});
            EXPECT_EQ(rejected.status,fe::ShellPublicationStatus::StaleTrial);
            ASSERT_TRUE(Capture(*r,still,held)); SameFields(state,still); SameShells(base,held);
            Prepared retry(r->nodes()); ShellFields repeated(r->quads(),r->triangles());
            ASSERT_TRUE(Prepare(*r,retry)); ASSERT_TRUE(Evaluate(*r,retry,repeated));
            SameShells(proposed,repeated); EXPECT_EQ(p.endpoint.x,retry.endpoint.x);
            EXPECT_EQ(p.endpoint.v,retry.endpoint.v); EXPECT_EQ(p.endpoint.w,retry.endpoint.w);
            EXPECT_EQ(p.endpoint.orientation,retry.endpoint.orientation);
            ASSERT_TRUE(Publish(*r,retry,repeated));
        } else ASSERT_TRUE(Publish(*r,p,proposed));
        ASSERT_TRUE(Capture(*r,state,held)); SameShells(proposed,held); base=held;
        EXPECT_EQ(state.stamp.epoch,step+1u);
        ASSERT_NO_FATAL_FAILURE(CheckFlight(*r,state,base)); SameAllocations(*r,allocated);
    }
    EXPECT_EQ(base.quad.back().proposed_history.stamp().sample_index,4u);
    EXPECT_EQ(base.triangle.back().proposed_history.stamp().sample_index,4u);
    RecordProperty("source_inventory_sha256",source.data().identity.sha256);
    RecordProperty("scope","original V2 source CUDA free flight and rollback; released interfaces; no wall trajectory");
    RecordProperty("source_parent_count",int(source.data().parents.size()));
    RecordProperty("host_parent_interval_checks",int(4*source.data().parents.size()));
}
INSTANTIATE_TEST_SUITE_P(ActualSource,SourceAnalyticFlight,::testing::Values(false,true));
} // namespace crash::qualification::source_assembly
