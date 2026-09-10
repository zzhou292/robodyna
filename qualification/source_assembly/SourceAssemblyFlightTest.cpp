#include "SourceAssemblyFlightFixture.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include "lib_src/elements/t3/T3History.h"
#include <cuda_runtime_api.h>

namespace crash::qualification::source_assembly {
namespace {
class SourceAssemblyFlight:public ::testing::Test {
    void SetUp() override {int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);}
};
void RecordSource(const Rig& r) {
    ::testing::Test::RecordProperty("scope","actual source CUDA storage/free flight; internal rigid groups inactive; no contact trajectory");
    ::testing::Test::RecordProperty("source_inventory_sha256",r.bindings.source().data().identity.sha256);
    ::testing::Test::RecordProperty("source_nodes",int(r.nodes()));
    ::testing::Test::RecordProperty("source_qeph_parents",int(r.quads()));
    ::testing::Test::RecordProperty("source_t3_parents",int(r.triangles()));
    ::testing::Test::RecordProperty("source_internal_groups_retained",6);
    ::testing::Test::RecordProperty("active_internal_groups",0);
    ::testing::Test::RecordProperty("initial_speed_m_s",8);
    ::testing::Test::RecordProperty("native_mass_kg",cases::source_assembly::test::Number(r.bindings.shells().totals().mass));
    const auto allocations=Allocations(r);
    for(unsigned i=0;i<allocations.size();++i)
        ::testing::Test::RecordProperty("component_"+std::to_string(i)+"_device_bytes",std::to_string(allocations[i].device_bytes));
}
}
TEST_F(SourceAssemblyFlight, AuthenticatedFullSourceCatalogAndNativeMassReachOneCudaOwner) {
    const auto source=src::test::Load();auto r=std::make_unique<Rig>(source);ASSERT_TRUE(r->Initialize());
    ASSERT_NO_FATAL_FAILURE(CheckSource(*r));
    Fields initial(r->nodes());ShellFields shells(r->quads(),r->triangles());ASSERT_TRUE(Capture(*r,initial,shells));
    EXPECT_EQ(initial.x,r->initial.x);EXPECT_EQ(initial.v,r->initial.v);EXPECT_EQ(initial.w,r->initial.w);
    EXPECT_EQ(initial.orientation,r->initial.orientation);ASSERT_NO_FATAL_FAILURE(CheckFlight(*r,initial,shells));
    for(std::size_t e=0;e<r->quads();++e) {
        SCOPED_TRACE(r->bindings.shells().qeph_source_id(e));q::History expected;
        const auto& reference=r->bindings.shells().qeph_reference(e);
        ASSERT_EQ(q::InitializeHistory(reference,{0,0},expected),q::Status::kSuccess);
        EXPECT_TRUE(shells.quad[e].proposed_history.matches_reference(reference));
        EXPECT_EQ(std::memcmp(&shells.quad[e].proposed_history.data(),&expected.data(),sizeof(q::HistoryValues)),0);
        for(unsigned n=0;n<4;++n) {
            EXPECT_EQ(shells.quad[e].internal_force[n].x,0);EXPECT_EQ(shells.quad[e].internal_force[n].y,0);EXPECT_EQ(shells.quad[e].internal_force[n].z,0);
            EXPECT_EQ(shells.quad[e].internal_couple[n].x,0);EXPECT_EQ(shells.quad[e].internal_couple[n].y,0);EXPECT_EQ(shells.quad[e].internal_couple[n].z,0);
        }
    }
    for(std::size_t e=0;e<r->triangles();++e) {
        SCOPED_TRACE(r->bindings.shells().t3_source_id(e));t::History expected;
        const auto& reference=r->bindings.shells().t3_reference(e);
        ASSERT_EQ(t::InitializeHistory(reference,{0,0},expected),t::Status::kSuccess);
        EXPECT_TRUE(shells.triangle[e].proposed_history.matches_reference(reference));
        EXPECT_EQ(std::memcmp(&shells.triangle[e].proposed_history.data(),&expected.data(),sizeof(t::HistoryValues)),0);
        for(unsigned n=0;n<3;++n) {
            EXPECT_EQ(shells.triangle[e].internal_force[n].x,0);EXPECT_EQ(shells.triangle[e].internal_force[n].y,0);EXPECT_EQ(shells.triangle[e].internal_force[n].z,0);
            EXPECT_EQ(shells.triangle[e].internal_couple[n].x,0);EXPECT_EQ(shells.triangle[e].internal_couple[n].y,0);EXPECT_EQ(shells.triangle[e].internal_couple[n].z,0);
        }
    }
    for(const auto* family:{&shells.qsection,&shells.tsection})for(const auto& section:*family)
        for(const auto& point:section.history.point) {
            for(double stress:point.stress)EXPECT_EQ(stress,0);
            EXPECT_EQ(point.filtered_rate_per_s,0);EXPECT_EQ(point.plastic_strain,0);
        }
    const auto allocated=Allocations(*r);EXPECT_EQ(allocated[0].device_allocations,6u);
    EXPECT_EQ(allocated[1].device_allocations,2u);EXPECT_EQ(allocated[2].device_allocations,2u);EXPECT_EQ(allocated[3].device_allocations,1u);
    RecordSource(*r);
}
TEST_F(SourceAssemblyFlight, FourActualSourceIntervalsCheckEveryHostHistoryAndExactRejectedRetry) {
    const auto source=src::test::Load();auto r=std::make_unique<Rig>(source);ASSERT_TRUE(r->Initialize());
    ASSERT_NO_FATAL_FAILURE(CheckSource(*r));const auto allocated=Allocations(*r);
    Fields state(r->nodes());ShellFields base(r->quads(),r->triangles());ASSERT_TRUE(Capture(*r,state,base));
    for(unsigned step=0;step<4;++step) {
        SCOPED_TRACE(step);Prepared p(r->nodes());ShellFields proposed(r->quads(),r->triangles());
        ASSERT_TRUE(Prepare(*r,p));ASSERT_TRUE(Evaluate(*r,p,proposed));
        ASSERT_NO_FATAL_FAILURE(CheckHostParents(*r,p,base,proposed));
        Fields still(r->nodes());ShellFields held(r->quads(),r->triangles());ASSERT_TRUE(Capture(*r,still,held));
        SameFields(state,still);SameShells(base,held);
        if(step==1) {
            const auto& d=proposed.diagnostics.qeph;
            const auto rejected=r->publication.Commit(r->owner,p.token,proposed.diagnostics,
                {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,false});
            EXPECT_EQ(rejected.status,fe::ShellPublicationStatus::StaleTrial);
            ASSERT_TRUE(Capture(*r,still,held));SameFields(state,still);SameShells(base,held);
            Prepared retry(r->nodes());ShellFields repeated(r->quads(),r->triangles());
            ASSERT_TRUE(Prepare(*r,retry));ASSERT_TRUE(Evaluate(*r,retry,repeated));
            SameShells(proposed,repeated);EXPECT_EQ(p.endpoint.x,retry.endpoint.x);EXPECT_EQ(p.endpoint.v,retry.endpoint.v);
            EXPECT_EQ(p.endpoint.w,retry.endpoint.w);EXPECT_EQ(p.endpoint.orientation,retry.endpoint.orientation);
            ASSERT_TRUE(Publish(*r,retry,repeated));
        } else ASSERT_TRUE(Publish(*r,p,proposed));
        ASSERT_TRUE(Capture(*r,state,held));SameShells(proposed,held);base=held;
        EXPECT_EQ(state.stamp.epoch,step+1u);ASSERT_NO_FATAL_FAILURE(CheckFlight(*r,state,base));SameAllocations(*r,allocated);
        EXPECT_EQ(r->bindings.source().data().identity.sha256,src::PinnedYarisSixPartInventory().sha256);
    }
    // These are the original final source parents, not selected demo cells.
    EXPECT_EQ(base.quad.back().proposed_history.stamp().sample_index,4u);
    EXPECT_EQ(base.triangle.back().proposed_history.stamp().sample_index,4u);
    RecordSource(*r);RecordProperty("accepted_intervals",4);RecordProperty("host_parent_interval_checks",4*915);
    RecordProperty("rejected_complete_candidate_intervals",1);
}
}
