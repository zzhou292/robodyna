#include "SourceAssemblyFlightFixture.h"
#include <cuda_runtime_api.h>
#include <cmath>
#include <cstring>

namespace crash::qualification::source_assembly {
namespace {
class SourceAssemblyGroups:public ::testing::Test {
    void SetUp() override {int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);}
};
using Groups=std::vector<fe::NodalRigidGroupSnapshot>;
bool AcceptedGroups(Rig& r,Groups& values) {
    values.resize(r.owner.rigid_groups().group_count);fe::NodalStamp stamp;
    const auto read=r.owner.CopyAcceptedRigidGroups({values.data(),values.size()},&stamp);
    EXPECT_EQ(read.status,fe::NodalStatus::Ok)<<read.message;
    EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,r.owner.accepted()));
    return read.status==fe::NodalStatus::Ok;
}
bool PreparedGroups(Rig& r,const Prepared& candidate,Groups& values) {
    values.resize(r.owner.rigid_groups().group_count);fe::NodalPreparedView view;
    const auto read=r.owner.CopyPreparedRigidGroups(candidate.token,{values.data(),values.size()},&view);
    EXPECT_EQ(read.status,fe::NodalStatus::Ok)<<read.message;
    EXPECT_TRUE(fe::trial_identity::SamePrepared(view,candidate.view));
    return read.status==fe::NodalStatus::Ok;
}
void SameGroups(const Groups& a,const Groups& b) {
    ASSERT_EQ(a.size(),b.size());
    for(std::size_t g=0;g<a.size();++g) {
        EXPECT_EQ(a[g].source_group_id,b[g].source_group_id);
        EXPECT_EQ(a[g].source_node_set_id,b[g].source_node_set_id);
        const auto& x=a[g].state;const auto& y=b[g].state;
        const tl::math::Vec3 av[]{x.center,x.velocity,x.omega},bv[]{y.center,y.velocity,y.omega};
        for(unsigned k=0;k<3;++k) {
            EXPECT_EQ(av[k].x,bv[k].x);EXPECT_EQ(av[k].y,bv[k].y);EXPECT_EQ(av[k].z,bv[k].z);
        }
        for(unsigned i=0;i<9;++i)EXPECT_EQ(x.principal_axes.v[i],y.principal_axes.v[i]);
    }
}
void CheckGroups(const Rig& r,const Fields& nodes,const Groups& values) {
    const auto& model=*r.bindings.rigid_groups();ASSERT_EQ(values.size(),model.group_count());
    ASSERT_EQ(values.size(),6u);EXPECT_EQ(model.member_count(),76u);
    const auto stamp=r.owner.accepted();
    EXPECT_TRUE(fe::SameRigidGroupInfo(stamp.rigid_groups,{r.bindings.source_instance_id(),6,76}));
    for(std::size_t g=0;g<values.size();++g) {
        const auto& source=model.groups()[g];const auto& state=values[g].state;
        EXPECT_EQ(values[g].source_group_id,source.source_group_id);
        EXPECT_EQ(values[g].source_node_set_id,source.source_node_set_id);
        EXPECT_TRUE(fe::rigid::ValidGroupState(state));
        // Reuse the established source-flight dimensional rounding coefficient.
        const long double error=2e-13L*(stamp.epoch+1);
        const long double cx=source.center.x+static_cast<long double>(Speed)*stamp.time;
        EXPECT_LE(std::abs(state.center.x-cx),error*(1+std::abs(cx)));
        EXPECT_LE(std::abs(state.center.y-static_cast<long double>(source.center.y)),error*(1+std::abs(source.center.y)));
        EXPECT_LE(std::abs(state.center.z-static_cast<long double>(source.center.z)),error*(1+std::abs(source.center.z)));
        EXPECT_LE(std::abs(state.velocity.x-Speed),error*(1+Speed));
        EXPECT_LE(std::abs(state.velocity.y),error*(1+Speed));
        EXPECT_LE(std::abs(state.velocity.z),error*(1+Speed));
        for(std::size_t k=0;k<source.member_count;++k) {
            const auto& member=model.members()[source.member_offset+k];
            const auto n=member.global_node;
            EXPECT_EQ(member.source_node_id,r.bindings.shells().nodes()[n].source_id);
            const double omega[]{state.omega.x,state.omega.y,state.omega.z};
            for(unsigned axis=0;axis<3;++axis)
                EXPECT_LE(std::abs(nodes.w[3*n+axis]-omega[axis]),error*(1+std::abs(omega[axis])));
        }
    }
}
}
TEST_F(SourceAssemblyGroups, AllSixActualSourceGroupsBindNativeMembersAndInitialShells) {
    auto r=std::make_unique<Rig>(src::test::Load(),true);ASSERT_TRUE(r->Initialize());
    ASSERT_NO_FATAL_FAILURE(CheckSource(*r));
    Fields nodes(r->nodes());ShellFields shells(r->quads(),r->triangles());Groups groups;
    ASSERT_TRUE(Capture(*r,nodes,shells));ASSERT_TRUE(AcceptedGroups(*r,groups));
    ASSERT_NO_FATAL_FAILURE(CheckFlight(*r,nodes,shells));
    ASSERT_NO_FATAL_FAILURE(CheckGroups(*r,nodes,groups));
    EXPECT_EQ(r->owner.allocations().device_allocations,7u);
    RecordProperty("scope","actual source grouped initialization; no contact or trajectory admission");
    RecordProperty("active_internal_groups",6);RecordProperty("active_group_members",76);
    RecordProperty("source_inventory_sha256",r->bindings.source().data().identity.sha256);
}
TEST_F(SourceAssemblyGroups, FourConnectedSourceIntervalsPublishAndRollbackEveryGroupAndShellTogether) {
    auto r=std::make_unique<Rig>(src::test::Load(),true);ASSERT_TRUE(r->Initialize());
    const auto allocated=Allocations(*r);
    Fields nodes(r->nodes());ShellFields shells(r->quads(),r->triangles());Groups accepted;
    ASSERT_TRUE(Capture(*r,nodes,shells));ASSERT_TRUE(AcceptedGroups(*r,accepted));
    for(unsigned step=0;step<4;++step) {
        SCOPED_TRACE(step);Prepared proposed(r->nodes());ShellFields candidate(r->quads(),r->triangles());Groups trial,held;
        ASSERT_TRUE(Prepare(*r,proposed));ASSERT_TRUE(Evaluate(*r,proposed,candidate));
        ASSERT_TRUE(PreparedGroups(*r,proposed,trial));ASSERT_NO_FATAL_FAILURE(CheckHostParents(*r,proposed,shells,candidate));
        ASSERT_TRUE(AcceptedGroups(*r,held));SameGroups(accepted,held);
        if(step==1) {
            const auto& d=candidate.diagnostics.qeph;
            EXPECT_EQ(r->publication.Commit(r->owner,proposed.token,candidate.diagnostics,
                {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,false}).status,fe::ShellPublicationStatus::StaleTrial);
            Fields still(r->nodes());ShellFields old(r->quads(),r->triangles());
            ASSERT_TRUE(Capture(*r,still,old));SameFields(nodes,still);SameShells(shells,old);
            ASSERT_TRUE(AcceptedGroups(*r,held));SameGroups(accepted,held);
            Prepared retry(r->nodes());ShellFields repeated(r->quads(),r->triangles());Groups retry_groups;
            ASSERT_TRUE(Prepare(*r,retry));ASSERT_TRUE(Evaluate(*r,retry,repeated));
            ASSERT_TRUE(PreparedGroups(*r,retry,retry_groups));SameGroups(trial,retry_groups);SameShells(candidate,repeated);
            EXPECT_EQ(proposed.endpoint.x,retry.endpoint.x);EXPECT_EQ(proposed.endpoint.v,retry.endpoint.v);
            EXPECT_EQ(proposed.endpoint.w,retry.endpoint.w);EXPECT_EQ(proposed.endpoint.orientation,retry.endpoint.orientation);
            ASSERT_TRUE(Publish(*r,retry,repeated));
        } else ASSERT_TRUE(Publish(*r,proposed,candidate));
        ASSERT_TRUE(Capture(*r,nodes,shells));SameShells(candidate,shells);
        ASSERT_TRUE(AcceptedGroups(*r,accepted));SameGroups(trial,accepted);
        EXPECT_EQ(nodes.stamp.epoch,step+1u);ASSERT_NO_FATAL_FAILURE(CheckFlight(*r,nodes,shells));
        ASSERT_NO_FATAL_FAILURE(CheckGroups(*r,nodes,accepted));SameAllocations(*r,allocated);
    }
    RecordProperty("scope","actual source connected free flight; raw native nodal kinetic diagnostic; no wall impact");
    RecordProperty("active_internal_groups",6);RecordProperty("active_group_members",76);
    RecordProperty("accepted_intervals",4);RecordProperty("host_parent_interval_checks",4*915);
    RecordProperty("rejected_complete_candidate_intervals",1);
}
}
