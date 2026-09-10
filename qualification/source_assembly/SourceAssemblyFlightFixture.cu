#include "SourceAssemblyFlightFixture.h"
#include "SourceAssemblyFlightPackets.h"
#include <cuda_runtime.h>

namespace crash::qualification::source_assembly {
Rig::Rig(const src::SourceAssembly& source,bool attach_groups)
    :bindings(SourceAssemblyBindings::Prepare(source,cases::source_assembly::test::Options())),
     groups_attached(attach_groups),
     initial(nodes()),inverse_mass(nodes()),inverse_inertia(nodes()),free(nodes()) {}
bool Rig::Initialize() {
    const auto geometry=contact_geometry.Initialize(bindings.shells());
    EXPECT_TRUE(geometry)<<geometry.message; if(!geometry)return false;
    for(std::size_t n=0;n<nodes();++n) {
        const auto& node=bindings.shells().nodes()[n];
        initial.x[3*n]=node.position.x;initial.x[3*n+1]=node.position.y;initial.x[3*n+2]=node.position.z;
        initial.v[3*n]=Speed;initial.orientation[4*n]=1;
        inverse_mass[n]=1/node.native.mass;inverse_inertia[n]=1/node.native.isotropic_inertia;
    }
    fe::NodalStateConfig config;config.node_count=nodes();config.fixed_dt=TimeStep;
    config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    const fe::HostNodalKinematicsView startup{initial.x.data(),initial.v.data(),initial.w.data(),nodes(),initial.orientation.data()};
    const fe::NodalDofConfig dofs{free.data(),free.data(),inverse_inertia.data()};
    if(groups_attached&&!bindings.rigid_groups())return false;
    const auto initialized=groups_attached?
        owner.Initialize(config,startup,inverse_mass.data(),dofs,*bindings.rigid_groups()):
        owner.Initialize(config,startup,inverse_mass.data(),dofs);
    EXPECT_EQ(initialized.status,fe::NodalStatus::Ok)<<initialized.message;
    if(initialized.status!=fe::NodalStatus::Ok)return false;
    q::QephBatchConfig qc;qc.owner=owner.accepted();qc.element_count=quads();
    qc.configuration_id=Configuration;qc.qualification_id=Qualification;qc.usage=q::BatchUsage::CoupledForces;
    qc.storage_limits.max_parents=fe::MaxShellResidentParents;qc.storage_limits.max_nodes=fe::MaxShellResidentNodes;
    qc.max_device_bytes=4*1024*1024;qc.startup={fe::ShellBatchStartupKind::ReferenceUniformTranslation,{Speed,0,0}};
    t::T3BatchConfig tc;tc.owner=qc.owner;tc.element_count=triangles();
    tc.configuration_id=Configuration;tc.qualification_id=Qualification;tc.usage=t::BatchUsage::CoupledForces;
    tc.storage_limits=qc.storage_limits;tc.max_device_bytes=qc.max_device_bytes;tc.startup=qc.startup;
    const auto qr=qeph.InitializeJoined(qc,bindings.shells(),bindings.materials());
    const auto tr=t3.InitializeJoined(tc,bindings.shells(),bindings.materials());
    EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
    if(qr.status!=q::BatchStatus::Success||tr.status!=t::BatchStatus::Success)return false;
    fe::NodalTrialToken token;fe::NodalAssemblyView view;
    EXPECT_EQ(owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    std::vector<double> actual_inverse(nodes()),actual_j(nodes());
    EXPECT_EQ(cudaMemcpyAsync(actual_inverse.data(),view.mass.inverse_mass,nodes()*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
    EXPECT_EQ(cudaMemcpyAsync(actual_j.data(),view.inverse_inertia,nodes()*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
    EXPECT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
    EXPECT_EQ(actual_inverse,inverse_mass);EXPECT_EQ(actual_j,inverse_inertia);
    // Moving startup must authenticate actual buffers through the live owner.
    EXPECT_EQ(qeph.AssembleAccepted(owner,view).status,q::BatchStatus::Success);
    EXPECT_EQ(t3.AssembleAccepted(owner,view).status,t::BatchStatus::Success);Discard();
    if(::testing::Test::HasFailure())return false;
    fe::ShellPublicationLimits limits;limits.max_nodes=fe::MaxShellResidentNodes;
    const auto joined=publication.Initialize(owner,qeph,t3,limits);
    EXPECT_EQ(joined.status,fe::ShellPublicationStatus::Success)<<joined.message;
    return joined.status==fe::ShellPublicationStatus::Success;
}
bool Prepare(Rig& r,Prepared& p) {
    fe::NodalAssemblyView assembly;
    EXPECT_EQ(r.owner.BeginTrial(&p.token,&assembly).status,fe::NodalStatus::Ok);
    EXPECT_EQ(r.qeph.AssembleAccepted(r.owner,assembly).status,q::BatchStatus::Success);
    EXPECT_EQ(r.t3.AssembleAccepted(r.owner,assembly).status,t::BatchStatus::Success);
    // Group enforcement, when attached, is inside the same owner advance.
    // No contact or external force is assembled in this short free-flight gate.
    EXPECT_EQ(r.owner.SealAssembly(p.token).status,fe::NodalStatus::Ok);
    const fe::NodalStaggeredHistoryAdmission admission{
        assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,TimeStep,1,Qualification};
    const auto advanced=r.groups_attached?fe::AdvanceStaggeredRigidGroups(r.owner,p.token,admission):
        fe::AdvanceStaggeredHistory(r.owner,p.token,admission);
    EXPECT_EQ(advanced.status,fe::NodalStatus::Ok)<<advanced.message;
    EXPECT_EQ(r.owner.BorrowPrepared(p.token,&p.view).status,fe::NodalStatus::Ok);
    if(::testing::Test::HasFailure())return false;
    const auto bytes=3*r.nodes()*sizeof(double);const auto& v=p.view;
    EXPECT_EQ(cudaMemcpyAsync(p.endpoint.x.data(),v.kinematics.position_xyz,bytes,cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
    EXPECT_EQ(cudaMemcpyAsync(p.endpoint.v.data(),v.kinematics.velocity_xyz,bytes,cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
    EXPECT_EQ(cudaMemcpyAsync(p.endpoint.w.data(),v.kinematics.angular_velocity_xyz,bytes,cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
    EXPECT_EQ(cudaMemcpyAsync(p.endpoint.orientation.data(),v.kinematics.orientation_wxyz,4*r.nodes()*sizeof(double),cudaMemcpyDeviceToHost,v.stream),cudaSuccess);
    EXPECT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess);return !::testing::Test::HasFailure();
}
bool Capture(Rig& r,Fields& state,ShellFields& shells) { return packets::CapturePackets(r,state,shells); }
bool Evaluate(Rig& r,const Prepared& p,ShellFields& shells) { return packets::EvaluatePackets(r,p,shells); }
bool Publish(Rig& r,const Prepared& p,const ShellFields& shells) { return packets::PublishPackets(r,p,shells); }
bool Capture(Rig& r,Fields& state,LayeredShellFields& shells) { return packets::CapturePackets(r,state,shells); }
bool Evaluate(Rig& r,const Prepared& p,LayeredShellFields& shells) { return packets::EvaluatePackets(r,p,shells); }
bool Publish(Rig& r,const Prepared& p,const LayeredShellFields& shells) { return packets::PublishPackets(r,p,shells); }
void SameShells(const ShellFields& a,const ShellFields& b) {
    EXPECT_EQ(std::memcmp(a.quad.data(),b.quad.data(),a.quad.size()*sizeof(q::ForceTrial)),0);
    EXPECT_EQ(std::memcmp(a.triangle.data(),b.triangle.data(),a.triangle.size()*sizeof(t::ForceTrial)),0);
    EXPECT_EQ(std::memcmp(a.qsection.data(),b.qsection.data(),a.qsection.size()*sizeof(fe::ShellBatchSectionState)),0);
    EXPECT_EQ(std::memcmp(a.tsection.data(),b.tsection.data(),a.tsection.size()*sizeof(fe::ShellBatchSectionState)),0);
}
void SameFields(const Fields& a,const Fields& b) {
    EXPECT_EQ(a.x,b.x);EXPECT_EQ(a.v,b.v);EXPECT_EQ(a.w,b.w);EXPECT_EQ(a.orientation,b.orientation);
    EXPECT_EQ(a.reaction,b.reaction);EXPECT_EQ(a.couple,b.couple);
    EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
}
std::array<fe::NodalAllocationInfo,4> Allocations(const Rig& r) {
    return {r.owner.allocations(),r.qeph.allocations(),r.t3.allocations(),r.publication.allocations()};
}
void SameAllocations(const Rig& r,const std::array<fe::NodalAllocationInfo,4>& old) {
    const auto next=Allocations(r);for(unsigned i=0;i<4;++i) {
        EXPECT_EQ(old[i].device_bytes,next[i].device_bytes);EXPECT_EQ(old[i].device_allocations,next[i].device_allocations);
    }
}
}
