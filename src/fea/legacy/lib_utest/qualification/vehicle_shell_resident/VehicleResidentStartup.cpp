#include "VehicleResidentFixture.h"
namespace vehicle_resident_test {
Rig::Rig(std::size_t qc,std::size_t tc,std::size_t nodes):nq(qc),nt(tc),n(nodes),source(qc,tc,nodes,2),
    initial(nodes),inverse(nodes),inverse_j(nodes),fixed(nodes) {
  // Isolate the last Q4 square without omitting any node or parent. The final
  // T3 remains the only user of the last global node/high original-ID slot.
  const auto squares=(n-1)/4;
  auto relocate=[&](std::size_t e,std::size_t first){auto& p=source.geometry.q[e];
    p.nodes={first,first+1,first+2,first+3};source.geometry.Set(p.reference,p.nodes,1024.,1./32);};
  for(std::size_t e=squares-1;e<nq;e+=squares)relocate(e,0);
  relocate(nq-1,4*(squares-1));
}
q::QephBatchConfig Rig::QConfig() const {
  q::QephBatchConfig c;c.owner=owner.accepted();c.element_count=nq;c.configuration_id=Configuration;c.qualification_id=Qualification;
  c.usage=q::BatchUsage::CoupledForces;c.storage_limits=fe::ShellResidentLimits::Vehicle();
  c.max_device_bytes=fe::MaxVehicleShellResidentDeviceBytes;
  c.startup.kind=fe::ShellBatchStartupKind::ReferenceUniformTranslation;c.startup.uniform_velocity={2,0,0};return c;
}
t::T3BatchConfig Rig::TConfig() const {
  t::T3BatchConfig c;c.owner=owner.accepted();c.element_count=nt;c.configuration_id=Configuration;c.qualification_id=Qualification;
  c.usage=t::BatchUsage::CoupledForces;c.storage_limits=fe::ShellResidentLimits::Vehicle();
  c.max_device_bytes=fe::MaxVehicleShellResidentDeviceBytes;
  c.startup.kind=fe::ShellBatchStartupKind::ReferenceUniformTranslation;c.startup.uniform_velocity={2,0,0};return c;
}
bool Rig::Initialize(bool with_plastic) {
  plastic=with_plastic;
  const auto b=binding.Initialize(source.geometry.input(),fe::ShellHostBindingLimits::Vehicle());
  EXPECT_EQ(b.status,fe::ShellBindingStatus::Success)<<b.message;if(b.status!=fe::ShellBindingStatus::Success)return false;
  const auto p=catalog.InitializeCatalog(binding,source.input(),fe::ShellPlasticityCatalogLimits::Vehicle());
  EXPECT_EQ(p.status,fe::ShellPlasticityBindingStatus::Success)<<p.message;if(p.status!=fe::ShellPlasticityBindingStatus::Success)return false;
  for(std::size_t i=0;i<n;++i){const auto& node=binding.nodes()[i];
    initial.x[3*i]=node.position.x;initial.x[3*i+1]=node.position.y;initial.x[3*i+2]=node.position.z;
    initial.v[3*i]=2;initial.orientation[4*i]=1;inverse[i]=1/node.native.mass;inverse_j[i]=1/node.native.isotropic_inertia;}
  fe::NodalStateConfig nc;nc.node_count=n;nc.fixed_dt=H;nc.max_nodes=fe::MaxActiveNodalStateNodes;
  nc.max_device_bytes=fe::MaxActiveNodalStateDeviceBytes;nc.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto nodal=owner.Initialize(nc,{initial.x.data(),initial.v.data(),initial.w.data(),n,initial.orientation.data()},
    inverse.data(),{fixed.data(),fixed.data(),inverse_j.data()});
  EXPECT_EQ(nodal.status,fe::NodalStatus::Ok)<<nodal.message;if(nodal.status!=fe::NodalStatus::Ok)return false;
  const auto qr=plastic?qeph.InitializeJoined(QConfig(),binding,catalog):qeph.InitializeJoined(QConfig(),binding);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;if(qr.status!=q::BatchStatus::Success)return false;
  const auto tr=plastic?t3.InitializeJoined(TConfig(),binding,catalog):t3.InitializeJoined(TConfig(),binding);
  EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;if(tr.status!=t::BatchStatus::Success)return false;
  fe::NodalTrialToken token;fe::NodalAssemblyView view;
  EXPECT_EQ(owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  EXPECT_EQ(qeph.AssembleAccepted(owner,view).status,q::BatchStatus::Success);
  EXPECT_EQ(t3.AssembleAccepted(owner,view).status,t::BatchStatus::Success);Discard();
  if(::testing::Test::HasFailure())return false;
  const auto joined=publication.Initialize(owner,qeph,t3,fe::ShellPublicationLimits::Vehicle());
  EXPECT_EQ(joined.status,fe::ShellPublicationStatus::Success)<<joined.message;return joined.status==fe::ShellPublicationStatus::Success;
}
bool Read(Rig& r,Snapshot& out) {
  const auto status=r.owner.CopyAccepted(out.buffer(),&out.stamp);EXPECT_EQ(status.status,fe::NodalStatus::Ok);return status.status==fe::NodalStatus::Ok;
}
bool Accepted(Rig& r,Results& out) {
  const auto stamp=r.owner.accepted();
  EXPECT_EQ(r.qeph.CopyAcceptedResults(stamp,out.qr.data(),r.nq,&out.diagnostics.qeph).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.CopyAcceptedResults(stamp,out.tr.data(),r.nt,&out.diagnostics.t3).status,t::BatchStatus::Success);
  if(r.plastic){
    EXPECT_EQ(r.qeph.CopyAcceptedSectionHistory(stamp,out.qs.data(),r.nq,&out.diagnostics.qeph).status,q::BatchStatus::Success);
    EXPECT_EQ(r.t3.CopyAcceptedSectionHistory(stamp,out.ts.data(),r.nt,&out.diagnostics.t3).status,t::BatchStatus::Success);}
  EXPECT_EQ(r.publication.CopyAcceptedDiagnostics(stamp,&out.diagnostics).status,fe::ShellPublicationStatus::Success);
  return !::testing::Test::HasFailure();
}
} // namespace vehicle_resident_test
