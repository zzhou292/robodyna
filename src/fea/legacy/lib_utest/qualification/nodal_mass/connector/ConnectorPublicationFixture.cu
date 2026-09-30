#include "ConnectorPublicationFixture.h"
#include <cstring>

namespace nodal_mass_test::joined_connector {
bool Rig::Start(bool moving) {
  const tl::math::Vec3 velocity=moving?tl::math::Vec3{8.,-2.,1.}:tl::math::Vec3{};
  // The fixture's small property J and nonzero rotational damping make its
  // native spring bound tighter than the shells. Use a qualified small step.
  if(!Joined::Initialize(true,true,velocity,0x1p-26))return false;
  spring::BatchConfig c;c.owner=owner.accepted();c.element_count=connectors.connection_count();
  c.configuration_id=7;c.qualification_id=8;
  if(moving)c.startup={fe::ShellBatchStartupKind::ReferenceUniformTranslation,velocity};
  auto report=connector.InitializeJoined(c,connectors,mass);
  EXPECT_EQ(report.status,CS::Success)<<report.message;if(report.status!=CS::Success)return false;
  fe::NodalTrialToken token;fe::NodalAssemblyView view;
  EXPECT_EQ(owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  EXPECT_EQ(qb.AssembleAccepted(owner,view).status,q::BatchStatus::Success);
  EXPECT_EQ(tb.AssembleAccepted(owner,view).status,t::BatchStatus::Success);
  report=connector.AssembleAccepted(owner,view);EXPECT_EQ(report.status,CS::Success)<<report.message;
  Discard();
  const auto joined=publication.Initialize(owner,qb,tb,connector);
  EXPECT_EQ(joined.status,PS::Success)<<joined.message;
  return !::testing::Test::HasFailure();
}
void Rig::Discard() { owner.Discard();qb.DiscardTrial();tb.DiscardTrial();connector.DiscardTrial();publication.DiscardTrial(); }
bool Rig::Read(Accepted& a) {
  if(!temporal::Read(owner,a.nodes))return false;
  q::BatchDiagnostics qd;t::BatchDiagnostics td;spring::BatchDiagnostics cd;
  EXPECT_EQ(qb.CopyAcceptedResults(owner.accepted(),&a.qeph,1,&qd).status,q::BatchStatus::Success);
  EXPECT_EQ(tb.CopyAcceptedResults(owner.accepted(),&a.t3,1,&td).status,t::BatchStatus::Success);
  EXPECT_EQ(connector.CopyAcceptedResults(owner.accepted(),a.connectors.data(),a.connectors.size(),&cd).status,CS::Success);
  EXPECT_EQ(publication.CopyAcceptedDiagnostics(owner.accepted(),&a.common).status,PS::Success);
  return !::testing::Test::HasFailure();
}
bool Rig::Begin(Candidate& p,const temporal::Loads& load,bool with_connector) {
  auto nr=owner.BeginTrial(&p.token,&p.assembly);EXPECT_EQ(nr.status,fe::NodalStatus::Ok);
  if(nr.status!=fe::NodalStatus::Ok)return false;
  const auto qr=qb.AssembleAccepted(owner,p.assembly);EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  const auto tr=tb.AssembleAccepted(owner,p.assembly);EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  if(with_connector) {
    const auto cr=connector.AssembleAccepted(owner,p.assembly);EXPECT_EQ(cr.status,CS::Success)<<cr.message;
    if(cr.status!=CS::Success)return false;
  }
  temporal::AddLoads<<<1,1,0,p.assembly.stream>>>(p.assembly,load,1.);
  EXPECT_EQ(cudaPeekAtLastError(),cudaSuccess);
  nr=owner.SealAssembly(p.token);EXPECT_EQ(nr.status,fe::NodalStatus::Ok)<<nr.message;
  if(nr.status!=fe::NodalStatus::Ok)return false;
  nr=fe::AdvanceStaggeredHistory(owner,p.token,
      {p.assembly.owner_id,p.assembly.accepted.base_epoch,p.assembly.attempt,initial.h,1.,8});
  EXPECT_EQ(nr.status,fe::NodalStatus::Ok)<<nr.message;if(nr.status!=fe::NodalStatus::Ok)return false;
  nr=owner.BorrowPrepared(p.token,&p.view);EXPECT_EQ(nr.status,fe::NodalStatus::Ok);
  return !::testing::Test::HasFailure();
}
bool Rig::Evaluate(Candidate& p,bool with_connector) {
  const auto qr=qb.EvaluateCandidate(owner,p.token,p.view,&p.qd);EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message;
  const auto tr=tb.EvaluateCandidate(owner,p.token,p.view,&p.td);EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  if(with_connector) {
    const auto cr=connector.EvaluateCandidate(owner,p.token,p.view,&p.cd);EXPECT_EQ(cr.status,CS::Success)<<cr.message;
    if(cr.status==CS::Success)EXPECT_LE(initial.h,.5*p.cd.minimum_native_dt);
  }
  if(::testing::Test::HasFailure())return false;
  const auto pr=publication.Prepare(owner,p.token,p.qd,p.td,p.cd,&p.common);
  EXPECT_EQ(pr.status,PS::Success)<<pr.message;return pr.status==PS::Success;
}
bool Rig::Commit(Candidate& p) {
  const auto r=publication.Commit(owner,p.token,p.common,
      {owner.accepted().owner_id,owner.accepted().epoch,p.view.attempt,8,true});
  EXPECT_EQ(r.status,PS::Success)<<r.message;return r.status==PS::Success;
}
void SameAccepted(const Accepted& a,const Accepted& b) {
  temporal::SameState(a.nodes,b.nodes);
  // Same accepted slabs read twice: their exact stored bytes must survive.
  EXPECT_EQ(Bytes(a.qeph),Bytes(b.qeph));EXPECT_EQ(Bytes(a.t3),Bytes(b.t3));
  for(std::size_t e=0;e<a.connectors.size();++e) {
    EXPECT_EQ(type25_test::EvaluationValues(a.connectors[e]),type25_test::EvaluationValues(b.connectors[e]));
    EXPECT_EQ(a.connectors[e].history.active,b.connectors[e].history.active);
  }
  EXPECT_EQ(Bytes(a.common),Bytes(b.common));
}
void CheckKinetic(const Rig& r,const temporal::Snapshot& s,const fe::ShellBatchKinetic& k) {
  long double expected[6]{};
  for(std::size_t n=0;n<r.mass.node_count();++n) {
    const auto& m=r.mass.nodes()[n].coefficients;long double vv=0,ww=0;
    for(unsigned a=0;a<3;++a) {
      const long double v=s.v[3*n+a],w=s.omega[3*n+a];vv+=v*v;ww+=w*w;
    }
    expected[0]+=.5L*m.mass*vv;expected[1]+=.5L*m.isotropic_inertia*ww;
    expected[2]+=.5L*m.shell.physical_inertia*ww;expected[3]+=.5L*m.shell.added_inertia*ww;
    expected[4]+=.5L*m.connector_mass*vv;expected[5]+=.5L*m.connector_inertia*ww;
  }
  const double actual[]{k.translation,k.rotation,k.physical_isotropic,k.added_isotropic,k.connector_translation,k.connector_rotation};
  for(unsigned i=0;i<6;++i)Near(actual[i],expected[i]);
}
void CheckCacheScatter(Rig& r,const Accepted& old,const Candidate& p,const temporal::Loads& load) {
  constexpr std::size_t count=5;std::array<double,6*count> actual{},expected{};
  const double* fields[]{p.assembly.forces.force_x,p.assembly.forces.force_y,p.assembly.forces.force_z,
      p.assembly.forces.couple_x,p.assembly.forces.couple_y,p.assembly.forces.couple_z};
  for(unsigned a=0;a<6;++a)ASSERT_EQ(cudaMemcpyAsync(actual.data()+a*count,fields[a],
      count*sizeof(double),cudaMemcpyDeviceToHost,p.view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
  auto component=[](tl::math::Vec3 v,unsigned a){return a==0?v.x:a==1?v.y:v.z;};
  auto shell=[&](const auto& nodes,const auto& value) {
    for(std::size_t n=0;n<nodes.size();++n)for(unsigned a=0;a<3;++a) {
      expected[a*count+nodes[n]]-=component(value.internal_force[n],a);
      expected[(a+3)*count+nodes[n]]-=component(value.internal_couple[n],a);
    }
  };
  shell(r.shells.qeph_nodes(),old.qeph);shell(r.shells.t3_nodes(),old.t3);
  for(std::size_t e=0;e<old.connectors.size();++e)for(unsigned n=0;n<2;++n)for(unsigned a=0;a<3;++a) {
    const auto node=r.connectors.connections()[e].global_node[n];
    expected[a*count+node]+=component(old.connectors[e].endpoints[n].force_N,a);
    expected[(a+3)*count+node]+=component(old.connectors[e].endpoints[n].couple_Nm,a);
  }
  for(std::size_t n=0;n<count;++n)for(unsigned a=0;a<3;++a) {
    expected[a*count+n]+=load.force[3*n+a];expected[(a+3)*count+n]+=load.couple[3*n+a];
  }
  EXPECT_EQ(actual,expected);
}
} // namespace nodal_mass_test::joined_connector
