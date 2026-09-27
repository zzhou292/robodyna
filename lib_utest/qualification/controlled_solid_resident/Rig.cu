// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
namespace controlled_resident_test {
bool Rig::Initialize() {
  auto& f=fixture.Mechanics();auto c=f.Config();c.fixed_dt=1e-8;const auto cin=f.Cin();
  if(!Good(owner.Initialize(c,f.Kinematics(),f.im.data(),f.Dofs(),fixture.binding,&cin)))return false;
  config=fixture.Configuration();config.owner=owner.accepted();
  if(!Good(batch.InitializeJoined(config,fixture.model)))return false;
  if(!Good(Peer::PreflightAttach(batch,owner,fixture.ledger,fixture.binding,fixture.Witnesses(),fixture.model,config)))return false;
  Peer::Attach(batch);
  const auto& model=fixture.model;const auto units=model.control_selection()->units();
  const auto& hp=model.solid24()[0];const auto& fp=model.solid18_law90()[0];foam::Material fm;
  if(h24::PrepareReference(hp.reference,model.materials42()[hp.material_index].value,units,hreference)!=fe::solid24::ForceStatus::Success||
     foam::PrepareMaterial(model.materials90()[fp.material_index].value,units,fm)!=fe::solid_common::distortion::Status::Success||
     foam::PrepareReference(fp.reference,fm,freference)!=fe::solid_common::distortion::Status::Success)return false;
  h24::Scratch hs;foam::Scratch fs;
  return h24::PrepareInitial(hreference,config.startup.uniform_velocity,hs)==fe::solid24::ForceStatus::Success&&
    h24::Complete(hs,hs.activity.triggers_native_batch,hexpected)==fe::solid24::ForceStatus::Success&&
    foam::PrepareInitial(freference,config.startup.uniform_velocity,fs)==fe::solid_common::distortion::Status::Success&&
    foam::Complete(fs,fs.activity.triggers_native_batch,fexpected)==fe::solid_common::distortion::Status::Success;
}
bool Rig::Read(Results& r,s::BatchDiagnostics& d){return Good(batch.CopyAcceptedResultsWithControls(owner.accepted(),r.Buffers(),&d));}
bool Rig::Begin(fe::NodalTrialToken& t,fe::NodalAssemblyView& a){return Good(owner.BeginTrial(&t,&a))&&Good(batch.AssembleAccepted(owner,t,a));}
bool Rig::Prepare(const fe::NodalTrialToken& token,const fe::NodalAssemblyView& a,fe::NodalPreparedView& out) {
  fe::NodalCinAssemblyView cin;
  if(!Good(owner.BorrowCinAssembly(token,&cin)))return false;
  std::vector<double> stiffness(cin.node_count);
  if(cudaMemcpyAsync(stiffness.data(),cin.translational_stiffness,stiffness.size()*sizeof(double),
      cudaMemcpyDeviceToHost,cin.stream)!=cudaSuccess||cudaStreamSynchronize(cin.stream)!=cudaSuccess)return false;
  // Explicit other-contributor coefficients for the disjoint tiny CIN patch.
  const auto rows=fixture.Mechanics().cin_model.rows();
  for(std::size_t k=0;k<rows.count;++k) {
    for(auto node:rows.data[k].master_domain_nodes)stiffness[node]+=1;
    stiffness[rows.data[k].secondary_domain_node]+=1;
  }
  if(cudaMemcpyAsync(cin.translational_stiffness,stiffness.data(),stiffness.size()*sizeof(double),
      cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess||
      cudaMemsetAsync(cin.witness_activity,1,cin.witness_count,cin.stream)!=cudaSuccess)return false;
  const auto node=fixture.Mechanics().ordinary;double force=0;
  if(cudaMemcpyAsync(&force,a.forces.force_x+node,sizeof(force),cudaMemcpyDeviceToHost,cin.stream)!=cudaSuccess||
      cudaStreamSynchronize(cin.stream)!=cudaSuccess)return false;
  force+=10;
  if(cudaMemcpyAsync(a.forces.force_x+node,&force,sizeof(force),cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess||
      cudaStreamSynchronize(cin.stream)!=cudaSuccess)return false;
  if(!Good(owner.SealAssembly(token))||!Good(fe::AdvanceStaggeredCin(owner,token,
      {a.owner_id,a.accepted.base_epoch,a.attempt,cin.qualification_id,config.owner.fixed_dt,.2,true})))return false;
  return Good(owner.BorrowPrepared(token,&out));
}

bool Rig::ReferenceStep(const fe::NodalPreparedView& view) {
  std::vector<double> x(3*config.owner.node_count),v(x.size());
  if(cudaMemcpyAsync(x.data(),view.kinematics.position_xyz,x.size()*sizeof(double),cudaMemcpyDeviceToHost,view.stream)!=cudaSuccess||
     cudaMemcpyAsync(v.data(),view.kinematics.velocity_xyz,v.size()*sizeof(double),cudaMemcpyDeviceToHost,view.stream)!=cudaSuccess||
     cudaStreamSynchronize(view.stream)!=cudaSuccess)return false;
  auto hi=d::Traits24::Phase(view.base_time,config.owner.fixed_dt,view.kinematics.base_epoch);
  auto fi=d::Traits18Law90::Phase(view.base_time,config.owner.fixed_dt,view.kinematics.base_epoch);
  auto fill=[&](auto& interval,const auto& parent,auto traits){for(unsigned n=0;n<8;++n){const auto j=parent.domain_nodes[n];
    traits.Node(interval,n,{x[3*j],x[3*j+1],x[3*j+2]},{v[3*j],v[3*j+1],v[3*j+2]});}};
  fill(hi,fixture.model.solid24()[0],d::Traits24{});fill(fi,fixture.model.solid18_law90()[0],d::Traits18Law90{});
  h24::Scratch hs;foam::Scratch fs;
  return h24::PrepareCandidate(hreference,hexpected.proposed_history,hi,hs)==fe::solid24::ForceStatus::Success&&
    h24::Complete(hs,hs.activity.triggers_native_batch,hexpected)==fe::solid24::ForceStatus::Success&&
    foam::PrepareCandidate(freference,fexpected.proposed_history,fi,fs)==fe::solid_common::distortion::Status::Success&&
    foam::Complete(fs,fs.activity.triggers_native_batch,fexpected)==fe::solid_common::distortion::Status::Success;
}
void Near(double a,double b){EXPECT_NEAR(a,b,2e-10*std::max(1.,std::abs(b)));}
void Rig::Compare(const Results& r) {
  const auto& h=r.h24[0];const auto& f=r.foam[0];
  ASSERT_NE(h.history.native(),nullptr);ASSERT_NE(f.history.native(),nullptr);
  EXPECT_EQ(h.history.legacy(),nullptr);EXPECT_EQ(f.history.legacy(),nullptr);
  EXPECT_EQ(h.history_units.length_m,hreference.units().length_m);EXPECT_EQ(f.history_units.length_m,freference.material().units().length_m);
  EXPECT_EQ(h.stamp.sample_index,hexpected.proposed_history.stamp().sample_index);
  EXPECT_EQ(f.stamp.sample_index,fexpected.proposed_history.native_history().stamp().sample_index);
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k){
    Near(fe::solid_common::Component(h.cache.rhs_force_n[n],k),fe::solid_common::Component(hexpected.rhs_force_n[n],k));
    Near(fe::solid_common::Component(f.cache.rhs_force_n[n],k),fe::solid_common::Component(fexpected.rhs_force_n[n],k));}
  Near(h.cache.stiffness.raw_stiffness_n_m,hexpected.nodal_raw_stiffness_n_m);
  Near(f.cache.stiffness.raw_stiffness_n_m,fexpected.nodal_raw_stiffness_n_m);
  Near(h.cache.response.distortion_energy_j,hexpected.distortion_energy_j);
  Near(f.cache.response.distortion_energy_j,fexpected.distortion_energy_j);
  Near(h.cache.response.hourglass_work_j,hexpected.hourglass_work_increment_j);
  Near(h.cache.response.distortion_work_j,hexpected.distortion_work_increment_j);
  Near(f.cache.response.distortion_work_j,fexpected.distortion_work_increment_j);
  const auto& hv=hexpected.proposed_history.native_values();
  for(unsigned k=0;k<6;++k)Near(h.history.native()->values.material.stress_pa[k],hv.material.stress_pa[k]);
  for(unsigned a=0;a<3;++a)for(unsigned b=0;b<4;++b)Near(h.history.native()->values.controlled_hourglass.force_n[a][b],hv.controlled_hourglass.force_n[a][b]);
  const auto& fv=fexpected.proposed_history.native_history().data();
  for(unsigned ip=0;ip<8;++ip){const auto& a=f.history.native()->values.point[ip];const auto& b=fv.point[ip];
    for(unsigned k=0;k<6;++k)Near(a.stress_pa[k],b.stress_pa[k]);
    Near(a.point.effective_modulus_pa,b.point.effective_modulus_pa);
    for(unsigned k=0;k<3;++k)EXPECT_EQ(a.point.cursor[k],b.point.cursor[k]);}
}
void Rig::CheckAssembly(const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view,const Results& result) {
  const auto n=config.owner.node_count;
  std::vector<double> expected[4],actual[4];
  for(unsigned k=0;k<4;++k){expected[k].resize(n);actual[k].resize(n);}
  
  auto add=[&](auto parents,const auto& values,unsigned slots) {
    for(std::size_t p=0;p<parents.size();++p)for(unsigned k=0;k<slots;++k) {
      const auto node=parents[p].domain_nodes[k];const auto force=values[p].cache.rhs_force_n[k];
      expected[0][node]+=force.x;expected[1][node]+=force.y;expected[2][node]+=force.z;
      expected[3][node]+=values[p].cache.stiffness.translation_n_m;
    }
  };
  const auto& m=fixture.model;
  add(m.solid18(),result.old18,8);add(m.solid24(),result.h24,8);add(m.solid6z(),result.old6z,6);
  add(m.solid18_law44(),result.rear,8);add(m.solid18_law90(),result.foam,8);
  fe::NodalCinAssemblyView cin;ASSERT_TRUE(Good(owner.BorrowCinAssembly(token,&cin)));
  const double* fields[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,cin.translational_stiffness};
  for(unsigned k=0;k<4;++k)ASSERT_EQ(cudaMemcpyAsync(actual[k].data(),fields[k],n*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  for(unsigned k=0;k<4;++k)for(std::size_t i=0;i<n;++i)EXPECT_TRUE(fe::shell_startup_detail::SameBits(actual[k][i],expected[k][i]))<<k<<':'<<i;
}
} // namespace controlled_resident_test
