// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
namespace extended_resident_test {
bool Rig::Initialize(bool attach) {
  auto& f=fixture.Mechanics();auto c=f.Config();c.fixed_dt=1e-8;
  const auto cin=f.Cin();
  if(!Good(owner.Initialize(c,f.Kinematics(),f.im.data(),f.Dofs(),fixture.binding,&cin)))return false;
  config=fixture.Configuration();config.owner=owner.accepted();
  if(!Good(batch.InitializeJoined(config,fixture.model))||
      !native.Initialize(fixture.model,fixture.foam_input,{}))return false;
  return !attach||Attach();
}
bool Rig::Attach() {
  if(!Good(Peer::PreflightAttach(batch,owner,fixture.ledger,fixture.binding,
      fixture.Witnesses(),fixture.model,config)))return false;
  Peer::Attach(batch);return true;
}
bool Rig::Read(Results& r,s::BatchDiagnostics& d) {return Good(batch.CopyAcceptedResults(owner.accepted(),r.Buffers(),&d));}
bool Rig::Begin(fe::NodalTrialToken& t,fe::NodalAssemblyView& a) {
  return Good(owner.BeginTrial(&t,&a))&&Good(batch.AssembleAccepted(owner,t,a));
}
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
bool Rig::Compare(const fe::NodalPreparedView& view,const Results& result) {
  std::vector<double> x(3*config.owner.node_count),v(x.size());
  if(cudaMemcpyAsync(x.data(),view.kinematics.position_xyz,x.size()*sizeof(double),cudaMemcpyDeviceToHost,view.stream)!=cudaSuccess||
      cudaMemcpyAsync(v.data(),view.kinematics.velocity_xyz,v.size()*sizeof(double),cudaMemcpyDeviceToHost,view.stream)!=cudaSuccess||
      cudaStreamSynchronize(view.stream)!=cudaSuccess)return false;
  return native.Evaluate(x,v,view.base_time,config.owner.fixed_dt,view.kinematics.base_epoch)&&native.Compare(result,true);
}
void Rig::CheckAssembly(const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view,const Results& result,double stiffness_seed) {
  const auto n=config.owner.node_count;
  std::vector<double> expected[4],actual[4];
  for(unsigned k=0;k<4;++k){expected[k].resize(n);actual[k].resize(n);}
  std::fill(expected[3].begin(),expected[3].end(),stiffness_seed);
  auto add=[&](auto parents,const auto& values,unsigned slots) {
    for(std::size_t p=0;p<parents.size();++p)for(unsigned k=0;k<slots;++k) {
      const auto node=parents[p].domain_nodes[k];const auto force=values[p].cache.rhs_force_n[k];
      expected[0][node]+=force.x;expected[1][node]+=force.y;expected[2][node]+=force.z;
      expected[3][node]+=values[p].cache.stiffness.translation_n_m;
    }
  };
  const auto& m=fixture.model;
  add(m.solid18(),result.old18,8);add(m.solid24(),result.old24,8);add(m.solid6z(),result.old6z,6);
  add(m.solid18_law44(),result.rear,8);add(m.solid18_law90(),result.foam,8);
  fe::NodalCinAssemblyView cin;ASSERT_TRUE(Good(owner.BorrowCinAssembly(token,&cin)));
  const double* fields[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,cin.translational_stiffness};
  for(unsigned k=0;k<4;++k)ASSERT_EQ(cudaMemcpyAsync(actual[k].data(),fields[k],n*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  for(unsigned k=0;k<4;++k)for(std::size_t i=0;i<n;++i)EXPECT_TRUE(fe::shell_startup_detail::SameBits(actual[k][i],expected[k][i]))<<k<<':'<<i;
}
} // namespace extended_resident_test
