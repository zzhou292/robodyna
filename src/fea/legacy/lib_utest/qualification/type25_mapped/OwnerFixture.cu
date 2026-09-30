// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"
namespace type25_mapped_test {
bool Rig::Initialize() {
  auto report=InitializeOwner(fixture,owner);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  if (report.status!=fe::NodalStatus::Ok) return false;
  config=fixture.Config();
  config.owner=owner.accepted();
  auto made=batch.InitializeMapped(config,fixture.physical,owner,fixture.Witnesses(),spring::CapacityProfile::Legacy);
  EXPECT_EQ(made.status,spring::BatchStatus::Success)<<made.message;
  if (made.status!=spring::BatchStatus::Success) return false;
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  if (!Begin(token,view)) return false;
  made=spring::BatchQualificationPeer::Claim(batch,owner,fixture.physical,config);
  EXPECT_EQ(made.status,spring::BatchStatus::Success)<<made.message;
  owner.Discard();
  return made.status==spring::BatchStatus::Success;
}
bool Rig::Begin(fe::NodalTrialToken& token,fe::NodalAssemblyView& view) {
  const auto started=owner.BeginTrial(&token,&view);
  EXPECT_EQ(started.status,fe::NodalStatus::Ok)<<started.message;
  if (started.status!=fe::NodalStatus::Ok) return false;
  const auto assembled=batch.AssembleMappedAccepted(owner,token,view);
  EXPECT_EQ(assembled.status,spring::BatchStatus::Success)<<assembled.message;
  return assembled.status==spring::BatchStatus::Success;
}
std::vector<spring::Evaluation> Rig::Accepted() {
  std::vector<spring::Evaluation> values(config.element_count);
  spring::BatchDiagnostics diagnostics;
  const auto report=batch.CopyAcceptedResults(owner.accepted(),values.data(),values.size(),&diagnostics);
  EXPECT_EQ(report.status,spring::BatchStatus::Success)<<report.message;
  return values;
}
void Rig::CheckAssembly(const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view) {
  fe::NodalCinAssemblyView cin;
  ASSERT_EQ(owner.BorrowCinAssembly(token,&cin).status,fe::NodalStatus::Ok);
  const auto n=config.owner.node_count;
  std::vector<double> expected_t(n),expected_r(n),actual_t(n),actual_r(n);
  std::vector<spring::Vec3> force(n),couple(n);
  const auto accepted=Accepted();
  for (std::size_t e=0;e<accepted.size();++e) {
    const auto& connection=fixture.model.connections()[e];
    const auto& property=fixture.model.properties()[connection.property_index].property;
    const auto native=NativeStiffness(fixture.model.source_units(),property,
        accepted[e].frame.length_m,accepted[e].history.active,2);
    for (unsigned slot=0;slot<2;++slot) {
      const auto node=connection.global_node[slot];
      expected_t[node]+=native[slot];
      expected_r[node]+=native[slot+2];
      force[node]=tl::math::fixed3::Add(force[node],accepted[e].endpoints[slot].force_N);
      couple[node]=tl::math::fixed3::Add(couple[node],accepted[e].endpoints[slot].couple_Nm);
    }
  }
  ASSERT_EQ(cudaMemcpyAsync(actual_t.data(),cin.translational_stiffness,n*sizeof(double),
      cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(actual_r.data(),cin.rotational_stiffness,n*sizeof(double),
      cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(cin.stream),cudaSuccess);
  for (std::size_t node=0;node<n;++node) {
    EXPECT_DOUBLE_EQ(actual_t[node],expected_t[node])<<node;
    EXPECT_DOUBLE_EQ(actual_r[node],expected_r[node])<<node;
  }
  const double* fields[6]{view.forces.force_x,view.forces.force_y,view.forces.force_z,
      view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
  std::vector<double> actual(n);
  for (unsigned field=0;field<6;++field) {
    ASSERT_EQ(cudaMemcpyAsync(actual.data(),fields[field],n*sizeof(double),
        cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(cin.stream),cudaSuccess);
    for (std::size_t node=0;node<n;++node) {
      const auto v=field<3?force[node]:couple[node];
      const double expected=field%3==0?v.x:field%3==1?v.y:v.z;
      EXPECT_EQ(Bits(actual[node]),Bits(expected))<<node<<":"<<field;
    }
  }
}
bool Rig::Prepare(fe::NodalTrialToken& token,fe::NodalAssemblyView& assembly,fe::NodalPreparedView& prepared) {
  if (!Begin(token,assembly)) return false;
  CheckAssembly(token,assembly);
  fe::NodalCinAssemblyView cin;
  if (owner.BorrowCinAssembly(token,&cin).status!=fe::NodalStatus::Ok) return false;
  // Explicit remaining-producer stiffness/activity in this mixed fixture. The
  // measured TYPE25 terms above are real; these prescribed test contributions
  // do not assert a joined shell/solid/rigid production participant inventory.
  std::vector<double> values(cin.node_count);
  if (cudaMemcpyAsync(values.data(),cin.translational_stiffness,values.size()*sizeof(double),
      cudaMemcpyDeviceToHost,cin.stream)!=cudaSuccess || cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  for (std::size_t row=0;row<fixture.base.mechanics.cin_model.rows().count;++row) {
    const auto& item=fixture.base.mechanics.cin_model.rows().data[row];
    for (auto node:item.master_domain_nodes) values[node]+=1;
    values[item.secondary_domain_node]+=1;
  }
  if (cudaMemcpyAsync(cin.translational_stiffness,values.data(),values.size()*sizeof(double),
      cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess) return false;
  std::vector<std::uint8_t> activity(cin.witness_count,1);
  if (cudaMemcpyAsync(cin.witness_activity,activity.data(),activity.size(),
      cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess) return false;
  const auto node=fixture.base.mechanics.domain.Find(12);
  double load;
  if (cudaMemcpyAsync(&load,assembly.forces.force_x+node,sizeof(load),cudaMemcpyDeviceToHost,cin.stream)!=cudaSuccess ||
      cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  load+=1000;
  if (cudaMemcpyAsync(assembly.forces.force_x+node,&load,sizeof(load),cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess ||
      cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  auto report=owner.SealAssembly(token);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  if (report.status!=fe::NodalStatus::Ok) return false;
  // CIN qualification identity belongs to this common owner operation, so the
  // participant config uses that exact ID for subsequent common validation.
  report=fe::AdvanceStaggeredCin(owner,token,{assembly.owner_id,assembly.accepted.base_epoch,
      assembly.attempt,cin.qualification_id,config.owner.fixed_dt,1.,true});
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  if (report.status!=fe::NodalStatus::Ok) return false;
  report=owner.BorrowPrepared(token,&prepared);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  return report.status==fe::NodalStatus::Ok;
}
std::vector<spring::EndpointKinematics> Rig::Nodes(const fe::NodalPreparedView& view) {
  const auto n=config.owner.node_count;
  std::vector<double> packed(9*n);
  EXPECT_EQ(cudaMemcpyAsync(packed.data(),view.kinematics.position_xyz,3*n*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(packed.data()+3*n,view.kinematics.velocity_xyz,3*n*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(packed.data()+6*n,view.kinematics.angular_velocity_xyz,3*n*sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  std::vector<spring::EndpointKinematics> result(n);
  for (std::size_t node=0;node<n;++node) result[node]={
      {packed[3*node],packed[3*node+1],packed[3*node+2]},
      {packed[3*n+3*node],packed[3*n+3*node+1],packed[3*n+3*node+2]},
      {packed[6*n+3*node],packed[6*n+3*node+1],packed[6*n+3*node+2]}};
  return result;
}
std::vector<double> Rig::Snapshot() {
  const auto n=config.owner.node_count;
  const auto count=fixture.base.mechanics.cin_model.rows().count;
  std::vector<double> result(15*n+2*count+1);
  fe::NodalStamp stamp;
  EXPECT_EQ(owner.CopyAccepted({result.data(),result.data()+3*n,n,result.data()+6*n,
      result.data()+10*n},&stamp).status,fe::NodalStatus::Ok);
  EXPECT_EQ(owner.CopyAcceptedCin({result.data()+13*n,result.data()+14*n,
      result.data()+15*n,result.data()+15*n+count,result.data()+15*n+2*count,n,count},
      &stamp).status,fe::NodalStatus::Ok);
  return result;
}
}
