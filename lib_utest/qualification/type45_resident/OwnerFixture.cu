// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace type45_resident_test {
joint::BatchConfig Rig::Config() const {
  joint::BatchConfig c;c.owner=physical.owner.accepted();
  c.configuration_id=common::Configuration;c.qualification_id=common::Qualification;
  c.profile=joint::BatchProfile::PhysicalAggregateV1;
  c.cin_attachment_count=physical.fixture.ranges.size();c.cin_witness_count=physical.fixture.witnesses.size();
  return c;
}
fe::ShellPhysicalParticipants Rig::Participants() {auto p=physical.Participants();p.type45=&joints;return p;}
bool Rig::Initialize(joint::WorkingUnits units,bool attach,bool initialize_batch) {
  if(!physical.Initialize(true,false)) return false;
  std::array<joint::JointInput,3> rows;
  const auto& f=physical.fixture;
  for(unsigned k=0;k<3;++k) {
    auto& row=rows[k];row.property.kind=static_cast<joint::Kind>(k+1);row.property.working_units=units;
    row.property.automatic_stiffness_scale=.01;row.property.critical_damping_ratio=.05;
    row.geometry.source_joint_id=2200512+k;
    row.geometry.source_node_id[0]=778;row.geometry.source_node_id[1]=k==2?9303:9302;
    if(k) row.geometry.source_node_id[2]=9307;
    for(unsigned n=0;n<(k?3u:2u);++n)
      row.geometry.position_m[n]=f.domain.nodes()[f.domain.Find(row.geometry.source_node_id[n])].position;
    row.body[0]={fe::RigidBindingSourceKind::NodalGroup,201};
    row.body[1]={fe::RigidBindingSourceKind::Part,200};
  }
  const auto built=model.Initialize(f.rigid,{f.domain.source_instance_id(),{rows.data(),rows.size()}});
  EXPECT_TRUE(built)<<built.message;if(!built) return false;
  if(initialize_batch && !Good(joints.InitializeJoined(Config(),model))) return false;
  return !attach || Attach();
}
bool Rig::Attach() {
  return Good(physical.publication.InitializePhysicalWithJoints(physical.owner,physical.fixture.physical,
      physical.fixture.rigid,physical.fixture.WitnessSource(),model,Participants(),physical.fixture.Identity()));
}
bool Rig::CopyAccepted(std::array<joint::Result,3>& values,joint::BatchDiagnostics& d) {
  return Good(joints.CopyAcceptedResults(physical.owner.accepted(),{values.data(),values.size()},&d));
}
bool Rig::Stage(fe::NodalTrialToken& token,fe::NodalPreparedView& view,fe::ShellPhysicalDiagnostics& d,bool eval) {
  fe::NodalAssemblyView assembly;
  if(!physical.Begin(token,assembly) || !JointScatter(*this,token,assembly)) return false;
  // A real external fixture load on a PART member exercises joint forces and
  // couples. The existing common fixture also retains its independent solid load.
  const auto node=physical.fixture.domain.Find(9302);
  double force=0;
  if(cudaMemcpyAsync(&force,assembly.forces.force_y+node,sizeof(force),cudaMemcpyDeviceToHost,assembly.stream)!=cudaSuccess ||
      cudaStreamSynchronize(assembly.stream)!=cudaSuccess) return false;
  force+=1;
  if(cudaMemcpyAsync(assembly.forces.force_y+node,&force,sizeof(force),cudaMemcpyHostToDevice,assembly.stream)!=cudaSuccess)
    return false;
  if(!physical.Advance(token,assembly,view) || !physical.Evaluate(token,view,d)) return false;
  return !eval || Good(joints.EvaluateCandidate(physical.owner,token,view,&d.type45));
}
bool Rig::Prepare(fe::NodalTrialToken& token,fe::NodalPreparedView& view,fe::ShellPhysicalDiagnostics& output) {
  fe::ShellPhysicalDiagnostics candidates;
  return Stage(token,view,candidates) &&
      Good(physical.publication.PreparePhysical(physical.owner,token,Candidates(candidates),&output));
}
} // namespace type45_resident_test
