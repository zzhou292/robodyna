// SPDX-License-Identifier: MIT
#include "ResidentFixture.h"
#include "ResultValues.h"

namespace qbat_resident_test {
namespace {
__global__ void AddLoads(fe::NodalAssemblyView view,const double* loads) {
  for(std::size_t node=blockIdx.x*blockDim.x+threadIdx.x;node<view.accepted.node_count;node+=gridDim.x*blockDim.x) {
    view.forces.force_x[node]+=loads[6*node];
    view.forces.force_y[node]+=loads[6*node+1];
    view.forces.force_z[node]+=loads[6*node+2];
    view.forces.couple_x[node]+=loads[6*node+3];
    view.forces.couple_y[node]+=loads[6*node+4];
    view.forces.couple_z[node]+=loads[6*node+5];
  }
}
}
bool Endpoint(const fe::NodalPreparedView& view,Fields& output) {
  Fields next(view.kinematics.node_count);
  const double* sources[]{view.kinematics.position_xyz,view.kinematics.velocity_xyz,
      view.kinematics.angular_velocity_xyz,view.kinematics.orientation_wxyz};
  double* destinations[]{next.x.data(),next.v.data(),next.omega.data(),next.q.data()};
  for(unsigned field=0;field<4;++field) {
    const auto copied=cudaMemcpyAsync(destinations[field],sources[field],
        (field==3?4:3)*view.kinematics.node_count*sizeof(double),cudaMemcpyDeviceToHost,view.stream);
    EXPECT_EQ(copied,cudaSuccess);
    if(copied!=cudaSuccess) return false;
  }
  const auto completed=cudaStreamSynchronize(view.stream);
  EXPECT_EQ(completed,cudaSuccess);
  if(completed!=cudaSuccess) return false;
  output=std::move(next);
  return true;
}
bool Rig::Prepare(Prepared& output,double rate,bool evaluate) {
  const auto count=binding->node_count();
  Fields before(count);
  const auto read=owner.CopyAccepted({before.x.data(),before.v.data(),count,before.q.data(),before.omega.data()},&before.stamp);
  EXPECT_EQ(read.status,fe::NodalStatus::Ok);
  if(read.status!=fe::NodalStatus::Ok) return false;
  std::vector<double> host_loads(6*count);
  if(!coupled||rate!=0) {
    const auto origin=binding->nodes()[0].position;
    const double kick=before.stamp.epoch?dt:.5*dt;
    for(std::size_t node=0;node<count;++node) {
      const auto& reference=binding->nodes()[node];
      const double x=reference.position.x-origin.x,y=reference.position.y-origin.y;
      const double velocity[]{rate*(x+.17*y),-.24*rate*y,0};
      const double omega[]{0,0,.2*rate};
      for(unsigned axis=0;axis<3;++axis) {
        host_loads[6*node+axis]=(mass?mass->nodes()[node].coefficients.mass:reference.native.mass)*(velocity[axis]-before.v[3*node+axis])/kick;
        host_loads[6*node+3+axis]=(mass?mass->nodes()[node].coefficients.isotropic_inertia:reference.native.isotropic_inertia)*(omega[axis]-before.omega[3*node+axis])/kick;
      }
    }
  }
  Prepared next;
  fe::NodalAssemblyView assembly;
  auto status=owner.BeginTrial(&next.token,&assembly);
  EXPECT_EQ(status.status,fe::NodalStatus::Ok);
  if(status.status!=fe::NodalStatus::Ok) return false;
  const auto copied=cudaMemcpyAsync(loads,host_loads.data(),host_loads.size()*sizeof(double),cudaMemcpyHostToDevice,assembly.stream);
  EXPECT_EQ(copied,cudaSuccess);
  if(copied!=cudaSuccess) return false;
  AddLoads<<<1u+static_cast<unsigned>((count-1)/64),64,0,assembly.stream>>>(assembly,loads);
  if(coupled&&!Assemble(assembly)) return false;
  status=owner.SealAssembly(next.token);
  EXPECT_EQ(status.status,fe::NodalStatus::Ok)<<status.message;
  if(status.status!=fe::NodalStatus::Ok) return false;
  status=fe::AdvanceStaggeredHistory(owner,next.token,
      {assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,dt,1.,Qualification});
  EXPECT_EQ(status.status,fe::NodalStatus::Ok)<<status.message;
  if(status.status!=fe::NodalStatus::Ok) return false;
  status=owner.BorrowPrepared(next.token,&next.view);
  EXPECT_EQ(status.status,fe::NodalStatus::Ok);
  if(status.status!=fe::NodalStatus::Ok||!Endpoint(next.view,next.endpoint)) return false;
  if(evaluate&&!Evaluate(next)) return false;
  output=std::move(next);
  return true;
}
bool Rig::Evaluate(Prepared& prepared) {
  auto& diagnostics=prepared.diagnostics;
  if(binding->qeph_count()) {
    const auto result=qeph.EvaluateCandidate(owner,prepared.token,prepared.view,&diagnostics.qeph);
    EXPECT_EQ(result.status,fe::qeph::BatchStatus::Success)<<result.message;
    if(result.status!=fe::qeph::BatchStatus::Success) return false;
  }
  if(binding->t3_count()) {
    const auto result=t3.EvaluateCandidate(owner,prepared.token,prepared.view,&diagnostics.t3);
    EXPECT_EQ(result.status,fe::t3::BatchStatus::Success)<<result.message;
    if(result.status!=fe::t3::BatchStatus::Success) return false;
  }
  const auto result=qbat.EvaluateCandidate(owner,prepared.token,prepared.view,&diagnostics.qbat);
  EXPECT_EQ(result.status,qb::BatchStatus::Success)<<result.message;
  if(result.status!=qb::BatchStatus::Success) return false;
  if(mass) {
    const auto evaluated=connector.EvaluateCandidate(owner,prepared.token,prepared.view,&diagnostics.connector);
    EXPECT_EQ(evaluated.status,fe::type25::BatchStatus::Success)<<evaluated.message;
    if(evaluated.status!=fe::type25::BatchStatus::Success) return false;
  }
  fe::ShellBatchDiagnostics common;
  const fe::ShellFormulationCandidates candidates{binding->qeph_count()?&diagnostics.qeph:nullptr,
      binding->t3_count()?&diagnostics.t3:nullptr,&diagnostics.qbat,mass?&diagnostics.connector:nullptr};
  const auto checked=publication.PrepareFormulations(owner,prepared.token,candidates,&common);
  EXPECT_EQ(checked.status,fe::ShellPublicationStatus::Success)<<checked.message;
  if(checked.status!=fe::ShellPublicationStatus::Success) return false;
  diagnostics=common;
  prepared.qbat.resize(binding->qbat_count());
  const auto copied=qbat.CopyPreparedResults(diagnostics.qbat,prepared.qbat.data(),prepared.qbat.size());
  EXPECT_EQ(copied.status,qb::BatchStatus::Success)<<copied.message;
  return copied.status==qb::BatchStatus::Success;
}
bool Rig::Commit(const Prepared& prepared) {
  const auto& d=prepared.diagnostics.qbat;
  const auto result=publication.Commit(owner,prepared.token,prepared.diagnostics,
      {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true});
  EXPECT_EQ(result.status,fe::ShellPublicationStatus::Success)<<result.message;
  return result.status==fe::ShellPublicationStatus::Success;
}
bool Rig::Accepted(Fields& fields,std::vector<qb::BatchResult>& results,fe::ShellBatchDiagnostics& diagnostics) {
  Fields next(binding->node_count());
  const auto copied=owner.CopyAccepted({next.x.data(),next.v.data(),binding->node_count(),next.q.data(),next.omega.data()},&next.stamp);
  EXPECT_EQ(copied.status,fe::NodalStatus::Ok);
  if(copied.status!=fe::NodalStatus::Ok) return false;
  std::vector<qb::BatchResult> rows(binding->qbat_count());
  qb::BatchDiagnostics b;
  const auto read=qbat.CopyAcceptedResults(next.stamp,rows.data(),rows.size(),&b);
  EXPECT_EQ(read.status,qb::BatchStatus::Success)<<read.message;
  if(read.status!=qb::BatchStatus::Success) return false;
  const auto common=publication.CopyAcceptedDiagnostics(next.stamp,&diagnostics);
  EXPECT_EQ(common.status,fe::ShellPublicationStatus::Success)<<common.message;
  if(common.status!=fe::ShellPublicationStatus::Success) return false;
  EXPECT_TRUE(qb::batch_detail::SameDiagnostics(b,diagnostics.qbat));
  fields=std::move(next);
  results=std::move(rows);
  return true;
}
qb::PrescribedInterval Interval(const Rig& rig,const Prepared& prepared,std::size_t parent) {
  qb::PrescribedInterval interval;
  interval.base_time=prepared.view.base_time;
  interval.dt=rig.dt;
  interval.sample_index=prepared.view.kinematics.base_epoch+1;
  for(unsigned i=0;i<4;++i) {
    const auto node=rig.binding->qbat_nodes(parent)[i];
    interval.position_endpoint[i]={prepared.endpoint.x[3*node],prepared.endpoint.x[3*node+1],prepared.endpoint.x[3*node+2]};
    interval.velocity_midpoint[i]={prepared.endpoint.v[3*node],prepared.endpoint.v[3*node+1],prepared.endpoint.v[3*node+2]};
    interval.omega_midpoint[i]={prepared.endpoint.omega[3*node],prepared.endpoint.omega[3*node+1],prepared.endpoint.omega[3*node+2]};
  }
  return interval;
}
void Exact(const std::vector<qb::BatchResult>& a,const std::vector<qb::BatchResult>& b) {
  ASSERT_EQ(a.size(),b.size());
  for(std::size_t parent=0;parent<a.size();++parent) {
    EXPECT_EQ(ResultValues(a[parent]),ResultValues(b[parent]))<<parent;
  }
}
void SameFields(const Fields& a,const Fields& b) {
  EXPECT_EQ(a.x,b.x);EXPECT_EQ(a.v,b.v);EXPECT_EQ(a.omega,b.omega);EXPECT_EQ(a.q,b.q);
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
}
} // namespace qbat_resident_test
