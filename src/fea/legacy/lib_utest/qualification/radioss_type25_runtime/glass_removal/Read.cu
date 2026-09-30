// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
namespace glass_removal_test {
State Rig::Read() {
  State s;
  Check(owner.CopyAccepted({s.x.data(),s.v.data(),11,s.q.data(),s.omega.data(),s.reaction.data(),s.couple.data()},&s.stamp));
  double numerical=0;fe::NodalStamp coefficient;
  Check(owner.CopyAcceptedCin({s.mass.data(),s.inertia.data(),nullptr,nullptr,&numerical,11,0},&coefficient));
  base.Require(fe::trial_identity::SameStamp(s.stamp,coefficient),"Accepted coefficient stamp differs");
  fe::qeph::BatchDiagnostics qd;fe::t3::BatchDiagnostics td;
  Check(qeph.CopyAcceptedResults(s.stamp,s.force.data(),2,&qd));
  Check(qeph.CopyAcceptedLayeredSectionHistory(s.stamp,s.section.data(),2,&qd));
  Check(qeph.CopyAcceptedFailureHistory(s.stamp,s.failure.data(),2,&qd));
  Check(triangle.CopyAcceptedResults(s.stamp,&s.triangle_force,1,&td));
  Check(triangle.CopyAcceptedLayeredSectionHistory(s.stamp,&s.triangle_section,1,&td));
  Check(triangle.CopyAcceptedFailureHistory(s.stamp,&s.triangle_failure,1,&td));
  n::Transaction* contacts[]{&self,&wall};
  for(unsigned i=0;i<2;++i){s.contacts[i].resize(7);s.flags[i].resize(7);
    Check(contacts[i]->CopyAccepted({s.contacts[i].data(),s.flags[i].data(),7},&s.publication[i]));}
  return s;
}
std::array<fe::ShellBatchFailureState,2> Rig::PreparedFailure(const Attempt& a) {
  std::array<fe::ShellBatchFailureState,2> result;
  Check(qeph.CopyPreparedFailureHistory(a.materials.qeph,result.data(),result.size()));return result;
}
std::vector<double> Rig::Force(const Attempt& a) {
  fe::NodalCinAssemblyView cin;Check(owner.BorrowCinAssembly(a.token,&cin));
  const double* columns[]{a.assembly.forces.force_x,a.assembly.forces.force_y,a.assembly.forces.force_z,
    a.assembly.forces.couple_x,a.assembly.forces.couple_y,a.assembly.forces.couple_z,
    cin.translational_stiffness,cin.rotational_stiffness};
  std::vector<double> result(8*11);
  struct Drain{cudaStream_t stream;~Drain(){cudaStreamSynchronize(stream);}} drain{a.assembly.stream};
  for(unsigned i=0;i<8;++i)Check(cudaMemcpyAsync(result.data()+11*i,columns[i],11*sizeof(double),cudaMemcpyDeviceToHost,a.assembly.stream));
  Check(cudaStreamSynchronize(a.assembly.stream));return result;
}
unsigned FailedPoints(const fe::ShellBatchFailureState& value) {
  const auto* points=value.tab1_points();if(!points)return 0;
  unsigned count=0;for(unsigned i=0;i<3;++i)count+=!points[i].point_active;return count;
}
} // namespace glass_removal_test
