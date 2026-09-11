// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../rigid_assembly_owner/OwnerFixture.h"
namespace cin_step_owner {
namespace old=rigid_assembly_owner_test;
namespace fe=tl::fea;
using Code=fe::NodalStatus;
struct Coefficients {
  std::size_t n,r;
  std::vector<double> values;
  explicit Coefficients(const old::Fixture& f):n(f.m.size()),r(f.cin_model.rows().count),values(2*n+2*r+1) {}
  fe::NodalCinSnapshotBuffer Buffer() {
    return {values.data(),values.data()+n,values.data()+2*n,values.data()+2*n+r,
      values.data()+2*n+2*r,n,r};
  }
  void Read(fe::FENodalState& owner) {
    fe::NodalStamp stamp;
    ASSERT_EQ(owner.CopyAcceptedCin(Buffer(),&stamp).status,Code::Ok);
  }
};
inline void Begin(fe::FENodalState& owner,const old::Fixture& f,
    const std::vector<double>& kn,const std::vector<double>& kr,
    fe::NodalTrialToken& token,fe::NodalAssemblyView& assembly) {
  ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Code::Ok);
  const auto load=old::Loads(f);
  double* fields[]{assembly.forces.force_x,assembly.forces.force_y,assembly.forces.force_z,
    assembly.forces.couple_x,assembly.forces.couple_y,assembly.forces.couple_z};
  for(unsigned axis=0;axis<6;++axis) {
    ASSERT_EQ(cudaMemcpyAsync(fields[axis],load.data()+axis*f.m.size(),f.m.size()*sizeof(double),
      cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
  }
  fe::NodalCinAssemblyView cin;
  ASSERT_EQ(owner.BorrowCinAssembly(token,&cin).status,Code::Ok);
  ASSERT_EQ(cudaMemcpyAsync(cin.translational_stiffness,kn.data(),kn.size()*sizeof(double),
    cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(cin.rotational_stiffness,kr.data(),kr.size()*sizeof(double),
    cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
  ASSERT_EQ(cudaMemsetAsync(cin.witness_activity,1,cin.witness_count,assembly.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
  ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
}
inline fe::NodalCinAdmission Admission(const fe::NodalAssemblyView& assembly,bool enabled=true) {
  fe::NodalCinAdmission result{assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,
    old::Qualification,old::H,.2,true};
  if(enabled) result.structural={fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8};
  return result;
}
inline std::vector<double> Prepared(fe::FENodalState& owner,const old::Fixture& f,
    const fe::NodalTrialToken& token) {
  old::Snapshot fields(f.m.size());
  Coefficients coefficients(f);
  fe::NodalPreparedView receipt;
  EXPECT_EQ(owner.CopyPrepared(token,fields.Buffer(),&receipt).status,Code::Ok);
  EXPECT_EQ(owner.CopyPreparedRigidGroups(token,{fields.groups.data(),fields.groups.size()},&receipt).status,Code::Ok);
  EXPECT_EQ(owner.CopyPreparedCin(token,coefficients.Buffer(),&receipt).status,Code::Ok);
  std::vector<double> values=fields.values;
  values.insert(values.end(),coefficients.values.begin(),coefficients.values.end());
  for(const auto& group:fields.groups) {
    double state[18];
    fe::rigid::WriteGroupState(state,group.state);
    values.insert(values.end(),state,state+18);
  }
  return values;
}
} // namespace cin_step_owner
