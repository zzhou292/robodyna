// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace rigid_assembly_owner_test {
TEST_F(Cuda, AssemblySourceAndRoleRejectionBeforeAllocationAndRetry) {
  Fixture f;
  fe::FENodalState owner;
  auto config=f.Config();
  config.rigid_limits=fe::NodalRigidOwnerLimits::Vehicle();
  EXPECT_EQ(owner.Initialize(config,f.Kinematics(),f.im.data(),f.Dofs(),f.binding).status,Code::ResourceLimit);
  EXPECT_EQ(owner.allocations().device_allocations,0);
  const double x=f.x[3*f.ordinary];
  f.x[3*f.ordinary]=std::nextafter(x,INFINITY);
  EXPECT_EQ(Initialize(owner,f).status,Code::InvalidInput);
  f.x[3*f.ordinary]=x;
  const double mass=f.m[f.ordinary];
  f.m[f.ordinary]*=2;
  EXPECT_EQ(Initialize(owner,f).status,Code::InvalidInput);
  f.m[f.ordinary]=mass;
  f.present[f.zero_mass]=0;
  EXPECT_EQ(Initialize(owner,f).status,Code::InvalidInput);
  f.present[f.zero_mass]=1;
  EXPECT_EQ(owner.allocations().device_allocations,0);
  ASSERT_EQ(Initialize(owner,f).status,Code::Ok);
  Snapshot initial(f.m.size());
  initial.Read(owner);
  EXPECT_EQ(initial.groups[0].source_kind,fe::RigidBindingSourceKind::Part);
  EXPECT_EQ(initial.groups[1].source_kind,fe::RigidBindingSourceKind::NodalGroup);
  EXPECT_EQ(initial.groups[0].source_group_id,initial.groups[1].source_group_id);
}
TEST_F(Cuda, LateSecondGroupFailureAndReadbackFaultPreserveAcceptedStateAndRetry) {
  Fixture f;
  fe::FENodalState owner;
  ASSERT_EQ(Initialize(owner,f).status,Code::Ok);
  Snapshot initial(f.m.size()),unchanged(f.m.size());
  initial.Read(owner);
  auto loads=Loads(f);
  const auto last=f.binding.members()[f.binding.members().size()-1].domain_node;
  loads[5*f.m.size()+last]=1e12;
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  Begin(owner,f,loads,false,token,view);
  EXPECT_EQ(Advance(owner,token,view,false).status,Code::StepTooLarge);
  unchanged.Read(owner);
  Same(initial,unchanged);
  Begin(owner,f,Loads(f),false,token,view);
  ASSERT_EQ(Advance(owner,token,view,false).status,Code::Ok);
  fe::NodalPreparedView prepared;
  ASSERT_EQ(owner.BorrowPrepared(token,&prepared).status,Code::Ok);
  // A late group-state readback fault must not publish even the first row.
  const double bad=std::numeric_limits<double>::quiet_NaN();
  auto* tail=const_cast<double*>(prepared.kinematics.position_xyz)+19*f.m.size();
  ASSERT_EQ(cudaMemcpyAsync(tail+18,&bad,sizeof(double),cudaMemcpyHostToDevice,prepared.stream),cudaSuccess);
  std::array<fe::NodalRigidGroupSnapshot,2> out=initial.groups;
  const auto before=out;
  fe::NodalPreparedView output;
  output.owner_id=1234;
  EXPECT_EQ(owner.CopyPreparedRigidGroups(token,{out.data(),out.size()},&output).status,Code::InvalidOutput);
  EXPECT_EQ(std::memcmp(out.data(),before.data(),sizeof(out)),0);
  EXPECT_EQ(output.owner_id,1234);
  unchanged.Read(owner);
  Same(initial,unchanged);
  Begin(owner,f,Loads(f),false,token,view);
  ASSERT_EQ(Advance(owner,token,view,false).status,Code::Ok);
  Commit(owner,token,view);
  EXPECT_EQ(owner.accepted().epoch,1);
}
TEST_F(Cuda, CinZeroInversesAcrossInitialPreparedAcceptedAndCaptureSourceKinds) {
  Fixture f;
  fe::FENodalState owner;
  ASSERT_EQ(Initialize(owner,f,true,true).status,Code::Ok);
  const auto n=f.m.size(),rows=f.ranges.size();
  std::vector<double> coefficients(2*n+2*rows+1,-55.);
  fe::NodalCinSnapshotBuffer out{coefficients.data(),coefficients.data()+n,
    coefficients.data()+2*n,coefficients.data()+2*n+rows,coefficients.data()+2*n+2*rows,n,rows};
  fe::NodalStamp stamp;
  ASSERT_EQ(owner.CopyAcceptedCin(out,&stamp).status,Code::Ok);
  EXPECT_EQ(coefficients[f.zero_mass],0);
  EXPECT_EQ(coefficients[n+f.zero_mass],0);
  EXPECT_EQ(coefficients[n+f.ordinary],0);
  for (unsigned step=0;step<3;++step) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView view;
    Begin(owner,f,Loads(f,step),true,token,view);
    double inverse[2];
    ASSERT_EQ(cudaMemcpyAsync(inverse,view.mass.inverse_mass+f.zero_mass,sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(inverse+1,view.inverse_inertia+f.zero_mass,sizeof(double),cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
    EXPECT_EQ(inverse[0],0);
    EXPECT_EQ(inverse[1],0);
    ASSERT_EQ(Advance(owner,token,view,true).status,Code::Ok);
    fe::NodalPreparedView prepared;
    ASSERT_EQ(owner.CopyPreparedCin(token,out,&prepared).status,Code::Ok);
    for (const auto& member:f.binding.members()) {
      EXPECT_EQ(coefficients[member.domain_node],member.mass_kg);
      EXPECT_EQ(coefficients[n+member.domain_node],member.isotropic_inertia_kg_m2);
    }
    std::vector<double> acceleration(6*n);
    std::array<fe::NodalRigidGroupAccelerationSnapshot,2> groups;
    ASSERT_EQ(owner.CopyPreparedForceStage(token,{acceleration.data(),acceleration.data()+3*n,n,groups.data(),2},&prepared).status,Code::Ok);
    EXPECT_EQ(groups[0].source_kind,fe::RigidBindingSourceKind::Part);
    EXPECT_EQ(groups[1].source_kind,fe::RigidBindingSourceKind::NodalGroup);
    EXPECT_TRUE(std::isfinite(acceleration[3*f.zero_mass]));
    Commit(owner,token,view);
    ASSERT_EQ(owner.CopyAcceptedCin(out,&stamp).status,Code::Ok);
    EXPECT_EQ(stamp.epoch,step+1);
    EXPECT_EQ(coefficients[f.zero_mass],0);
    EXPECT_EQ(coefficients[n+f.ordinary],0);
  }
}
TEST_F(Cuda, PhysicalPlainZeroCoefficientCinReadbackAndLateRejectionKeepAcceptedState) {
  Fixture f(false,true,0);
  fe::FENodalState owner;
  ASSERT_EQ(Initialize(owner,f,true).status,Code::Ok);
  const auto node=f.domain.Find(778),n=f.m.size(),rows=f.ranges.size();
  std::vector<double> coefficients(2*n+2*rows+1,-55.);
  fe::NodalCinSnapshotBuffer out{coefficients.data(),coefficients.data()+n,
    coefficients.data()+2*n,coefficients.data()+2*n+rows,coefficients.data()+2*n+2*rows,n,rows};
  fe::NodalStamp stamp;
  ASSERT_EQ(owner.CopyAcceptedCin(out,&stamp).status,Code::Ok);
  EXPECT_EQ(coefficients[node],0);
  EXPECT_EQ(coefficients[n+node],0);
  Snapshot initial(n),unchanged(n);
  initial.Read(owner);
  auto bad=Loads(f);
  bad[5*n+node]=1e12;
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  Begin(owner,f,bad,true,token,view);
  EXPECT_EQ(Advance(owner,token,view,true).status,Code::StepTooLarge);
  unchanged.Read(owner);
  Same(initial,unchanged);
  owner.Discard();
  for(unsigned step=0;step<3;++step) {
    Begin(owner,f,Loads(f,step),true,token,view);
    ASSERT_EQ(Advance(owner,token,view,true).status,Code::Ok);
    fe::NodalPreparedView prepared;
    ASSERT_EQ(owner.CopyPreparedCin(token,out,&prepared).status,Code::Ok);
    EXPECT_EQ(coefficients[node],0);
    EXPECT_EQ(coefficients[n+node],0);
    ASSERT_EQ(owner.ValidateRigidAssemblyBinding(f.binding).status,Code::Ok);
    Commit(owner,token,view);
    ASSERT_EQ(owner.CopyAcceptedCin(out,&stamp).status,Code::Ok);
    EXPECT_EQ(coefficients[node],0);
    EXPECT_EQ(coefficients[n+node],0);
    EXPECT_EQ(stamp.epoch,step+1);
  }
}
} // namespace rigid_assembly_owner_test
