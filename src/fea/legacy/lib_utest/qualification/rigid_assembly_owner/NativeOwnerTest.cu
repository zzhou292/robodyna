// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include "../nodal_rigid_group/GroupPhaseNativeFixture.h"

namespace rigid_assembly_owner_test {
namespace native = rigid_step_test;
namespace {
tl::math::Vec3 Node(const double* values,std::size_t i) {
  return {values[3*i],values[3*i+1],values[3*i+2]};
}
native::Input NativeInitial(const Fixture& f,unsigned group) {
  native::Input input{};
  const auto& source=f.binding.groups()[group];
  input.body.previous_frame=source.principal;
  input.body.mass=source.mass_kg;
  input.body.center=source.center;
  for (std::size_t k=0;k<source.member_count;++k) {
    const auto& member=f.binding.members()[source.member_offset+k];
    input.member[k]={member.position,{},{},{},{},member.mass_kg,member.isotropic_inertia_kg_m2};
  }
  return input;
}
void Compare(const fe::NodalRigidGroupSnapshot& actual,const native::Trial& expected) {
  native::Agreement(actual.state.center,expected.primary.center);
  native::Agreement(actual.state.velocity,expected.primary.velocity);
  native::Agreement(actual.state.omega,expected.primary.omega);
  for (unsigned k=0;k<9;++k)
    native::Agreement(actual.state.principal_axes.v[k],expected.primary.force_frame.axes.v[k]);
}
void Trajectory(bool use_cin,bool physical_plain=false,double point_mass_source=.002) {
  Fixture f(false,physical_plain,point_mass_source);
  EXPECT_EQ(f.binding.groups()[1].dependent_coefficients,physical_plain);
  if (physical_plain) {
    const auto node=f.domain.Find(778);
    EXPECT_EQ(f.m[node],point_mass_source*1000);
    EXPECT_EQ(f.j[node],0);
    EXPECT_EQ(f.ij[node],0);
    EXPECT_EQ(f.present[node],1);
  }
  fe::FENodalState owner;
  ASSERT_EQ(Initialize(owner,f,use_cin).status,Code::Ok);
  const auto allocation=owner.allocations();
  native::Input reference[]{NativeInitial(f,0),NativeInitial(f,1)};
  native::NativeSchedule schedule;
  Snapshot accepted(f.m.size());
  for (unsigned step=0;step<64;++step) {
    SCOPED_TRACE(step);
    const auto loads=Loads(f,step);
    const auto durations=schedule.Next(H);
    native::Trial expected[2];
    for (unsigned g=0;g<2;++g) {
      const auto& source=f.binding.groups()[g];
      reference[g].body.durations=durations;
      for (std::size_t k=0;k<source.member_count;++k) {
        const auto node=f.binding.members()[source.member_offset+k].domain_node;
        reference[g].member[k].force={loads[node],loads[f.m.size()+node],loads[2*f.m.size()+node]};
        reference[g].member[k].couple={loads[3*f.m.size()+node],loads[4*f.m.size()+node],loads[5*f.m.size()+node]};
      }
      expected[g]=g?native::NativeTwoPacket(reference[g],.001):native::NativePacket(reference[g]);
    }
    fe::NodalTrialToken token;
    fe::NodalAssemblyView view;
    Begin(owner,f,loads,use_cin,token,view);
    ASSERT_EQ(Advance(owner,token,view,use_cin).status,Code::Ok);
    std::array<fe::NodalRigidGroupSnapshot,2> candidate;
    fe::NodalPreparedView prepared;
    ASSERT_EQ(owner.CopyPreparedRigidGroups(token,{candidate.data(),candidate.size()},&prepared).status,Code::Ok);
    EXPECT_EQ(prepared.kick_dt,durations.kick_dt);
    EXPECT_EQ(prepared.rigid_groups.part_group_count,1);
    EXPECT_EQ(prepared.rigid_groups.plain_source_instance_id,29);
    for (unsigned g=0;g<2;++g) Compare(candidate[g],expected[g]);
    Commit(owner,token,view);
    accepted.Read(owner);
    for (unsigned g=0;g<2;++g) {
      Compare(accepted.groups[g],expected[g]);
      const auto& source=f.binding.groups()[g];
      for (std::size_t k=0;k<source.member_count;++k) {
        const auto node=f.binding.members()[source.member_offset+k].domain_node;
        const auto& value=expected[g].member[k];
        native::Agreement(Node(accepted.values.data(),node),value.position);
        native::Agreement(Node(accepted.values.data()+3*f.m.size(),node),value.velocity);
        native::Agreement(Node(accepted.values.data()+6*f.m.size(),node),value.omega);
        native::Agreement(Node(accepted.values.data()+13*f.m.size(),node),value.reaction_force);
        native::Agreement(Node(accepted.values.data()+16*f.m.size(),node),value.reaction_couple);
      }
      native::CarryNative(expected[g],reference[g]);
    }
    const auto i=f.ordinary,n=f.m.size();
    const double t=(step+1)*H;
    native::Agreement(accepted.values[3*i],f.x[3*i]+.5*.125*t*t);
    native::Agreement(accepted.values[3*n+3*i],.125*(step+.5)*H);
    for (unsigned axis=0;axis<3;++axis) {
      EXPECT_EQ(accepted.values[6*n+3*i+axis],0);
      EXPECT_EQ(accepted.values[16*n+3*i+axis],0);
    }
    EXPECT_EQ(accepted.stamp.epoch,step+1);
    EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
    if (::testing::Test::HasFailure()) return;
  }
  EXPECT_GT(std::abs(reference[0].member[0].velocity.x),0);
}
}
TEST_F(Cuda, AssemblyMixedOwnerMatchesIndependentNativeHistory) { Trajectory(false); }
TEST_F(Cuda, AssemblyMixedOwnerWithCinMatchesIndependentNativeRigidHistory) { Trajectory(true); }
TEST_F(Cuda, PhysicalPlainPointMassMatchesIndependentNativeHistory) {Trajectory(false,true);}
TEST_F(Cuda, PhysicalPlainPointMassAndCinMatchIndependentNativeHistory) {Trajectory(true,true);}
TEST_F(Cuda, PhysicalPlainZeroMassMatchesIndependentNativeHistory) {Trajectory(false,true,0);}
TEST_F(Cuda, PhysicalPlainZeroMassAndCinMatchIndependentNativeHistory) {Trajectory(true,true,0);}
} // namespace rigid_assembly_owner_test
