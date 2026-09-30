// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/solvers/NodalCinPhysicalMains.h"
#include "../cin_physical_timestep/OwnerSupport.h"
#include <cstring>
namespace cin_main_owner {
namespace support=cin_step_owner;
namespace old=rigid_assembly_owner_test;
namespace fe=tl::fea;
using Code=fe::NodalStatus;
using Rows=std::array<fe::NodalCinPhysicalMain,2>;
std::vector<double> Acceleration(fe::FENodalState& owner,const old::Fixture& f,const fe::NodalTrialToken& token) {
  std::vector<double> field(6*f.m.size());
  std::array<fe::NodalRigidGroupAccelerationSnapshot,2> groups;
  fe::NodalPreparedView receipt;
  EXPECT_EQ(owner.CopyPreparedForceStage(token,{field.data(),field.data()+3*f.m.size(),
    f.m.size(),groups.data(),groups.size()},&receipt).status,Code::Ok);
  for(const auto& group:groups) {
    for(double value:{group.acceleration.x,group.acceleration.y,group.acceleration.z,
        group.angular_acceleration.x,group.angular_acceleration.y,group.angular_acceleration.z})
      field.push_back(value);
  }
  return field;
}
void Same(const Rows& a,const Rows& b) {
  for(unsigned i=0;i<2;++i) {
    EXPECT_EQ(a[i].source_kind,b[i].source_kind);
    EXPECT_EQ(a[i].source_group_id,b[i].source_group_id);
    EXPECT_EQ(a[i].source_node_set_id,b[i].source_node_set_id);
    EXPECT_EQ(a[i].center_m.x,b[i].center_m.x);EXPECT_EQ(a[i].center_m.y,b[i].center_m.y);
    EXPECT_EQ(a[i].center_m.z,b[i].center_m.z);EXPECT_EQ(a[i].mass_kg,b[i].mass_kg);
    EXPECT_EQ(a[i].minimum_principal_inertia_kg_m2,b[i].minimum_principal_inertia_kg_m2);
    EXPECT_EQ(a[i].translational_stiffness_n_per_m,b[i].translational_stiffness_n_per_m);
    EXPECT_EQ(a[i].rotational_stiffness_nm,b[i].rotational_stiffness_nm);
  }
}
TEST(CinPhysicalMainCuda, CompleteTt0QueryUsesActualSourceAndPreservesPreparedStateAndForceCapture) {
  old::Fixture f;
  fe::FENodalState owner;
  ASSERT_EQ(old::Initialize(owner,f,true,true).status,Code::Ok);
  const auto allocations=owner.allocations();
  std::vector<double> kn(f.m.size(),1e-6),kr(f.m.size());
  for(std::size_t i=0;i<f.m.size();++i) if(f.present[i]) kr[i]=1e-12*(i+1);
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  support::Begin(owner,f,kn,kr,token,assembly);
  Rows rows;
  fe::NodalCinPhysicalMainStamp receipt;
  EXPECT_EQ(owner.CopyPreparedCinPhysicalMains(token,{rows.data(),rows.size()},&receipt).status,Code::WrongPhase);
  ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,support::Admission(assembly)).status,Code::Ok);
  const auto prepared=support::Prepared(owner,f,token);
  const auto acceleration=Acceleration(owner,f,token);
  ASSERT_EQ(owner.CopyPreparedCinPhysicalMains(token,{rows.data(),rows.size()},&receipt).status,Code::Ok);
  EXPECT_EQ(receipt.policy,fe::NodalCinPhysicalMainPolicy::PhysicalAggregateV1);
  EXPECT_EQ(receipt.owner_id,assembly.owner_id);EXPECT_EQ(receipt.base_epoch,0u);
  EXPECT_EQ(receipt.attempt,assembly.attempt);EXPECT_EQ(receipt.owner_fixed_dt,old::H);
  EXPECT_EQ(receipt.base_time,0);EXPECT_EQ(receipt.cin_qualification_id,old::Qualification);
  EXPECT_TRUE(fe::SameRigidGroupInfo(receipt.groups,owner.rigid_groups()));
  EXPECT_EQ(rows[0].source_group_id,rows[1].source_group_id);
  EXPECT_NE(rows[0].source_kind,rows[1].source_kind); // Independent source namespaces.
  for(std::size_t g=0;g<rows.size();++g) {
    const auto& source=f.binding.groups()[g];
    const auto& row=rows[g];
    EXPECT_EQ(row.source_kind,source.source_kind);EXPECT_EQ(row.source_group_id,source.source_id);
    EXPECT_EQ(row.source_node_set_id,source.source_node_set_id);
    EXPECT_EQ(row.mass_kg,source.mass_kg);
    EXPECT_EQ(row.minimum_principal_inertia_kg_m2,std::min({source.principal.inertia.x,
      source.principal.inertia.y,source.principal.inertia.z}));
    EXPECT_EQ(row.center_m.x,source.center.x);EXPECT_EQ(row.center_m.y,source.center.y);
    EXPECT_EQ(row.center_m.z,source.center.z);
    double expected_n=0,expected_r=0;
    for(std::size_t m=0;m<source.member_count;++m) {
      const auto n=f.binding.members()[source.member_offset+m].domain_node;
      const double dx=f.x[3*n]-row.center_m.x,dy=f.x[3*n+1]-row.center_m.y,dz=f.x[3*n+2]-row.center_m.z;
      expected_n+=kn[n];expected_r+=kr[n]+(dx*dx+dy*dy+dz*dz)*kn[n];
    }
    EXPECT_EQ(row.translational_stiffness_n_per_m,expected_n);
    EXPECT_EQ(row.rotational_stiffness_nm,expected_r);
  }
  Rows again;
  fe::NodalCinPhysicalMainStamp second;
  ASSERT_EQ(owner.CopyPreparedCinPhysicalMains(token,{again.data(),again.size()},&second).status,Code::Ok);
  Same(rows,again);EXPECT_EQ(second.attempt,receipt.attempt);
  EXPECT_EQ(support::Prepared(owner,f,token),prepared);
  EXPECT_EQ(Acceleration(owner,f,token),acceleration);
  old::Commit(owner,token,assembly);
  EXPECT_EQ(owner.accepted().epoch,1u);
  EXPECT_EQ(owner.allocations().device_bytes,allocations.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocations.device_allocations);
  support::Begin(owner,f,kn,kr,token,assembly);
  ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,support::Admission(assembly)).status,Code::Ok);
  EXPECT_EQ(owner.CopyPreparedCinPhysicalMains(token,{again.data(),again.size()},&second).status,Code::WrongPhase);
  owner.Discard();
}
TEST(CinPhysicalMainCuda, CompleteOutputPreflightPreservesSourceAndCandidateThenRetryAndNewAttempt) {
  old::Fixture f;
  fe::FENodalState owner;
  ASSERT_EQ(old::Initialize(owner,f,true,true).status,Code::Ok);
  std::vector<double> kn(f.m.size(),1e-6),kr(f.m.size());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  support::Begin(owner,f,kn,kr,token,assembly);
  ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,support::Admission(assembly)).status,Code::Ok);
  Rows rows;
  rows[1].mass_kg=-719;
  const Rows initial=rows;
  fe::NodalCinPhysicalMainStamp receipt;
  receipt.owner_id=981;
  const auto prepared=support::Prepared(owner,f,token);
  EXPECT_EQ(owner.CopyPreparedCinPhysicalMains(token,{rows.data(),1},&receipt).status,Code::ResourceLimit);
  Same(rows,initial);EXPECT_EQ(receipt.owner_id,981u);
  const auto nodes=f.cin_model.domain()->nodes();
  ASSERT_GE(nodes.size()*sizeof(fe::NodalDomainNode),sizeof(Rows));
  std::vector<unsigned char> source_before(sizeof(Rows));
  std::memcpy(source_before.data(),nodes.data(),source_before.size());
  auto* alias=reinterpret_cast<fe::NodalCinPhysicalMain*>(const_cast<fe::NodalDomainNode*>(nodes.data()));
  EXPECT_EQ(owner.CopyPreparedCinPhysicalMains(token,{alias,2},&receipt).status,Code::InvalidInput);
  EXPECT_EQ(std::memcmp(source_before.data(),nodes.data(),source_before.size()),0);
  EXPECT_EQ(receipt.owner_id,981u);
  auto* stamp_alias=reinterpret_cast<fe::NodalCinPhysicalMainStamp*>(rows.data());
  EXPECT_EQ(owner.CopyPreparedCinPhysicalMains(token,{rows.data(),2},stamp_alias).status,Code::InvalidInput);
  Same(rows,initial);
  EXPECT_EQ(support::Prepared(owner,f,token),prepared);
  ASSERT_EQ(owner.CopyPreparedCinPhysicalMains(token,{rows.data(),2},&receipt).status,Code::Ok);
  const auto successful=rows;
  const auto old_token=token;
  owner.Discard();
  support::Begin(owner,f,kn,kr,token,assembly);
  ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,support::Admission(assembly)).status,Code::Ok);
  EXPECT_EQ(owner.CopyPreparedCinPhysicalMains(old_token,{rows.data(),2},&receipt).status,Code::StaleTrial);
  Same(rows,successful);
  ASSERT_EQ(owner.CopyPreparedCinPhysicalMains(token,{rows.data(),2},&receipt).status,Code::Ok);
  Same(rows,successful);EXPECT_EQ(receipt.attempt,assembly.attempt);
  old::Commit(owner,token,assembly);
}
} // namespace cin_main_owner
