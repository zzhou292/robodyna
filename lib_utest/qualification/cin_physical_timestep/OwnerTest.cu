// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerSupport.h"
namespace cin_step_owner {
TEST(CinPhysicalStepCuda, LoadedPartPlainOrdinaryAndCinLateBodyRejectRetryPreserveAllState) {
  old::Fixture f;
  fe::FENodalState owner;
  ASSERT_EQ(old::Initialize(owner,f,true).status,Code::Ok);
  ASSERT_EQ(f.m[f.zero_mass],0);
  ASSERT_EQ(f.j[f.zero_mass],0);
  const auto allocation=owner.allocations();
  std::vector<double> kn(f.m.size(),1e-6),kr(f.m.size());
  for(std::size_t node=0;node<f.m.size();++node) if(f.present[node]) kr[node]=1e-12;
  for(unsigned epoch=0;epoch<2;++epoch) {
    old::Snapshot accepted(f.m.size()),after(f.m.size());
    accepted.Read(owner);
    Coefficients coefficients(f),after_coefficients(f);
    coefficients.Read(owner);
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    Begin(owner,f,kn,kr,token,assembly);
    auto result=fe::AdvanceStaggeredCin(owner,token,Admission(assembly));
    ASSERT_EQ(result.status,Code::Ok) << result.message << " bound " << result.stable_dt;
    EXPECT_GT(result.stable_dt,old::H);
    const auto clean=Prepared(owner,f,token);
    owner.Discard();
    const auto last=f.binding.groups()[epoch==0?1:0];
    const auto last_node=epoch==0
      ? f.binding.members()[last.member_offset+last.member_count-1].domain_node : f.zero_mass;
    const auto original=kn[last_node];
    kn[last_node]=1e30;
    Begin(owner,f,kn,kr,token,assembly);
    result=fe::AdvanceStaggeredCin(owner,token,Admission(assembly));
    EXPECT_EQ(result.status,Code::StepTooLarge);
    EXPECT_EQ(result.node,f.binding.members()[last.member_offset].domain_node);
    EXPECT_GT(result.stable_dt,0);
    EXPECT_LT(result.stable_dt,old::H);
    after.Read(owner);after_coefficients.Read(owner);
    old::Same(accepted,after);
    EXPECT_EQ(coefficients.values,after_coefficients.values);
    owner.Discard();
    kn[last_node]=original;
    Begin(owner,f,kn,kr,token,assembly);
    ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,Admission(assembly)).status,Code::Ok);
    EXPECT_EQ(Prepared(owner,f,token),clean);
    old::Commit(owner,token,assembly);
    EXPECT_EQ(owner.accepted().epoch,epoch+1);
    EXPECT_EQ(owner.accepted().reaction_kick_dt,epoch?old::H:.5*old::H);
  }
  EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
}
TEST(CinPhysicalStepCuda, ActualTransferredSecondaryStiffnessAndOrdinaryRotationBoundBeforeKick) {
  old::Fixture f;
  fe::FENodalState owner;
  ASSERT_EQ(old::Initialize(owner,f,true).status,Code::Ok);
  old::Snapshot initial(f.m.size()),after(f.m.size());initial.Read(owner);
  Coefficients initial_coefficients(f),after_coefficients(f);initial_coefficients.Read(owner);
  std::vector<double> kn(f.m.size()),kr(f.m.size());
  const auto secondary=f.cin_model.rows().data[f.cin_model.rows().count-1].secondary_domain_node;
  kn[secondary]=1e30;
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  Begin(owner,f,kn,kr,token,assembly);
  auto result=fe::AdvanceStaggeredCin(owner,token,Admission(assembly));
  EXPECT_EQ(result.status,Code::StepTooLarge);
  EXPECT_NE(result.node,secondary); // Actual transferred master, never zero-M secondary inverse.
  EXPECT_GT(result.stable_dt,0);
  EXPECT_LT(result.stable_dt,old::H);
  after.Read(owner);after_coefficients.Read(owner);
  old::Same(initial,after);EXPECT_EQ(initial_coefficients.values,after_coefficients.values);
  owner.Discard();
  // Old explicitly unscreened operation preserves its historical behavior.
  Begin(owner,f,kn,kr,token,assembly);
  EXPECT_EQ(fe::AdvanceStaggeredCin(owner,token,Admission(assembly,false)).status,Code::Ok);
  owner.Discard();
  kn[secondary]=0;
  std::size_t ordinary=f.m.size();
  for(std::size_t node=0;node<f.m.size();++node) {
    if(!f.present[node] || f.binding.FindMember(node) || f.j[node]<=0) continue;
    bool cin_role=false;
    const auto rows=f.cin_model.rows();
    for(std::size_t i=0;i<rows.count;++i) {
      const auto& row=rows.data[i];
      cin_role=cin_role || row.secondary_domain_node==node;
      for(auto master:row.master_domain_nodes) cin_role=cin_role || master==node;
    }
    if(!cin_role) { ordinary=node; break; }
  }
  ASSERT_LT(ordinary,f.m.size());
  ASSERT_GT(f.j[ordinary],0);
  kr[ordinary]=8*f.j[ordinary]/(old::H*old::H);
  Begin(owner,f,kn,kr,token,assembly);
  result=fe::AdvanceStaggeredCin(owner,token,Admission(assembly));
  EXPECT_EQ(result.status,Code::StepTooLarge);
  EXPECT_GT(result.stable_dt,0);EXPECT_LT(result.stable_dt,old::H);
  after.Read(owner);old::Same(initial,after);
  owner.Discard();
  kr[ordinary]=0;
  Begin(owner,f,kn,kr,token,assembly);
  auto admission=Admission(assembly);
  admission.structural.factor=0;
  EXPECT_EQ(fe::AdvanceStaggeredCin(owner,token,admission).status,Code::MissingStepAdmission);
  owner.Discard();
  Begin(owner,f,kn,kr,token,assembly);
  ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,Admission(assembly)).status,Code::Ok);
  old::Commit(owner,token,assembly);
}
} // namespace cin_step_owner
