#include "WallCoupledFixture.h"

namespace qeph_wall_test {
void Prefix(unsigned cells) {
  for(unsigned refinement:{1u,2u}) {
    SCOPED_TRACE(refinement);
    WallRig w(cells); ASSERT_TRUE(w.Initialize(H0/refinement)); auto& r=w.shell;
    NativeSequence native; ASSERT_TRUE(native.Initialize(r));
    const auto owner_allocation=r.owner.allocations(),shell_allocation=r.batch.allocations(),wall_allocation=w.wall.allocations();
    sc::NodalWallDeviceResults published;
    LedgerEvidence source; ContactEvidence contact;
    const auto suffix="_refinement_"+std::to_string(refinement);
    for(unsigned step=0;step<4*refinement;++step) {
      SCOPED_TRACE(step);
      Snapshot base; ASSERT_TRUE(Read(r.owner,base)); OwnerAgreement(r,base,native.state);
      EXPECT_EQ(base.stamp.epoch,native.epoch); EXPECT_EQ(base.stamp.time,native.time);
      PortResults cache{}; q::BatchDiagnostics accepted; ASSERT_TRUE(ReadAccepted(r,cache,accepted));
      // Recompute the stateless contact law on the independent native BASE,
      // not on the candidate or by reusing the device RHS/cache.
      const auto native_contact=w.Host(native.state,native.epoch,1);
      NativeProposal expected; ASSERT_TRUE(native.Propose(r,ContactLoads(native_contact),expected));
      Trial t; ASSERT_TRUE(Begin(w,t));
      const auto host=w.Host(base,base.stamp.epoch,t.nodal.view.attempt);
      const auto load=ContactLoads(host);
      ContactAgreement(t.contact_base_result,host);
      Staged s; ASSERT_TRUE(Evaluate(w,t,s,step%2)); Identity(w,t,s);
      Agreement(w,t,s,expected,native.time,native.epoch);
      CheckLedgers(r,base,t.nodal,load,cache,s.shell,source);
      CheckSourceWork(r,cache,s.elements,s.shell,source);
      ContactLedgers(w,base,t,s,contact); Controls(w,base,t,s,load,contact);
      double internal_norm=0;
      for(unsigned e=0;e<r.count;++e) for(unsigned i=0;i<4;++i)
        internal_norm+=qeph_startup_test::Length(s.elements[e].internal_force[i])+
                       qeph_startup_test::Length(s.elements[e].internal_couple[i])/Side;
      EXPECT_GT(internal_norm,0.);
      if(!step) {
        EXPECT_GT(t.contact_base.potential.lower,0.);
        Property("initial_contact_potential_J"+suffix,t.contact_base.potential.value);
      }
      if(step+1==4*refinement) {
        // Both controls are mandatory before the final receipt, with frozen
        // budgets and a failed gate retained if the signal is too small.
        EXPECT_GT(contact.omitted_contact,32.); EXPECT_GT(contact.omitted_shell,32.);
        Property("omitted_contact_budget_ratio"+suffix,contact.omitted_contact);
        Property("omitted_shell_budget_ratio"+suffix,contact.omitted_shell);
        Property("omitted_contact_base_epoch"+suffix,contact.omitted_contact_epoch);
        Property("omitted_shell_base_epoch"+suffix,contact.omitted_shell_epoch);
        Property("premature_endpoint_budget_ratio_diagnostic"+suffix,contact.early_endpoint);
      }
      ASSERT_FALSE(::testing::Test::HasFailure())<<"No CW0 receipt after a failed source/contact/ledger check";
      const auto saved_elements=Bytes(s.elements);
      const auto saved_contact=Bytes(s.contact_result);
      ASSERT_TRUE(Publish(w,t,s,published)); native.Accept(r,expected);
      EXPECT_EQ(Bytes(published),saved_contact);
      Snapshot committed; ASSERT_TRUE(Read(r.owner,committed)); OwnerAgreement(r,committed,native.state);
      EXPECT_EQ(committed.x,t.nodal.state.x); EXPECT_EQ(committed.v,t.nodal.state.v);
      EXPECT_EQ(committed.omega,t.nodal.state.omega); EXPECT_EQ(committed.q,t.nodal.state.q);
      ASSERT_TRUE(ReadAccepted(r,cache,accepted)); EXPECT_EQ(Bytes(cache),saved_elements);
      EXPECT_EQ(accepted.epoch,native.epoch); EXPECT_EQ(accepted.attempt,s.shell.attempt);
      EXPECT_EQ(published.diagnostics.base_epoch+1,accepted.epoch);
      EXPECT_EQ(published.diagnostics.phase,sc::NodalWallDevicePhase::PreparedCandidate);
      EXPECT_EQ(published.diagnostics.time,accepted.time); // Retained completed interval association.
      EXPECT_EQ(r.owner.allocations().device_bytes,owner_allocation.device_bytes);
      EXPECT_EQ(r.owner.allocations().device_allocations,owner_allocation.device_allocations);
      EXPECT_EQ(r.batch.allocations().device_bytes,shell_allocation.device_bytes);
      EXPECT_EQ(r.batch.allocations().device_allocations,shell_allocation.device_allocations);
      EXPECT_EQ(w.wall.allocations().device_bytes,wall_allocation.device_bytes);
      EXPECT_EQ(w.wall.allocations().device_allocations,wall_allocation.device_allocations);
    }
    EXPECT_EQ(r.owner.accepted().epoch,4*refinement); EXPECT_EQ(r.owner.accepted().time,4*H0);
    Property("maximum_kick_ledger_budget_ratio"+suffix,source.kick_ratio);
    Property("maximum_momentum_ledger_budget_ratio"+suffix,source.momentum_ratio);
    Property("maximum_angular_ledger_budget_ratio"+suffix,source.angular_ratio);
    Property("maximum_internal_work_budget_ratio"+suffix,source.internal_work_ratio);
    Property("maximum_source_work_budget_ratio"+suffix,source.source_work_ratio);
    Property("maximum_contact_work_budget_ratio"+suffix,contact.work_ratio);
    Property("maximum_contact_impulse_budget_ratio"+suffix,contact.impulse_ratio);
    Property("maximum_contact_defect_budget_ratio"+suffix,contact.defect_ratio);
    Property("final_contact_depth_m"+suffix,published.diagnostics.maximum_penetration);
    Property("final_source_work_J"+suffix,source.source_internal_work);
    Property("fixed_dt_s"+suffix,r.h);
  }
  ::testing::Test::RecordProperty("native_cell_intervals",12*cells);
  ::testing::Test::RecordProperty("incoming_impact_qualified","false");
  ::testing::Test::RecordProperty("combined_long_response_stability_qualified","false");
}
} // namespace qeph_wall_test
