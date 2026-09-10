#include "GroupOwnerFixture.h"
#include <memory>
namespace rigid_owner_test {
TEST_F(Cuda,StartupOwnsExactSourceAssociationAndPreservesLegacyAllocationContract) {
  auto fixture=std::make_unique<Fixture>(); fe::FENodalState owner,legacy;
  ASSERT_EQ(fixture->Initialize(owner).status,Code::Ok);
  ASSERT_EQ(fixture->input.Initialize(legacy).status,Code::Ok);
  EXPECT_EQ(legacy.allocations().device_allocations,6u);
  EXPECT_EQ(owner.allocations().device_allocations,7u);
  const auto allocation=owner.allocations(); EXPECT_LE(allocation.device_bytes,fe::MaxTranslationDeviceBytes);
  EXPECT_GT(allocation.device_bytes,legacy.allocations().device_bytes);
  Groups initial; ASSERT_TRUE(Read(owner,initial));
  EXPECT_TRUE(fe::SameRigidGroupInfo(owner.rigid_groups(),{Source,2,8}));
  EXPECT_TRUE(fe::SameRigidGroupInfo(initial.stamp.rigid_groups,owner.rigid_groups()));
  for(unsigned g=0;g<2;++g) {
    EXPECT_EQ(initial.values[g].source_group_id,300+g); EXPECT_EQ(initial.values[g].source_node_set_id,400+g);
    rigid_step_test::Agreement(initial.values[g].state.center,fixture->model.groups()[g].center,0);
    rigid_step_test::Agreement(initial.values[g].state.velocity,{.3,-.2,.1},0);
    rigid_step_test::Agreement(initial.values[g].state.omega,{},0);
  }
  const auto loads=fixture->Load(); fixture.reset(); // All source/model storage is now gone.
  ASSERT_TRUE(Step(owner,loads));
  EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
}

TEST_F(Cuda,StartupRejectsMismatchedSourceStateBeforePublicationAndCanRetry) {
  for(unsigned bad=0;bad<8;++bad) {
    SCOPED_TRACE(bad); Fixture fixture; const auto good=fixture.input; fe::FENodalState owner;
    switch(bad) {
      case 0: fixture.input.x[0]+=.001; break;
      case 1: fixture.input.inverse[0]=.4; break;
      case 2: fixture.input.inverse_inertia[0]=900; break;
      case 3: fixture.input.fixed[0]=1; fixture.input.v[0]=0; break;
      case 4: fixture.input.rotation_fixed[0]=1; fixture.input.inverse_inertia[0]=0; break;
      case 5: fixture.input.omega[0]=.001; break;
      case 6: fixture.input.v[3]+=.001; break;
      case 7: --fixture.input.n; break;
    }
    EXPECT_NE(fixture.Initialize(owner).status,Code::Ok);
    EXPECT_EQ(owner.accepted().owner_id,0u); EXPECT_EQ(owner.allocations().device_allocations,0u);
    EXPECT_EQ(owner.rigid_groups().group_count,0u);
    fixture.input=good; ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  }
  Fixture fixture; fe::FENodalState owner;
  EXPECT_EQ(fixture.Initialize(owner,fe::MaxTranslationDeviceBytes,fe::NodalTemporalScheme::VelocityFirst).status,
    Code::UnsupportedTemporalScheme);
  EXPECT_EQ(owner.allocations().device_allocations,0u);
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
}

TEST_F(Cuda,WholeOwnerByteBudgetIncludesBothGroupHistoriesAndImmutableArena) {
  Fixture fixture; fe::FENodalState measured,rejected,exact;
  ASSERT_EQ(fixture.Initialize(measured).status,Code::Ok);
  const auto bytes=measured.allocations().device_bytes;
  EXPECT_EQ(fixture.Initialize(rejected,bytes-1).status,Code::ResourceLimit);
  EXPECT_EQ(rejected.allocations().device_allocations,0u); EXPECT_EQ(rejected.accepted().owner_id,0u);
  ASSERT_EQ(fixture.Initialize(exact,bytes).status,Code::Ok);
  EXPECT_EQ(exact.allocations().device_bytes,bytes);
}

TEST_F(Cuda,DiagnosticReadbackIsFailureAtomicAndDescriptorCannotBeForged) {
  Fixture fixture; fe::FENodalState owner; ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  Groups output; output.values[0].source_group_id=919; output.stamp.owner_id=929; const auto saved=output;
  EXPECT_EQ(owner.CopyAcceptedRigidGroups({output.values.data(),1},&output.stamp).status,Code::ResourceLimit);
  SameGroups(output,saved);
  EXPECT_EQ(owner.CopyAcceptedRigidGroups({nullptr,2},&output.stamp).status,Code::InvalidInput);
  SameGroups(output,saved);
  EXPECT_EQ(owner.CopyAcceptedRigidGroups(output.buffer(),reinterpret_cast<fe::NodalStamp*>(output.values.data())).status,
    Code::InvalidInput); SameGroups(output,saved);
  auto* overflow=reinterpret_cast<fe::NodalRigidGroupSnapshot*>(UINTPTR_MAX-sizeof(fe::NodalRigidGroupSnapshot)+1);
  EXPECT_EQ(owner.CopyAcceptedRigidGroups({overflow,2},&output.stamp).status,Code::InvalidInput);
  SameGroups(output,saved);
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_TRUE(nt::BeginLoad(owner,fixture.Load(),token,view));
  EXPECT_TRUE(fe::SameRigidGroupInfo(view.rigid_groups,owner.rigid_groups()));
  EXPECT_EQ(view.mass.model,tlfea::contact::TranslationMassModel::kUnspecified);
  auto forged=view; ++forged.rigid_groups.source_instance_id;
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(forged).status,Code::StaleTrial);
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(view).status,Code::Ok);
  fe::NodalPreparedView prepared; prepared.owner_id=949;
  const auto prepared_before=rigid_step_test::Bytes(prepared);
  EXPECT_EQ(owner.CopyPreparedRigidGroups(token,output.buffer(),&prepared).status,Code::WrongPhase);
  EXPECT_EQ(rigid_step_test::Bytes(prepared),prepared_before); SameGroups(output,saved);
  owner.Discard(); ASSERT_TRUE(Prepare(owner,fixture.Load(),token,view));
  const auto token_before=rigid_step_test::Bytes(token);
  auto* token_groups=reinterpret_cast<fe::NodalRigidGroupSnapshot*>(&token);
  EXPECT_EQ(owner.CopyPreparedRigidGroups(token,{token_groups,2},&prepared).status,Code::InvalidInput);
  EXPECT_EQ(rigid_step_test::Bytes(token),token_before);
  auto* token_prepared=reinterpret_cast<fe::NodalPreparedView*>(&token);
  EXPECT_EQ(owner.CopyPreparedRigidGroups(token,output.buffer(),token_prepared).status,Code::InvalidInput);
  EXPECT_EQ(rigid_step_test::Bytes(token),token_before);
  EXPECT_EQ(rigid_step_test::Bytes(prepared),prepared_before); SameGroups(output,saved);
  EXPECT_EQ(owner.CopyPreparedRigidGroups(token,{output.values.data(),1},&prepared).status,Code::ResourceLimit);
  EXPECT_EQ(rigid_step_test::Bytes(prepared),prepared_before); SameGroups(output,saved);
  ASSERT_EQ(owner.CopyPreparedRigidGroups(token,output.buffer(),&prepared).status,Code::Ok);
  EXPECT_TRUE(fe::SameRigidGroupInfo(prepared.rigid_groups,owner.rigid_groups()));
  EXPECT_EQ(prepared.kick_dt,fixture.input.h/2);
  owner.Discard();
  EXPECT_NE(owner.CopyPreparedRigidGroups(token,output.buffer(),&prepared).status,Code::Ok);
}
} // namespace rigid_owner_test
