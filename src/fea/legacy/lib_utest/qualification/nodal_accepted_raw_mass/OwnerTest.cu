#include "Fixture.h"
#include "../tied_cin_runtime/NativeFixture.h"
#include <array>
#include <cstddef>
#include <limits>

namespace accepted_mass_test {
namespace cr = cin_runtime_test;
TEST_F(Cuda, NativeCurrentMassPersistsAcrossCommitDiscardAndRetry) {
  cr::Fixture fixture;
  cr::NativeState native(fixture);
  fe::FENodalState owner;
  ASSERT_EQ(cr::Initialize(owner,fixture).status,Status::Ok);
  const auto allocations=owner.allocations();
  std::vector<View> old_views;
  const auto initial_load=fixture.load;
  for (unsigned step=0;step<3;++step) {
    SCOPED_TRACE(step);
    for (std::size_t i=0;i<fixture.load.size();++i) fixture.load[i]=initial_load[i]*(step+1);
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    fe::NodalCinAssemblyView cin;
    ASSERT_NO_FATAL_FAILURE(cr::Fill(owner,fixture,token,assembly,cin));
    View view;
    ASSERT_EQ(owner.BorrowAcceptedRawMass(token,&view).status,Status::Ok);
    ASSERT_EQ(owner.AuthenticateAcceptedRawMass(token,view).status,Status::Ok);
    EXPECT_EQ(view.owner_id,assembly.owner_id);
    EXPECT_EQ(view.base_epoch,step);
    EXPECT_EQ(view.attempt,assembly.attempt);
    EXPECT_EQ(view.qualification_id,cin.qualification_id);
    EXPECT_EQ(view.stream,assembly.stream);
    ASSERT_NO_FATAL_FAILURE(SameBits(Read(view),native.mass));
    for (const auto& stale:old_views)
      EXPECT_EQ(owner.AuthenticateAcceptedRawMass(token,stale).status,Status::StaleTrial);
    old_views.push_back(view);
    for (const auto& row:fixture.rows) {
      if (!step) EXPECT_GT(native.mass[row.secondary],0.);
      else EXPECT_EQ(native.mass[row.secondary],0.);
    }
    auto next=native;
    const auto dt=fixture.Config().fixed_dt;
    ASSERT_EQ(next.Step(fixture,step*dt,dt,step?dt:dt/2),0);
    next.Drift(dt);
    ASSERT_EQ(owner.SealAssembly(token).status,Status::Ok);
    EXPECT_EQ(owner.AuthenticateAcceptedRawMass(token,view).status,Status::WrongPhase);
    auto sentinel=view;
    EXPECT_EQ(owner.BorrowAcceptedRawMass(token,&sentinel).status,Status::WrongPhase);
    Same(sentinel,view);
    ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,cr::Admission(assembly)).status,Status::Ok);
    cr::Snapshot before_commit(native.n,native.r);
    fe::NodalStamp accepted;
    ASSERT_NO_FATAL_FAILURE(cr::Accepted(owner,before_commit,accepted));
    ASSERT_NO_FATAL_FAILURE(SameBits(
      std::vector<double>(before_commit.coefficients.begin(),before_commit.coefficients.begin()+native.n),native.mass));
    if (step==1) {
      // The stale device borrow is not read after sealing. Existing accepted
      // readback proves rollback, then a fresh token grants a new borrow.
      owner.Discard();
      ASSERT_NO_FATAL_FAILURE(cr::Fill(owner,fixture,token,assembly,cin));
      View retry;
      ASSERT_EQ(owner.BorrowAcceptedRawMass(token,&retry).status,Status::Ok);
      EXPECT_NE(retry.attempt,view.attempt);
      ASSERT_NO_FATAL_FAILURE(SameBits(Read(retry),native.mass));
      EXPECT_EQ(owner.AuthenticateAcceptedRawMass(token,view).status,Status::StaleTrial);
      ASSERT_EQ(owner.SealAssembly(token).status,Status::Ok);
      ASSERT_EQ(fe::AdvanceStaggeredCin(owner,token,cr::Admission(assembly)).status,Status::Ok);
    }
    ASSERT_NO_FATAL_FAILURE(cr::Complete(owner,token,assembly));
    ASSERT_EQ(owner.Commit(token).status,Status::Ok);
    native=std::move(next);
    cr::Snapshot current(native.n,native.r);
    ASSERT_NO_FATAL_FAILURE(cr::Accepted(owner,current,accepted));
    EXPECT_EQ(accepted.epoch,step+1);
    ASSERT_NO_FATAL_FAILURE(SameBits(
      std::vector<double>(current.coefficients.begin(),current.coefficients.begin()+native.n),native.mass));
    for (const auto& row:fixture.rows) {
      EXPECT_EQ(native.mass[row.secondary],0.);
      EXPECT_GT(native.mass[row.masters[0]],fixture.mass[row.masters[0]]);
    }
  }
  EXPECT_EQ(owner.allocations().device_bytes,allocations.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocations.device_allocations);
}

TEST_F(Cuda, MetadataForeignTokensAndOutputAliasesRejectWithoutChangingOpenAttempt) {
  cr::Fixture fixture;
  fe::FENodalState owner,other;
  ASSERT_EQ(cr::Initialize(owner,fixture).status,Status::Ok);
  ASSERT_EQ(cr::Initialize(other,fixture).status,Status::Ok);
  fe::NodalTrialToken token,foreign;
  fe::NodalAssemblyView assembly,other_assembly;
  ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Status::Ok);
  ASSERT_EQ(other.BeginTrial(&foreign,&other_assembly).status,Status::Ok);
  View view;
  ASSERT_EQ(owner.BorrowAcceptedRawMass(token,&view).status,Status::Ok);
  const auto original=Read(view);
  for (unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault);
    auto bad=view;
    switch(fault) {
      case 0:++bad.mass_kg;break;
      case 1:--bad.node_count;break;
      case 2:++bad.owner_id;break;
      case 3:++bad.base_epoch;break;
      case 4:++bad.attempt;break;
      case 5:++bad.qualification_id;break;
      case 6:bad.stream=nullptr;break;
    }
    EXPECT_EQ(owner.AuthenticateAcceptedRawMass(token,bad).status,Status::StaleTrial);
    EXPECT_EQ(owner.AuthenticateAcceptedRawMass(token,view).status,Status::Ok);
  }
  auto output=view;
  EXPECT_EQ(owner.BorrowAcceptedRawMass(foreign,&output).status,Status::StaleTrial);
  Same(output,view);
  EXPECT_EQ(other.AuthenticateAcceptedRawMass(foreign,view).status,Status::StaleTrial);
  EXPECT_EQ(owner.BorrowAcceptedRawMass(token,nullptr).status,Status::InvalidInput);
  EXPECT_EQ(owner.BorrowAcceptedRawMass(token,reinterpret_cast<View*>(&owner)).status,Status::InvalidInput);
  EXPECT_EQ(owner.BorrowAcceptedRawMass(token,reinterpret_cast<View*>(&token)).status,Status::InvalidInput);
  EXPECT_EQ(owner.BorrowAcceptedRawMass(token,reinterpret_cast<View*>(const_cast<double*>(view.mass_kg))).status,Status::InvalidInput);
  alignas(View) std::array<unsigned char,sizeof(View)+alignof(View)> bytes;
  bytes.fill(0x5a);
  const auto saved=bytes;
  EXPECT_EQ(owner.BorrowAcceptedRawMass(token,reinterpret_cast<View*>(bytes.data()+1)).status,Status::InvalidInput);
  EXPECT_EQ(bytes,saved);
  const auto overflow=UINTPTR_MAX-(alignof(View)-1);
  EXPECT_EQ(owner.BorrowAcceptedRawMass(token,reinterpret_cast<View*>(overflow)).status,Status::InvalidInput);
  EXPECT_EQ(owner.AuthenticateAcceptedRawMass(token,view).status,Status::Ok);
  ASSERT_NO_FATAL_FAILURE(SameBits(Read(view),original));
  owner.Discard();
  EXPECT_NE(owner.AuthenticateAcceptedRawMass(token,view).status,Status::Ok);
  ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Status::Ok);
  EXPECT_EQ(owner.AuthenticateAcceptedRawMass(token,view).status,Status::StaleTrial);
  View retry;
  ASSERT_EQ(owner.BorrowAcceptedRawMass(token,&retry).status,Status::Ok);
  ASSERT_NO_FATAL_FAILURE(SameBits(Read(retry),original));
  owner.Discard();
  other.Discard();
}

TEST_F(Cuda, EmptyCinKeepsPositiveFixedMassAndNoStoreDoesNotInferItFromInverse) {
  cr::Fixture fixture;
  cr::tied::TiedCinAttachmentModel empty;
  ASSERT_TRUE(cr::tied::PrepareEmptyCinAttachments(fixture.source.domain,&empty));
  const auto n=fixture.mass.size();
  std::vector<std::uint8_t> fixed(n),rotation(n);
  for (std::size_t i=0;i<n;++i) {
    fixture.inverse[i]=1/fixture.mass[i];
    fixture.inverse_j[i]=1/fixture.inertia[i];
  }
  fixed[0]=7;
  rotation[0]=1;
  fixture.inverse[0]=0;
  fixture.inverse_j[0]=0;
  for (unsigned axis=0;axis<3;++axis) fixture.velocity[axis]=fixture.omega[axis]=0.;
  const fe::NodalDofConfig dofs{fixed.data(),rotation.data(),fixture.inverse_j.data()};
  const fe::NodalCinStartup startup{&empty,fixture.mass.data(),fixture.inertia.data(),nullptr,nullptr,0,991};
  fe::FENodalState owner,without_store;
  ASSERT_EQ(owner.Initialize(fixture.Config(),fixture.Kinematics(),fixture.inverse.data(),dofs,startup).status,Status::Ok);
  ASSERT_EQ(without_store.Initialize(fixture.Config(),fixture.Kinematics(),fixture.inverse.data(),dofs).status,Status::Ok);
  fe::NodalTrialToken token,other;
  fe::NodalAssemblyView assembly,other_assembly;
  ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Status::Ok);
  ASSERT_EQ(without_store.BeginTrial(&other,&other_assembly).status,Status::Ok);
  View view;
  ASSERT_EQ(owner.BorrowAcceptedRawMass(token,&view).status,Status::Ok);
  EXPECT_EQ(view.qualification_id,991u);
  ASSERT_NO_FATAL_FAILURE(SameBits(Read(view),fixture.mass));
  EXPECT_GT(fixture.mass[0],0.);
  EXPECT_EQ(fixture.inverse[0],0.);
  auto untouched=view;
  EXPECT_EQ(without_store.BorrowAcceptedRawMass(other,&untouched).status,Status::InvalidInput);
  Same(untouched,view);
  owner.Discard();
  without_store.Discard();
}
} // namespace accepted_mass_test
