#include "Fixture.h"
#include <limits>

namespace qbat_binding_test {
TEST(QbatBindingFailure, IndividuallyFiniteParentsCannotOverflowTheCompleteNativeMassLedger) {
  Fixture f;
  auto parent=f.b;
  auto& q=parent.reference.quadrilateral;
  for(auto& x:q.position) {
    x.x/=.04;
    x.y/=.02;
  }
  q.density=1e308;
  q.thickness=1;
  fe::qbat::Reference reference;
  ASSERT_EQ(fe::qbat::InitializeReference(parent.reference,reference),fe::qbat::Status::kSuccess);
  std::array<fe::ShellQbatBindingInput,2> parents{parent,parent};
  parents[1].source_parent_id++;
  const fe::ShellFormulationCollectionInput input{{nullptr,nullptr,0,0,4},parents.data(),parents.size()};
  Binding output;
  const auto before=Bytes(output);
  const auto report=output.InitializeFormulations(input);
  EXPECT_EQ(report.status,Status::NonfiniteMass);
  EXPECT_EQ(report.family,fe::ShellBindingFamily::Qbat);
  EXPECT_EQ(report.parent_index,1u);
  EXPECT_EQ(before,Bytes(output));
  // This is an arithmetic-domain test, not a physical material admission.
  for(auto& p:parents) p.reference.quadrilateral.density=1000;
  ASSERT_EQ(output.InitializeFormulations(input).status,Status::Success);
  Reduction(output);
}

TEST(QbatBindingFailure, LateQbatSourceGeometryAndSharedBitsRejectWithoutPublicationThenRetry) {
  const Fixture original;
  for(unsigned failure=0;failure<6;++failure) {
    auto f=original;
    if(failure==0) f.b.source_parent_id=f.q[1].source_parent_id;
    if(failure==1) f.b.nodes[3]=f.b.nodes[0];
    if(failure==2) f.b.reference.options.nptr=1;
    if(failure==3) f.b.reference.quadrilateral.position[3].x+=.0001;
    if(failure==4) f.b.reference.quadrilateral.node_ids[3]++;
    if(failure==5) f.b.reference.quadrilateral.position[0].z=-0.;
    Binding output;
    const auto before=Bytes(output);
    const auto report=output.InitializeFormulations(f.Input());
    const Status expected[]{Status::InvalidParentIdentity,Status::InvalidConnectivity,
        Status::InvalidQbatReference,Status::PositionMismatch,Status::IdentityMismatch,Status::PositionMismatch};
    EXPECT_EQ(report.status,expected[failure])<<failure<<report.message;
    EXPECT_EQ(report.family,fe::ShellBindingFamily::Qbat);
    EXPECT_EQ(report.parent_index,0u);
    EXPECT_EQ(before,Bytes(output));
    ASSERT_EQ(output.InitializeFormulations(original.Input()).status,Status::Success);
    Reduction(output);
  }
}

TEST(QbatBindingFailure, CountExtentAndExactPayloadScratchLimitsPrecedeBorrowedReads) {
  Fixture f;
  Binding measured;
  ASSERT_EQ(measured.InitializeFormulations(f.Input()).status,Status::Success);
  for(unsigned failure=0;failure<6;++failure) {
    auto input=f.Input();
    fe::ShellHostBindingLimits limits;
    if(failure==0) {
      input.qbat=reinterpret_cast<const fe::ShellQbatBindingInput*>(8);
      input.qbat_count=SIZE_MAX;
    }
    if(failure==1) {
      input.qbat=reinterpret_cast<const fe::ShellQbatBindingInput*>(8);
      limits.max_owned_bytes=1;
    }
    if(failure==2) {
      input.qbat=reinterpret_cast<const fe::ShellQbatBindingInput*>(8);
      limits.max_startup_scratch_bytes=1;
    }
    if(failure==3) input.qbat=reinterpret_cast<const fe::ShellQbatBindingInput*>(1);
    if(failure==4) limits.max_owned_bytes=measured.host_bytes()-1;
    if(failure==5) limits.max_startup_scratch_bytes=measured.startup_scratch_bytes()-1;
    Binding output;
    const auto before=Bytes(output);
    EXPECT_NE(output.InitializeFormulations(input,limits).status,Status::Success)<<failure;
    EXPECT_EQ(before,Bytes(output));
    limits.max_owned_bytes=measured.host_bytes();
    limits.max_startup_scratch_bytes=measured.startup_scratch_bytes();
    ASSERT_EQ(output.InitializeFormulations(f.Input(),limits).status,Status::Success);
    EXPECT_EQ(output.inventory(),measured.inventory());
  }
  Binding output;
  auto input=f.Input();
  input.qbat=nullptr;
  input.qbat_count=0;
  EXPECT_EQ(output.InitializeFormulations(input).status,Status::InvalidInput);
  EXPECT_FALSE(output.prepared());
}
} // namespace qbat_binding_test
