// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"

namespace qt_mapped_test {
template<class Family> class QtMappedCuda : public ::testing::Test {};
using Families=::testing::Types<Quad,Triangle>;
TYPED_TEST_SUITE(QtMappedCuda,Families);

TYPED_TEST(QtMappedCuda,CompleteNativeStiffnessAndStaleToken) {
  using Family=TypeParam;
  Rig<Family> rig;
  ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(rig.Begin(token,assembly));
  const auto accepted=Values(rig.Accepted());
  const auto accepted_state=rig.AcceptedState();
  fe::NodalCinAssemblyView cin;
  ASSERT_EQ(rig.owner.BorrowCinAssembly(token,&cin).status,fe::NodalStatus::Ok);
  std::vector<double> actual(cin.node_count),rotation(cin.node_count);
  std::vector<double> expected(cin.node_count),expected_rotation(cin.node_count);
  ASSERT_EQ(cudaMemcpyAsync(actual.data(),cin.translational_stiffness,actual.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(rotation.data(),cin.rotational_stiffness,rotation.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(cin.stream),cudaSuccess);
  for (std::size_t parent=0;parent<rig.config.element_count;++parent) {
    fe::shell_nodal_stiffness::Packet<Family::Slots> stiffness;
    ASSERT_TRUE(Family::Stiffness(rig.fixture,parent,stiffness));
    std::size_t nodes[Family::Slots];
    for (unsigned slot=0;slot<Family::Slots;++slot) {
      nodes[slot]=rig.fixture.mechanics.domain.Find(Family::Reference(rig.fixture,parent).input.node_ids[slot]);
    }
    ASSERT_TRUE(fe::shell_nodal_stiffness::Add(nodes,stiffness,expected.data(),expected_rotation.data(),expected.size()));
  }
  EXPECT_EQ(actual,expected);
  EXPECT_EQ(rotation,expected_rotation);
  EXPECT_GT(*std::max_element(rotation.begin(),rotation.end()),0);
  EXPECT_EQ(rotation[rig.fixture.mechanics.ordinary],0);
  // The complete ledger includes ordinary solid J0 and PART dependent zero M/J.
  EXPECT_EQ(rig.fixture.mechanics.j[rig.fixture.mechanics.ordinary],0);
  EXPECT_EQ(rig.fixture.mechanics.m[rig.fixture.mechanics.zero_mass],0);
  EXPECT_NE(rig.batch.AssembleAccepted(rig.owner,assembly).status,Family::Success);
  rig.owner.Discard();
  rig.batch.DiscardTrial();
  fe::NodalTrialToken next;
  ASSERT_EQ(rig.owner.BeginTrial(&next,&assembly).status,fe::NodalStatus::Ok);
  EXPECT_NE(rig.batch.AssembleMappedAccepted(rig.owner,token,assembly).status,Family::Success);
  ASSERT_TRUE(rig.Begin(next,assembly));
  // Binding survives discard, but epoch zero has no completed force cache.
  // Both families must scatter the same nonzero virgin coefficients on retry.
  ASSERT_EQ(rig.owner.BorrowCinAssembly(next,&cin).status,fe::NodalStatus::Ok);
  ASSERT_EQ(cudaMemcpyAsync(actual.data(),cin.translational_stiffness,actual.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(rotation.data(),cin.rotational_stiffness,rotation.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(cin.stream),cudaSuccess);
  EXPECT_EQ(actual,expected);
  EXPECT_EQ(rotation,expected_rotation);
  EXPECT_EQ(Values(rig.Accepted()),accepted);
  EXPECT_EQ(rig.AcceptedState(),accepted_state);
}

TYPED_TEST(QtMappedCuda,CinDependentZerosLateGeometryFailureAndTypedRetry) {
  using Family=TypeParam;
  Rig<Family> rig;
  ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView view;
  ASSERT_TRUE(rig.Prepare(token,assembly,view));
  const auto accepted=Values(rig.Accepted());
  const auto accepted_state=rig.AcceptedState();
  const auto n=rig.config.owner.node_count,r=rig.fixture.mechanics.ranges.size();
  std::vector<double> raw(2*n+2*r+1);
  fe::NodalPreparedView copied;
  ASSERT_EQ(rig.owner.CopyPreparedCin(token,{raw.data(),raw.data()+n,raw.data()+2*n,
      raw.data()+2*n+r,raw.data()+2*n+2*r,n,r},&copied).status,fe::NodalStatus::Ok);
  const auto dependent=rig.fixture.mechanics.cin_model.rows().data[0].secondary_domain_node;
  EXPECT_GT(rig.fixture.mechanics.m[dependent],0);
  EXPECT_EQ(raw[dependent],0);
  EXPECT_EQ(raw[n+dependent],0);
  const auto& reference=Family::Reference(rig.fixture,rig.config.element_count-1);
  const auto last=rig.fixture.mechanics.domain.Find(reference.input.node_ids[Family::Slots-1]);
  const double invalid=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(view.kinematics.position_xyz)+3*last,
      &invalid,sizeof(invalid),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  typename Family::Diagnostics output;
  const auto untouched=Bytes(output);
  EXPECT_NE(rig.batch.EvaluateCandidate(rig.owner,token,view,&output).status,Family::Success);
  EXPECT_EQ(Bytes(output),untouched);
  EXPECT_EQ(Values(rig.Accepted()),accepted);
  EXPECT_EQ(rig.AcceptedState(),accepted_state);
  rig.owner.Discard();
  rig.batch.DiscardTrial();
  ASSERT_TRUE(rig.Prepare(token,assembly,view));
  ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,view,&output).status,Family::Success)<<output.epoch;
  std::vector<typename Family::Result> proposed(rig.config.element_count);
  const auto previous=Bytes(proposed.back());
  EXPECT_NE(rig.batch.CopyPreparedResults(output,proposed.data(),proposed.size()-1).status,Family::Success);
  EXPECT_EQ(Bytes(proposed.back()),previous);
  ASSERT_EQ(rig.batch.CopyPreparedResults(output,proposed.data(),proposed.size()).status,Family::Success);
  EXPECT_EQ(proposed.back().proposed_history.stamp().sample_index,1u);
  EXPECT_NE(Values(proposed),accepted);
  std::vector<fe::ShellBatchLayeredSection> sections(proposed.size());
  ASSERT_EQ(rig.batch.CopyPreparedLayeredSectionHistory(output,sections.data(),sections.size()).status,Family::Success);
  for (std::size_t row=0;row<sections.size();++row) {
    EXPECT_EQ(sections[row].law(),Law(rig.fixture.Catalog(),Family::Family,row));
    if constexpr (Family::Slots==3) {
      EXPECT_NE(sections[row].one_point(),nullptr);
      EXPECT_EQ(sections[row].plastic(),nullptr);
    } else EXPECT_NE(sections[row].plastic(),nullptr);
  }
  std::vector<fe::ShellBatchFailureState> failure(sections.size());
  const auto failure_before=PayloadBytes(failure);
  const auto failure_report=rig.batch.CopyPreparedFailureHistory(output,failure.data(),failure.size());
  if constexpr (Family::Slots==3) {
    EXPECT_NE(failure_report.status,Family::Success);
    EXPECT_EQ(PayloadBytes(failure),failure_before);
  } else ASSERT_EQ(failure_report.status,Family::Success);
  StateBits proposed_state;
  for (std::size_t row=0;row<sections.size();++row) {
    Add(proposed_state,sections[row]);
    if constexpr (Family::Slots!=3) Add(proposed_state,failure[row]);
  }
  EXPECT_NE(proposed_state,accepted_state); // Actual material fields advanced.
  EXPECT_EQ(Values(rig.Accepted()),accepted);
  EXPECT_EQ(rig.AcceptedState(),accepted_state);
  // This gate stops before common mapped publication, owned by the coordinator.
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
}

TYPED_TEST(QtMappedCuda,WrongRosterAndForeignCoefficientRejectBeforeAllocation) {
  using Family=TypeParam;
  Fixture fixture;
  fe::FENodalState owner;
  ASSERT_EQ(InitializeOwner(fixture,owner).status,fe::NodalStatus::Ok);
  auto config=Config<Family>(fixture,owner.accepted());
  typename Family::Batch batch;
  auto witnesses=fixture.mechanics.witnesses;
  ++witnesses.back().source_element_id;
  auto source=fixture.Witnesses();
  source.witnesses=witnesses.data();
  EXPECT_NE(batch.InitializeMapped(config,fixture.Physical(),owner,source).status,Family::Success);
  EXPECT_EQ(batch.allocations().device_allocations,0u);
  fe::ElementMassContributions mass;
  const fe::ElementMassSource rows[]{{18000,777,fixture.mechanics.domain.Find(777),0},
      {18001,55,fixture.mechanics.domain.Find(55),.003}};
  ASSERT_TRUE(mass.Initialize(fixture.mechanics.domain,{1,1000,rows,2}));
  fe::NodalCoefficientLedger foreign;
  ASSERT_TRUE(foreign.InitializeWithSolids({{&fixture.mechanics.shells,&fixture.mechanics.springs},&mass,&fixture.mechanics.solids}));
  fe::ShellPhysicalBinding changed;
  ASSERT_TRUE(changed.Initialize({fixture.Physical().shells(),&fixture.Catalog(),&fixture.failure,nullptr},foreign));
  EXPECT_NE(batch.InitializeMapped(config,changed,owner,fixture.Witnesses()).status,Family::Success);
  EXPECT_EQ(batch.allocations().device_allocations,0u);
  ASSERT_EQ(batch.InitializeMapped(config,fixture.Physical(),owner,fixture.Witnesses()).status,Family::Success);
}
} // namespace qt_mapped_test
