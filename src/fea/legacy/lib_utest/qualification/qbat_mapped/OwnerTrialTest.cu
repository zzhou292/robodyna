// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"

namespace qbat_mapped_test {
TEST(QbatMappedCuda,NativeStiffnessDestinationsStaleTokenAndImmutableOutputAlias) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(rig.Begin(token,assembly));
  const auto virgin=rig.Accepted();
  fe::NodalCinAssemblyView cin;
  ASSERT_EQ(rig.owner.BorrowCinAssembly(token,&cin).status,fe::NodalStatus::Ok);
  std::vector<double> actual(cin.node_count),rotational(cin.node_count),expected(cin.node_count),expected_rotation(cin.node_count);
  ASSERT_EQ(cudaMemcpyAsync(actual.data(),cin.translational_stiffness,actual.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(rotational.data(),cin.rotational_stiffness,rotational.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(cin.stream),cudaSuccess);
  batch::Element element;
  element.reference=rig.fixture.physical.shells()->qbat_reference(0);
  ASSERT_TRUE(rig.fixture.catalog.Parameters(fe::ShellBindingFamily::Qbat,0,&element.material));
  for (unsigned slot=0;slot<4;++slot) element.nodes[slot]=rig.fixture.mechanics.domain.Find(10+slot);
  mapped::NodalStiffness stiffness;
  ASSERT_TRUE(mapped::AcceptedStiffness(element,virgin,stiffness));
  ASSERT_TRUE(mapped::AddStiffness(element.nodes,stiffness,expected.data(),expected_rotation.data(),expected.size()));
  EXPECT_EQ(actual,expected);
  EXPECT_EQ(rotational,expected_rotation);
  auto* alias=reinterpret_cast<qb::BatchDiagnostics*>(const_cast<fe::NodalCoefficientNode*>(rig.fixture.ledger.nodes().data()));
  const auto source_before=Bytes(rig.fixture.ledger.nodes()[0]);
  EXPECT_NE(rig.batch.CopyAcceptedDiagnostics(rig.owner.accepted(),alias).status,qb::BatchStatus::Success);
  EXPECT_EQ(source_before,Bytes(rig.fixture.ledger.nodes()[0]));
  rig.owner.Discard();rig.batch.DiscardTrial();
  fe::NodalTrialToken next;
  ASSERT_EQ(rig.owner.BeginTrial(&next,&assembly).status,fe::NodalStatus::Ok);
  EXPECT_NE(rig.batch.AssembleMappedAccepted(rig.owner,token,assembly).status,qb::BatchStatus::Success);
  ASSERT_TRUE(rig.Begin(next,assembly));
  EXPECT_EQ(qbat_resident_test::ResultValues(rig.Accepted()),qbat_resident_test::ResultValues(virgin));
}
TEST(QbatMappedCuda,CinTransferPreparedGeometryFaultAndExactHistoryRetry) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView view;
  ASSERT_TRUE(rig.Prepare(token,assembly,view));
  const auto accepted=rig.Accepted();
  const auto n=rig.config.owner.node_count,r=rig.fixture.mechanics.ranges.size();
  std::vector<double> raw(2*n+2*r+1);
  fe::NodalPreparedView copied;
  ASSERT_EQ(rig.owner.CopyPreparedCin(token,{raw.data(),raw.data()+n,raw.data()+2*n,
      raw.data()+2*n+r,raw.data()+2*n+2*r,n,r},&copied).status,fe::NodalStatus::Ok);
  const auto dependent=rig.fixture.mechanics.cin_model.rows().data[0].secondary_domain_node;
  EXPECT_GT(rig.fixture.mechanics.m[dependent],0);
  EXPECT_EQ(raw[dependent],0);
  EXPECT_EQ(raw[n+dependent],0);
  const auto last=rig.fixture.mechanics.domain.Find(13);
  const double invalid=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(view.kinematics.position_xyz)+3*last,
      &invalid,sizeof(invalid),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  qb::BatchDiagnostics output;
  const auto untouched=Bytes(output);
  EXPECT_NE(rig.batch.EvaluateCandidate(rig.owner,token,view,&output).status,qb::BatchStatus::Success);
  EXPECT_EQ(Bytes(output),untouched);
  EXPECT_EQ(qbat_resident_test::ResultValues(rig.Accepted()),qbat_resident_test::ResultValues(accepted));
  rig.owner.Discard();rig.batch.DiscardTrial();
  ASSERT_TRUE(rig.Prepare(token,assembly,view));
  ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,view,&output).status,qb::BatchStatus::Success);
  qb::BatchResult proposed;
  const auto proposed_before=Bytes(proposed);
  EXPECT_NE(rig.batch.CopyPreparedResults(output,&proposed,0).status,qb::BatchStatus::Success);
  EXPECT_EQ(Bytes(proposed),proposed_before);
  ASSERT_EQ(rig.batch.CopyPreparedResults(output,&proposed,1).status,qb::BatchStatus::Success);
  EXPECT_EQ(proposed.stamp.sample_index,1u);
  EXPECT_NE(qbat_resident_test::ResultValues(proposed),qbat_resident_test::ResultValues(accepted));
  EXPECT_EQ(qbat_resident_test::ResultValues(rig.Accepted()),qbat_resident_test::ResultValues(accepted));
  EXPECT_EQ(rig.batch.allocations().device_allocations,1u);
  // Common mapped publication is root's next integration gate. This function
  // deliberately leaves a prepared, uncommitted family candidate.
}
} // namespace qbat_mapped_test
