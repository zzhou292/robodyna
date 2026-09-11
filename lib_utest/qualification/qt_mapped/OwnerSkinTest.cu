// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"

namespace qt_mapped_test {
template<class Family> class QtMappedSkinCuda : public ::testing::Test {};
using SkinFamilies=::testing::Types<Quad,Triangle>;
TYPED_TEST_SUITE(QtMappedSkinCuda,SkinFamilies);

TYPED_TEST(QtMappedSkinCuda,ExplicitPartSkinHasZeroPointsForceAndStiffnessAtBothEndpoints) {
  using Family=TypeParam;
  Rig<Family> rig;
  ASSERT_TRUE(rig.Initialize(true));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(rig.Begin(token,assembly));
  auto results=rig.Accepted();
  std::vector<fe::ShellBatchLayeredSection> sections(results.size());
  typename Family::Diagnostics diagnostics;
  ASSERT_EQ(rig.batch.CopyAcceptedLayeredSectionHistory(rig.owner.accepted(),sections.data(),sections.size(),&diagnostics).status,Family::Success);
  EXPECT_EQ(sections[0].law(),fe::ShellSectionLaw::RigidSkin);
  EXPECT_EQ(sections[0].plastic(),nullptr);
  EXPECT_EQ(sections[0].elastic(),nullptr);
  EXPECT_EQ(sections[0].one_point(),nullptr);
  EXPECT_EQ(results[0].diagnostics.translational_stiffness,0);
  EXPECT_EQ(results[0].diagnostics.rotational_stiffness,0);
  EXPECT_EQ(results[0].kinematics.area,0); // No fabricated current force geometry.
  fe::NodalCinAssemblyView cin;
  ASSERT_EQ(rig.owner.BorrowCinAssembly(token,&cin).status,fe::NodalStatus::Ok);
  std::vector<double> actual(cin.node_count),rotation(cin.node_count);
  std::vector<double> expected(cin.node_count),expected_rotation(cin.node_count);
  ASSERT_EQ(cudaMemcpyAsync(actual.data(),cin.translational_stiffness,actual.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(rotation.data(),cin.rotational_stiffness,rotation.size()*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(cin.stream),cudaSuccess);
  for (std::size_t row=1;row<results.size();++row) {
    fe::shell_nodal_stiffness::Packet<Family::Slots> stiffness;
    ASSERT_TRUE(Family::Stiffness(rig.fixture,row,stiffness));
    std::size_t nodes[Family::Slots];
    for (unsigned slot=0;slot<Family::Slots;++slot) {
      nodes[slot]=rig.fixture.mechanics.domain.Find(Family::Reference(rig.fixture,row).input.node_ids[slot]);
    }
    ASSERT_TRUE(fe::shell_nodal_stiffness::Add(nodes,stiffness,expected.data(),expected_rotation.data(),expected.size()));
  }
  EXPECT_EQ(actual,expected);
  EXPECT_EQ(rotation,expected_rotation);
  fe::NodalPreparedView view;
  ASSERT_TRUE(PrepareOwner(rig.fixture,rig.owner,token,assembly,view));
  ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,view,&diagnostics).status,Family::Success);
  ASSERT_EQ(rig.batch.CopyPreparedResults(diagnostics,results.data(),results.size()).status,Family::Success);
  ASSERT_EQ(rig.batch.CopyPreparedLayeredSectionHistory(diagnostics,sections.data(),sections.size()).status,Family::Success);
  EXPECT_EQ(sections[0].law(),fe::ShellSectionLaw::RigidSkin);
  EXPECT_EQ(sections[0].plastic(),nullptr);
  EXPECT_EQ(results[0].proposed_history.stamp().sample_index,1u);
  EXPECT_EQ(results[0].proposed_history.data().active,1);
  EXPECT_EQ(results[0].diagnostics.translational_stiffness,0);
  EXPECT_EQ(results[0].diagnostics.rotational_stiffness,0);
  for (auto f:results[0].internal_force) { EXPECT_EQ(f.x,0); EXPECT_EQ(f.y,0); EXPECT_EQ(f.z,0); }
  for (auto f:results[0].internal_couple) { EXPECT_EQ(f.x,0); EXPECT_EQ(f.y,0); EXPECT_EQ(f.z,0); }
  for (double work:results[0].proposed_history.data().internal_work) EXPECT_EQ(work,0);
  if constexpr (Family::Slots==3) EXPECT_EQ(diagnostics.minimum_native_dt,0);
  std::vector<std::uint8_t> activity(results.size());
  ASSERT_EQ(rig.batch.CopyAcceptedParentActivity(rig.owner.accepted(),activity.data(),activity.size(),&diagnostics).status,Family::Success);
  EXPECT_EQ(activity[0],1u);
}

TYPED_TEST(QtMappedSkinCuda,RoleBackingAliasAndForeignPartOwnerRejectWithRetry) {
  using Family=TypeParam;
  Rig<Family> rig;
  ASSERT_TRUE(rig.Initialize(true));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(rig.Begin(token,assembly));
  const auto accepted=Values(rig.Accepted());
  const auto* execution=rig.fixture.Physical().execution();
  ASSERT_NE(execution,nullptr);
  const auto roots=execution->rigid()->members();
  ASSERT_GT(roots.size()*sizeof(roots[0]),sizeof(fe::ShellBatchLayeredSection)*rig.config.element_count);
  auto* alias=reinterpret_cast<fe::ShellBatchLayeredSection*>(const_cast<std::remove_const_t<std::remove_reference_t<decltype(roots[0])>>*>(roots.data()));
  const auto before=Bytes(roots[0]);
  typename Family::Diagnostics diagnostics;
  const auto untouched=Bytes(diagnostics);
  EXPECT_NE(rig.batch.CopyAcceptedLayeredSectionHistory(rig.owner.accepted(),alias,rig.config.element_count,&diagnostics).status,Family::Success);
  EXPECT_EQ(Bytes(roots[0]),before);
  EXPECT_EQ(Bytes(diagnostics),untouched);
  EXPECT_EQ(Values(rig.Accepted()),accepted);
  Fixture foreign;
  fe::FENodalState other;
  ASSERT_EQ(InitializeOwner(foreign,other).status,fe::NodalStatus::Ok);
  typename Family::Batch rejected;
  auto config=Config<Family>(rig.fixture,other.accepted());
  EXPECT_NE(rejected.InitializeMapped(config,rig.fixture.Physical(),other,rig.fixture.Witnesses()).status,Family::Success);
  EXPECT_EQ(rejected.allocations().device_allocations,0u);
  rig.owner.Discard();
  rig.batch.DiscardTrial();
  ASSERT_EQ(rejected.InitializeMapped(rig.config,rig.fixture.Physical(),rig.owner,rig.fixture.Witnesses()).status,Family::Success);
}
} // namespace qt_mapped_test
