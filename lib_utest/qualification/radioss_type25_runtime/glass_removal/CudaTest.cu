// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
#include <gtest/gtest.h>
#include <sstream>
#include <iomanip>
namespace glass_removal_test {
namespace {
// Loads, geometry and timestep are declared synthetic. The glass material,
// section and TAB1 policy retain the authenticated original part2000452 values.
void ReachRemoval(Rig& rig,Attempt& a,State& before,std::vector<double>& force) {
  rig.Initialize();rig.Step();rig.Step();
  for(unsigned step=0;step<48;++step){
    before=rig.Read();rig.Begin(a,true);rig.Assemble(a);force=rig.Force(a);rig.Prepare(a);
    const auto candidate=rig.PreparedFailure(a);
    if(!candidate[1].active){rig.Seal(a);return;}
    rig.Seal(a);Check(rig.Commit(a));
  }
  rig.Discard();throw std::runtime_error("Declared bounded glass bending load did not reach TAB1 removal");
}
bool GlassMain(const Rig& rig,int id) {
  for(const auto& main:rig.self_source.mains)if(main.global_id==id){
    bool glass=true;for(auto node:main.nodes)glass=glass&&node>=4&&node<8;return glass;
  }
  return false;
}
std::string WallWitness(const State& state) {
  std::ostringstream text;text<<std::setprecision(17)<<"epoch="<<state.stamp.epoch;
  for(std::size_t i=0;i<state.contacts[1].size();++i){
    const auto& row=state.contacts[1][i].row;const auto& h=row.history.normal;
    text<<"\nrow="<<i<<" irtlm=";
    for(auto value:row.irtlm)text<<value<<",";
    text<<" K="<<h.previous_stiffness<<","<<h.staged_stiffness
        <<" P="<<h.previous_penetration<<","<<h.staged_penetration
        <<" offset="<<row.penetration_offset;
  }
  return text.str();
}
void RemovalChecks(Rig& rig,const Attempt& a,const State& before) {
  const auto candidate=rig.PreparedFailure(a);
  EXPECT_TRUE(before.failure[0].active);
  EXPECT_TRUE(before.failure[1].active);
  EXPECT_EQ(before.failure[1].policy(),fe::ShellFailurePolicy::Tab1AnyPoint);
  EXPECT_EQ(FailedPoints(before.failure[1]),0u);
  EXPECT_TRUE(candidate[0].active);
  EXPECT_FALSE(candidate[1].active);
  EXPECT_GT(FailedPoints(candidate[1]),0u);
  EXPECT_LT(FailedPoints(candidate[1]),3u);
  // Release assertions below require a genuinely retained removed main and
  // an independently active fixed-wall ledger, not empty initial histories.
  bool retained_glass=false,active_wall_history=false;
  for(const auto& row:before.contacts[0])retained_glass=retained_glass||
      (GlassMain(rig,row.row.irtlm[0])&&(row.row.history.normal.previous_stiffness>0||row.row.history.normal.staged_stiffness>0));
  for(const auto& row:before.contacts[1])active_wall_history=active_wall_history||
      (row.row.irtlm[0]>0&&(row.row.history.normal.previous_stiffness>0||row.row.history.normal.staged_stiffness>0));
  EXPECT_TRUE(retained_glass);
  EXPECT_TRUE(active_wall_history)<<WallWitness(before);
  const auto self=rig.self.last_diagnostics(),wall=rig.wall.last_diagnostics();
  EXPECT_TRUE(rig.self_response_nonzero);
  EXPECT_TRUE(rig.wall_response_nonzero);
  EXPECT_GT(self.active_forces,0u);
  EXPECT_GT(wall.active_forces,0u);
  EXPECT_TRUE(self.activity_changed);
  EXPECT_EQ(self.activity_removed_mains,2u);
  EXPECT_EQ(self.activity_orphan_secondaries,4u);
  EXPECT_FALSE(wall.activity_changed);
  EXPECT_EQ(wall.activity_removed_mains,0u);
  EXPECT_EQ(wall.activity_orphan_secondaries,0u);
  // Candidate source slabs/history cannot become visible before common commit.
  Same(before,rig.Read());
}
void ContinueAfterRemoval(Rig& rig,const State& before) {
  const auto accepted=rig.Read();
  EXPECT_FALSE(accepted.failure[1].active);
  EXPECT_EQ(accepted.mass,before.mass);
  EXPECT_EQ(accepted.inertia,before.inertia);
  const auto generation=accepted.publication[0].selectors.activity_generation;
  EXPECT_EQ(generation,before.publication[0].selectors.activity_generation+1);
  EXPECT_EQ(accepted.publication[0].selectors.reference_activity_generation,before.publication[0].selectors.activity_generation);
  EXPECT_EQ(accepted.publication[1].selectors.activity_generation,before.publication[1].selectors.activity_generation);
  for(unsigned step=0;step<3;++step){
    Attempt a;rig.Begin(a);rig.Assemble(a);
    if(!step)EXPECT_TRUE(rig.self.last_diagnostics().reference_rebuilt);
    rig.Prepare(a);rig.Seal(a);
    EXPECT_FALSE(rig.self.last_diagnostics().activity_changed);
    EXPECT_FALSE(rig.wall.last_diagnostics().activity_changed);
    Check(rig.Commit(a));
    const auto state=rig.Read();
    EXPECT_EQ(state.publication[0].selectors.activity_generation,generation);
    EXPECT_EQ(state.publication[0].selectors.reference_activity_generation,generation);
    EXPECT_EQ(state.publication[1].selectors.activity_generation,before.publication[1].selectors.activity_generation);
    EXPECT_FALSE(state.failure[1].active);
    EXPECT_TRUE(state.failure[0].active);
    EXPECT_TRUE(state.triangle_failure.active);
    for(unsigned row=0;row<4;++row)for(int value:state.contacts[0][row].row.irtlm)EXPECT_EQ(value,0);
    for(const auto& row:state.contacts[0])EXPECT_FALSE(GlassMain(rig,row.row.irtlm[0]));
    for(unsigned i=0;i<12;++i)EXPECT_EQ(state.x[i],before.x[i]);
  }
}
}
TEST(NativeGlassRemovalCuda, GenuineAnyPointRemovalCommitsSelfAndWallTogetherThenContinues) {
  Rig rig;Attempt a;State before;std::vector<double> force;
  ASSERT_NO_THROW(ReachRemoval(rig,a,before,force));
  RemovalChecks(rig,a,before);
  ASSERT_NO_THROW(Check(rig.Commit(a)));
  ASSERT_NO_THROW(ContinueAfterRemoval(rig,before));
}
TEST(NativeGlassRemovalCuda, LateCommonRejectionPreservesCompleteStateAndRetriesExactly) {
  Rig rig;Attempt first;State before;std::vector<double> expected_force;
  ASSERT_NO_THROW(ReachRemoval(rig,first,before,expected_force));
  RemovalChecks(rig,first,before);
  const auto expected=rig.PreparedFailure(first);
  const auto diagnostics=rig.self.last_diagnostics();
  EXPECT_NE(rig.Commit(first,false).status,fe::ShellPublicationStatus::Success);
  rig.Discard();Same(before,rig.Read());
  Attempt retry;
  ASSERT_NO_THROW(rig.Begin(retry,true));
  ASSERT_NO_THROW(rig.Assemble(retry));
  failure_force_test::Exact(expected_force,rig.Force(retry));
  ASSERT_NO_THROW(rig.Prepare(retry));
  const auto actual=rig.PreparedFailure(retry);
  for(unsigned i=0;i<2;++i)failure_force_test::Exact(resident_tab1_test::Values(expected[i]),resident_tab1_test::Values(actual[i]));
  ASSERT_NO_THROW(rig.Seal(retry));
  RemovalChecks(rig,retry,before);
  const auto repeated=rig.self.last_diagnostics();
  EXPECT_EQ(repeated.activity_affected_events,diagnostics.activity_affected_events);
  EXPECT_EQ(repeated.activity_removed_events,diagnostics.activity_removed_events);
  EXPECT_EQ(repeated.activity_removed_mains,diagnostics.activity_removed_mains);
  EXPECT_EQ(repeated.activity_orphan_secondaries,diagnostics.activity_orphan_secondaries);
  ASSERT_NO_THROW(Check(rig.Commit(retry)));
  ASSERT_NO_THROW(ContinueAfterRemoval(rig,before));
}
} // namespace glass_removal_test
