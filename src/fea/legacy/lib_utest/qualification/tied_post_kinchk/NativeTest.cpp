#include "Fixture.h"
#include "NativeOracle.h"
namespace kinchk_test {
void Compare(const Fixture& f, const NativeControls& controls = {}) {
  tied::PostKinChkResult out;
  ASSERT_TRUE(tied::PostKinChk(f.Input(),&out));
  const auto native = Native(f.Input(),controls);
  EXPECT_EQ(native.statistics[1],0);
  EXPECT_EQ(native.statistics[4],1);
  bool possible = false;
  for (std::size_t i = 0; i < f.slaves.size(); ++i) {
    const auto& row = out.slaves().data[i];
    const auto n = f.slaves.size();
    Same(row.before,f.slaves[i]);
    EXPECT_EQ(row.kinet,native.kinet[i]);
    EXPECT_EQ(row.before.kinematics.conditions,native.five[i]);
    EXPECT_EQ(row.before.kinematics.translation,native.five[n+i]);
    EXPECT_EQ(row.before.kinematics.rotation,native.five[2*n+i]);
    EXPECT_EQ(row.before.kinematics.duplicate_conditions,native.five[3*n+i]);
    EXPECT_EQ(row.before.kinematics.incompatible_conditions,native.five[4*n+i]);
    possible |= row.repeated_condition || row.mixed_incompatible_conditions;
  }
  EXPECT_EQ(native.statistics[3],!possible);
  EXPECT_EQ(native.decode,f.decode);
}
TEST(PostKinChkNative, FullRoutinePreservesCinPenaltyAndReportsObservedPossibleConflicts) {
  Fixture f;
  Compare(f);
  f.slaves.back().kinematics = {9,37,47,8,8};
  Compare(f);
  f.slaves.back().kinematics = {8,7,7,8,0};
  Compare(f);
  f.slaves.back().kinematics = {9,7,7,0,1};
  Compare(f);
}
TEST(PostKinChkNative, MasterOnlyHierarchyWarningsDoNotChangeObservedSlaveOrDecode) {
  Fixture f;
  f.slaves.back().kinematics = {8,7,7,8,0};
  f.slaves.push_back({31,1,{8,7,7,0,0}});
  NativeControls c;
  c.bodies = {2,0,1,0,0, 3,1,1,0,0};
  c.members = {3,2};
  const auto native = Native(f.Input(),c);
  EXPECT_EQ(native.statistics[0],2);
  EXPECT_EQ(native.statistics[2],555);
  Compare(f,c);
  c.bodies[3] = 1;
  EXPECT_EQ(Native(f.Input(),c).statistics[0],0);
  Compare(f,c);
}
TEST(PostKinChkNative, ExcludedWallRbeAndCyclicControlsReachRealNativeMutationOrErrors) {
  Fixture f;
  NativeControls c;
  f.slaves[0].kinematics.conditions = 6;
  c.wall = 1;
  const auto wall = Native(f.Input(),c);
  EXPECT_EQ(wall.five[0],2);
  EXPECT_EQ(wall.statistics[2],446);
  f.slaves[0].kinematics.conditions = 2;
  c = {};
  c.rbe2 = 1;
  const auto rbe2 = Native(f.Input(),c);
  EXPECT_EQ(rbe2.statistics[1],1);
  EXPECT_EQ(rbe2.statistics[2],1036);
  EXPECT_EQ(rbe2.statistics[4],0);
  c = {};
  c.rbe3 = 1;
  EXPECT_EQ(Native(f.Input(),c).statistics[2],1035);
  c = {};
  c.cyclic = 1;
  f.slaves[0].kinematics.conditions = 1024;
  EXPECT_EQ(Native(f.Input(),c).statistics[2],1757);
}
TEST(PostKinChkNative, LateBodyPacketFailureLeavesEveryOutputUntouched) {
  std::array<std::int32_t,2> ids{101,202};
  std::array<std::int32_t,10> five{}, output;
  std::array<std::int32_t,8192> decode{}, output_decode;
  std::array<std::int32_t,2> kinet;
  std::array<std::int32_t,5> stats;
  const std::array<std::int32_t,10> bodies{1,0,1,0,0, 3,1,1,0,0};
  const std::array<std::int32_t,2> members{2,1};
  output.fill(99);
  output_decode.fill(98);
  kinet.fill(97);
  stats.fill(96);
  std::int32_t status = -1;
  native_post_kinchk(2,ids.data(),five.data(),decode.data(),2,bodies.data(),2,members.data(),
    0,0,0,0,0,output.data(),output_decode.data(),kinet.data(),stats.data(),&status);
  EXPECT_EQ(status,1);
  for (const auto v : output) EXPECT_EQ(v,99);
  for (const auto v : output_decode) EXPECT_EQ(v,98);
  for (const auto v : kinet) EXPECT_EQ(v,97);
  for (const auto v : stats) EXPECT_EQ(v,96);
}
}
