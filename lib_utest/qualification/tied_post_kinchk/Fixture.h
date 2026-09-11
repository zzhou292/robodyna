#pragma once
#include "lib_src/constraints/tied_shell/TiedPostKinChk.h"
#include <array>
#include <vector>
#include <gtest/gtest.h>
namespace kinchk_test {
namespace tied = tl::constraints::tied_shell;
struct Fixture {
  std::vector<tied::KinChkSlave> slaves{{91,0,{2,7,7,0,0}}, {19,1,{8,7,7,0,0}}};
  std::array<std::int32_t,8192> decode{};
  Fixture() { for (std::size_t i = 0; i < decode.size(); ++i) decode[i] = (i&2) != 0; }
  tied::KinChkInput Input() const {
    return {tied::KinChkProfile::NoWallRbeOrCyclic,tied::ClassificationPhase::InterfaceTaggedBeforeKinChk,
      73,991,{slaves.data(),slaves.size()},{decode.data(),decode.size()}};
  }
};
inline void Same(const tied::KinChkSlave& a, const tied::KinChkSlave& b) {
  EXPECT_EQ(a.source_id,b.source_id);
  EXPECT_EQ(a.irupt,b.irupt);
  EXPECT_EQ(a.kinematics.conditions,b.kinematics.conditions);
  EXPECT_EQ(a.kinematics.translation,b.kinematics.translation);
  EXPECT_EQ(a.kinematics.rotation,b.kinematics.rotation);
  EXPECT_EQ(a.kinematics.duplicate_conditions,b.kinematics.duplicate_conditions);
  EXPECT_EQ(a.kinematics.incompatible_conditions,b.kinematics.incompatible_conditions);
}
}
