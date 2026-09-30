// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"

namespace solid_validation_test {
inline void CompareAll(HostRig& rig, const fe::NodalPreparedView* view = nullptr) {
  ValidateAll(rig.state, 1, 0, 0);
  Reset(rig.state);
  const auto success = MeasureAll(rig.state, true, 0, 1, view);
  const auto expected = rig.state.control;
  Reset(rig.state);
  EXPECT_EQ(MeasureAll(rig.state, false, 0, 1, view), success);
  SameControl(rig.state.control, expected);
}
TEST(SolidCandidateMeasure, WholeFiveFamilyFoldPreservesSignedSlotAndAcceptedRhsOrder) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto count = rig.config.owner.node_count;
  std::vector<double> x(3 * count), base(x), velocity(x), base_velocity(x);
  for (std::size_t n = 0; n < x.size(); ++n) {
    base[n] = .03125 * double(n);
    x[n] = base[n] + (n % 2 ? -.125 : .125);
    velocity[n] = n % 2 ? -3 : 3;
    base_velocity[n] = n % 3 ? -0.0 : 1;
  }
  auto forces = [](auto& f) {
    for (unsigned n = 0; n < 8; ++n) {
      const double value[] {1e16, 1, -1e16, -0.0, DBL_MIN, -DBL_MIN, .5, -.5};
      f.slab[0][0].cache.rhs_force_n[n] = {value[n], -value[n], value[n]};
      f.slab[1][0].cache.rhs_force_n[n] = {11, 13, 17};
    }
  };
  forces(rig.state.solid18);
  forces(rig.state.solid24);
  forces(rig.state.solid18_law44);
  forces(rig.state.solid18_law90);
  fe::NodalPreparedView view;
  view.base_kinematics.position_xyz = base.data();
  view.base_kinematics.velocity_xyz = base_velocity.data();
  view.kinematics.position_xyz = x.data();
  view.kinematics.velocity_xyz = velocity.data();
  view.kick_dt = .125;
  CompareAll(rig, &view);
  EXPECT_EQ(rig.state.control.status, s::BatchStatus::Success);
}
TEST(SolidCandidateMeasure, EarlierCumulativeOverflowBeatsLaterInvalidPredicateOrStatus) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  Rows<d::Traits18> rows(rig.state, 3);
  rows.trial[0].cache.diagnostics.internal_work_increment_j = DBL_MAX;
  rows.trial[1].cache.diagnostics.internal_work_increment_j = DBL_MAX;
  for (int status : {0, 9}) {
    rows.status[2] = status;
    rows.trial[2].cache.rhs_force_n[7].z = NAN;
    CompareAll(rig);
    EXPECT_EQ(rig.state.control.status, s::BatchStatus::NonfiniteResult);
    EXPECT_EQ(rig.state.control.parent, 1u);
    EXPECT_EQ(rig.state.control.family, s::Family::Solid18);
    EXPECT_EQ(rig.state.control.element_status, 0);
  }
  rows.trial[1].cache.diagnostics.internal_work_increment_j = -DBL_MAX;
  rows.status[2] = 9;
  CompareAll(rig);
  EXPECT_EQ(rig.state.control.status, s::BatchStatus::ElementFailure);
  EXPECT_EQ(rig.state.control.parent, 2u);
}
TEST(SolidCandidateMeasure, CrossFamilyOverflowAndLateFailureHaveLiteralPrefixDiagnostics) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  rig.state.solid18.slab[1][0].cache.diagnostics.plastic_work_increment_j = DBL_MAX;
  rig.state.solid18_law44.slab[1][0].cache.diagnostics.plastic_work_increment_j = DBL_MAX;
  rig.state.solid18_law90.slab[1][0].cache.rhs_force_n[7].z = NAN;
  CompareAll(rig);
  EXPECT_EQ(rig.state.control.family, s::Family::Solid18Law44);
  EXPECT_EQ(rig.state.control.diagnostics.parent_count[4], 0u);
  rig.state.solid18_law44.slab[1][0].cache.diagnostics.plastic_work_increment_j = 0;
  CompareAll(rig);
  EXPECT_EQ(rig.state.control.family, s::Family::Solid18Law90);
  EXPECT_EQ(rig.state.control.diagnostics.parent_count[4], 1u);
  rig.state.solid18_law90.slab[1][0] = rig.state.solid18_law90.slab[0][0];
  CompareAll(rig);
  EXPECT_EQ(rig.state.control.status, s::BatchStatus::Success);
}
} // namespace solid_validation_test
