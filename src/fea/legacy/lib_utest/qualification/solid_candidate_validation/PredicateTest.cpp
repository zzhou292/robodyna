// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"

namespace solid_validation_test {
template<class Traits> void Predicates(HostRig& rig) {
  Rows<Traits> rows(rig.state, 3);
  const auto saved = rows.trial;
  for (unsigned parent = 0; parent < 3; ++parent) {
    for (unsigned slot = 0; slot < Traits::nodes; ++slot) {
      for (unsigned axis = 0; axis < 3; ++axis) {
        rows.trial = saved;
        auto& force = rows.trial[parent].cache.rhs_force_n[slot];
        if (axis == 0) force.x = NAN;
        if (axis == 1) force.y = NAN;
        if (axis == 2) force.z = NAN;
        Validate<Traits>(rig.state, 1, 0, 0);
        for (unsigned p = 0; p < 3; ++p) {
          const auto& material = d::MaterialAt<Traits>(rig.state, rows.parents[p].material_index);
          EXPECT_EQ(rows.flags[p], d::ValidResult(rows.parents[p], material, rows.trial[p], 0, 0));
          EXPECT_EQ(rows.flags[p], p == parent ? 0 : 1);
        }
      }
    }
  }
  rows.trial = saved;
  Validate<Traits>(rig.state, 1, 0, 0);
  for (auto flag : rows.flags) EXPECT_EQ(flag, 1);
  // Deliberately stale control cannot supply the launch's validation identity.
  rig.state.control.diagnostics.time = 9;
  rig.state.control.diagnostics.epoch = 17;
  Validate<Traits>(rig.state, 1, 0, 0);
  for (auto flag : rows.flags) EXPECT_EQ(flag, 1);
  Validate<Traits>(rig.state, 1, 0, 1);
  for (auto flag : rows.flags) EXPECT_EQ(flag, 0);
  Validate<Traits>(rig.state, 1, 1e-8, 0);
  for (auto flag : rows.flags) EXPECT_EQ(flag, 0);
  rows.trial.back().history = {};
  Validate<Traits>(rig.state, 1, 0, 0);
  EXPECT_EQ(rows.flags.back(), 0);
  Validate<Traits>(rig.state, 0, 0, 0);
  for (auto flag : rows.flags) EXPECT_EQ(flag, 1);
}
TEST(SolidCandidateValidation, FiveFamiliesFreshFullPredicateAllForceSlotsAndExplicitIdentity) {
  for (int flag : {1, 2}) {
    HostRig rig;
    ASSERT_TRUE(rig.Initialize(flag));
    Predicates<d::Traits18>(rig);
    Predicates<d::Traits24>(rig);
    Predicates<d::Traits6z>(rig);
    Predicates<d::Traits18Law44>(rig);
    Predicates<d::Traits18Law90>(rig);
  }
}
template<class Traits> void FailedStatus() {
  d::Storage state;
  auto& f = d::FamilyStorage<Traits>(state);
  int status[] {7, -1};
  f.status = status;
  f.count = 2;
  // All other pointers are null: the old short-circuit must precede dereference.
  EXPECT_EQ(d::CheckParentResult<Traits>(state, 1, 0, 1, 3), 0);
  EXPECT_EQ(d::CheckParentResult<Traits>(state, 0, 1, 0, 0), 0);
}
TEST(SolidCandidateValidation, FailedElementNeverDereferencesUnavailableMaterialOrTrial) {
  FailedStatus<d::Traits18>();
  FailedStatus<d::Traits24>();
  FailedStatus<d::Traits6z>();
  FailedStatus<d::Traits18Law44>();
  FailedStatus<d::Traits18Law90>();
}
TEST(SolidCandidateValidation, AnalyticRearMaterialUsesTheSameCompleteFreshPredicate) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize(1, true));
  Predicates<d::Traits18Law44>(rig);
}
TEST(SolidCandidateValidation, CompactFlagsRequireExactEncoding) {
  for (unsigned value = 0; value < 256; ++value) {
    const std::uint8_t flag = value;
    EXPECT_EQ(d::StagedResultCheck{&flag}(0, 0, 0), value == 1);
  }
}
} // namespace solid_validation_test
