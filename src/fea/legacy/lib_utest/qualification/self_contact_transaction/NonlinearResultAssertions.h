// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include <gtest/gtest.h>

namespace tlfea::contact::self_contact_transaction::test {
inline void ExpectFeature(const RepresentedFeaturePathKey& actual,
                          const RepresentedFeaturePathKey& expected) {
  EXPECT_EQ(actual.kind, expected.kind);
  EXPECT_EQ(fixed_triangle_features::Compare(actual.vertex, expected.vertex), 0);
  EXPECT_EQ(Compare(actual.face, expected.face), 0);
  for (unsigned side = 0; side < 2; ++side)
    EXPECT_EQ(fixed_triangle_features::Compare(actual.edges[side], expected.edges[side]), 0);
}

inline void ExpectResult(const NonlinearSeparationResult& actual, const NonlinearSeparationResult& expected) {
#define POLICY_FIELD(name) EXPECT_EQ(actual.name, expected.name)
  POLICY_FIELD(status); POLICY_FIELD(work); POLICY_FIELD(deepest);
  POLICY_FIELD(separated_cells); POLICY_FIELD(covered_cells); POLICY_FIELD(closed_covered_cells);
  POLICY_FIELD(accepted_certificate); POLICY_FIELD(accepted_source_order);
  ExpectFeature(actual.feature, expected.feature);
  ExpectFeature(actual.intersection_feature, expected.intersection_feature);
  POLICY_FIELD(intersection_time_numerator); POLICY_FIELD(intersection_time_depth);
  POLICY_FIELD(has_intersection); POLICY_FIELD(proof_digest);
  POLICY_FIELD(work_exhausted); POLICY_FIELD(depth_exhausted);
  POLICY_FIELD(unresolved_path); POLICY_FIELD(unresolved_depth); POLICY_FIELD(has_unresolved_cell);
  ExpectFeature(actual.transition_feature, expected.transition_feature);
  POLICY_FIELD(transition_time_lower_numerator); POLICY_FIELD(transition_time_depth);
  POLICY_FIELD(transition_time_exact); POLICY_FIELD(transition_zero_geometry_separated);
  POLICY_FIELD(has_contact_transition); POLICY_FIELD(excluded_rigid_group);
#undef POLICY_FIELD
}

}  // namespace tlfea::contact::self_contact_transaction::test
