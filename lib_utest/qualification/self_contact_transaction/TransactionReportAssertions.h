// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/SelfContactTransactionTypes.h"
#include "lib_src/collision/fixed_triangle_features/Geometry.h"
#include <gtest/gtest.h>
#include <cstring>

namespace tlfea::contact::self_contact_transaction::test {
namespace c = ::tlfea::contact;
inline std::uint64_t ReportBits(double value) noexcept {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}
inline void ExpectTransactionReport(const c::SelfContactTransactionReport& a,
                 const c::SelfContactTransactionReport& b) {
#define FIELD(name) EXPECT_EQ(a.name, b.name)
  FIELD(status); FIELD(candidate); FIELD(pair); FIELD(count_kind);
  FIELD(force_status); FIELD(activity_status); FIELD(broadphase_status);
  FIELD(regularity_status); FIELD(discovery_status); FIELD(discovery_task);
  FIELD(discovery_reason); FIELD(crossing_status); FIELD(crossing_reason);
  FIELD(nonlinear_subdivision_work); FIELD(nonlinear_subdivision_depth);
  FIELD(nonlinear_subdivision_work_exhausted);
  FIELD(nonlinear_subdivision_depth_exhausted);
  FIELD(publication_status); FIELD(owner_status); FIELD(filter_status);
  FIELD(filter_scope); FIELD(filter_chunk_begin); FIELD(filter_chunk_pairs);
  FIELD(crossing_device_status); FIELD(crossing_fault_cohort_begin);
  FIELD(crossing_fault_cohort_count); FIELD(crossing_fault_pair_ordinal);
  FIELD(crossing_diagnostics.available);
  FIELD(crossing_diagnostics.batch_pair_offset);
  FIELD(crossing_diagnostics.prior_batch_work);
  FIELD(crossing_diagnostics.input_path);
  FIELD(crossing_diagnostics.input_pair);
  FIELD(crossing_diagnostics.input_paths);
  FIELD(crossing_diagnostics.input_pairs);
  FIELD(crossing_diagnostics.unique_pairs);
  FIELD(crossing_diagnostics.certified_separated);
  FIELD(crossing_diagnostics.certified_crossing_contact);
  FIELD(crossing_diagnostics.unresolved);
  FIELD(crossing_diagnostics.admitted_work);
  FIELD(crossing_diagnostics.total_work_limit);
  FIELD(crossing_diagnostics.rejected_pair_work);
#undef FIELD
  EXPECT_STREQ(a.message, b.message);
  EXPECT_EQ(ReportBits(a.offending_feature_distance_m),
            ReportBits(b.offending_feature_distance_m));
  const auto vector = [](c::Vec3 first, c::Vec3 second) {
    EXPECT_EQ(ReportBits(first.x), ReportBits(second.x));
    EXPECT_EQ(ReportBits(first.y), ReportBits(second.y));
    EXPECT_EQ(ReportBits(first.z), ReportBits(second.z));
  };
  for (unsigned side = 0; side < 2; ++side) {
    const auto& first = a.offending_motion[side];
    const auto& second = b.offending_motion[side];
    EXPECT_EQ(c::fixed_triangle_features::Compare(first.facet, second.facet), 0);
    EXPECT_EQ(first.active_parent, second.active_parent);
    EXPECT_EQ(first.motion, second.motion);
    EXPECT_EQ(first.rigid_group_count, second.rigid_group_count);
    for (unsigned group = 0; group < 4; ++group) {
      const auto& x = first.rigid_groups[group];
      const auto& y = second.rigid_groups[group];
      EXPECT_EQ(x.binding_group, y.binding_group);
      EXPECT_EQ(x.source_kind, y.source_kind);
      EXPECT_EQ(x.source_group_id, y.source_group_id);
      EXPECT_EQ(x.source_node_set_id, y.source_node_set_id);
    }
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      vector(a.offending_quadratic_lower[side][vertex],
             b.offending_quadratic_lower[side][vertex]);
      vector(a.offending_quadratic_upper[side][vertex],
             b.offending_quadratic_upper[side][vertex]);
    }
    vector(a.offending_swept_bounds[side].lower,
           b.offending_swept_bounds[side].lower);
    vector(a.offending_swept_bounds[side].upper,
           b.offending_swept_bounds[side].upper);
    EXPECT_EQ(ReportBits(a.offending_half_thickness_m[side]),
              ReportBits(b.offending_half_thickness_m[side]));
    EXPECT_EQ(ReportBits(a.offending_edge_parameters[side]),
              ReportBits(b.offending_edge_parameters[side]));
  }
}

}  // namespace tlfea::contact::self_contact_transaction::test
