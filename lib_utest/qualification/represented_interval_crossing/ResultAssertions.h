// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include <tuple>

namespace represented_interval_test {
inline auto VertexFields(const ct::FacetVertexKey& value) {
  return std::tie(value.source_instance_id, value.kind, value.first,
                  value.second, value.numerator, value.denominator,
                  value.level, value.grid_i, value.grid_j);
}

inline auto PathFields(const ct::RepresentedTrianglePathKey& value) {
  return std::tie(value.source_instance_id, value.parent_eid,
                  value.level, value.local_facet);
}

inline void SameEdge(const ct::FacetEdgeKey& first, const ct::FacetEdgeKey& second) {
  EXPECT_EQ(first.parent_boundary, second.parent_boundary);
  EXPECT_EQ(first.parent_eid, second.parent_eid);
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint)
    EXPECT_EQ(VertexFields(first.endpoints[endpoint]),
              VertexFields(second.endpoints[endpoint]));
}

inline void SameResult(const ct::RepresentedIntervalResult& first,
                const ct::RepresentedIntervalResult& second) {
  for (unsigned side = 0; side < 2; ++side) {
    EXPECT_EQ(PathFields(first.key.paths[side]), PathFields(second.key.paths[side]));
    SameEdge(first.feature.edges[side], second.feature.edges[side]);
  }
  EXPECT_EQ(first.feature.kind, second.feature.kind);
  EXPECT_EQ(VertexFields(first.feature.vertex), VertexFields(second.feature.vertex));
  EXPECT_EQ(PathFields(first.feature.face), PathFields(second.feature.face));
  EXPECT_EQ(first.classification, second.classification);
  EXPECT_EQ(first.reason, second.reason);
  EXPECT_EQ(first.geometry, second.geometry);
  EXPECT_EQ(first.witness_time_numerator, second.witness_time_numerator);
  EXPECT_EQ(first.witness_time_depth, second.witness_time_depth);
  EXPECT_EQ(first.work, second.work);
  EXPECT_EQ(first.accepted_event, second.accepted_event);
}

inline void SameNativeReport(const ct::RepresentedIntervalReport& first,
                      const ct::RepresentedIntervalReport& second) {
  EXPECT_EQ(std::tie(first.status, first.input_path, first.input_pair,
                     first.input_paths, first.input_pairs, first.unique_pairs,
                     first.certified_separated, first.certified_crossing_contact,
                     first.unresolved, first.work, first.total_work_limit,
                     first.rejected_pair_work),
            std::tie(second.status, second.input_path, second.input_pair,
                     second.input_paths, second.input_pairs, second.unique_pairs,
                     second.certified_separated, second.certified_crossing_contact,
                     second.unresolved, second.work, second.total_work_limit,
                     second.rejected_pair_work));
  EXPECT_STREQ(first.message, second.message);
}

inline void SameView(ct::RepresentedIntervalResultView first,
              ct::RepresentedIntervalResultView second) {
  ASSERT_EQ(first.complete, second.complete);
  ASSERT_EQ(first.count, second.count);
  if (first.count) {
    ASSERT_NE(first.data, nullptr);
    ASSERT_NE(second.data, nullptr);
  }
  for (std::size_t index = 0; index < first.count; ++index) {
    SCOPED_TRACE(index);
    SameResult(first.data[index], second.data[index]);
  }
}

}  // namespace represented_interval_test
