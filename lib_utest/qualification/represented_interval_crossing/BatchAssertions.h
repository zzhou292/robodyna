// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/Batch.h"
namespace represented_interval_test {
inline void SameBatch(const ct::represented_interval_crossing::BatchReport& first,
                      const ct::represented_interval_crossing::BatchReport& second) {
  EXPECT_EQ(std::tie(first.status, first.input_pair, first.native_called,
                     first.batch_offset, first.prior_work,
                     first.completed_pairs, first.completed_batches),
            std::tie(second.status, second.input_pair, second.native_called,
                     second.batch_offset, second.prior_work,
                     second.completed_pairs, second.completed_batches));
  EXPECT_STREQ(first.message, second.message);
  SameNativeReport(first.native_report, second.native_report);
  SameView(first.results, second.results);
  if (!second.results.complete) EXPECT_EQ(second.results.data, nullptr);
}
inline void SameRosterWork(const ct::represented_interval_crossing::PathRosterWork& first,
                           const ct::represented_interval_crossing::PathRosterWork& second) {
  EXPECT_EQ(std::tie(first.authentications, first.path_rows, first.vertex_rows,
                     first.path_sorts, first.vertex_sorts),
            std::tie(second.authentications, second.path_rows, second.vertex_rows,
                     second.path_sorts, second.vertex_sorts));
}
inline void OneFullAuthentication(const ct::represented_interval_crossing::BatchReport& report,
                                  std::size_t paths) {
  EXPECT_EQ(report.path_roster_work.authentications, 1u);
  EXPECT_EQ(report.path_roster_work.path_rows, paths);
  EXPECT_EQ(report.path_roster_work.vertex_rows, 3 * paths);
  EXPECT_EQ(report.path_roster_work.path_sorts, 1u);
  EXPECT_EQ(report.path_roster_work.vertex_sorts, 1u);
}
}  // namespace represented_interval_test
