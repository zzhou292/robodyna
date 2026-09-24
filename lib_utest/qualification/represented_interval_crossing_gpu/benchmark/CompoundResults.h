// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Results.h"
#include "lib_src/collision/represented_interval_crossing/DeviceBatch.h"
#include <cstring>
#include <stdexcept>
#include <tuple>

namespace native_gpu_benchmark {
namespace b = native_batch_benchmark;
namespace c = tlfea::contact;
namespace batch = c::represented_interval_crossing;
inline constexpr std::size_t PublicationSlice = 256;
inline void Require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
inline auto BatchFields(const batch::BatchReport& value) {
  return std::tie(value.status, value.input_pair, value.native_called,
      value.batch_offset, value.prior_work, value.completed_pairs, value.completed_batches,
      value.path_roster_work.authentications, value.path_roster_work.path_rows,
      value.path_roster_work.vertex_rows, value.path_roster_work.path_sorts,
      value.path_roster_work.vertex_sorts);
}
inline auto DeviceFields(const c::RepresentedIntervalDeviceReport& value) {
  return std::tie(value.status, value.device_pairs, value.host_pairs, value.batches,
      value.scene_uploads, value.consumed_device_pairs, value.numeric_cohorts,
      value.fault_cohort_begin, value.fault_cohort_count, value.fault_pair_ordinal);
}
inline void VerifyBatch(const b::Cases& cases, const b::VerifiedResults& rows,
    const batch::BatchReport& actual, c::RepresentedIntervalResultView publication,
    const batch::BatchReport* expected = nullptr) {
  Require(actual.status == c::RepresentedIntervalStatus::Ok && actual.message &&
      std::strcmp(actual.message, "OK") == 0 && actual.input_pair == SIZE_MAX && actual.native_called,
      "Compound native certification failed");
  Require(actual.completed_pairs == cases.pairs.size() &&
      actual.completed_batches == (cases.pairs.size() + PublicationSlice - 1) / PublicationSlice &&
      actual.batch_offset == ((cases.pairs.size() - 1) / PublicationSlice) * PublicationSlice,
      "Compound publication slice accounting changed");
  Require(actual.prior_work + actual.native_report.work == rows.work,
      "Compound work differs from the complete raw CPU oracle");
  const auto& auth = actual.path_roster_work;
  Require(auth.authentications == 1 && auth.path_rows == cases.paths.size() &&
      auth.vertex_rows == 3 * cases.paths.size() && auth.path_sorts == 1 && auth.vertex_sorts == 1,
      "Compound path roster must authenticate once");
  b::VerifyRows(rows.rows.data(), rows.rows.size(), actual.results);
  b::VerifyRows(rows.rows.data() + actual.batch_offset,
      rows.rows.size() - actual.batch_offset, publication);
  if (expected) {
    Require(BatchFields(actual) == BatchFields(*expected) && expected->message &&
        std::strcmp(actual.message, expected->message) == 0, "Compound report field changed");
    b::VerifyNativeReport(expected->native_report, actual.native_report);
  }
}
inline void VerifyDevice(const c::RepresentedIntervalDeviceReport& actual,
    std::size_t selected_pairs, std::size_t eligible_pairs, std::size_t cohort,
    const c::RepresentedIntervalDeviceReport* expected) {
  Require(actual.status == c::RepresentedIntervalDeviceStatus::Ok && actual.message &&
      std::strcmp(actual.message, "OK") == 0 && actual.device_pairs == eligible_pairs &&
      actual.consumed_device_pairs == eligible_pairs && actual.host_pairs == selected_pairs - eligible_pairs &&
      actual.scene_uploads == (eligible_pairs ? 1u : 0u), "Compound device routing changed");
  Require(actual.fault_cohort_begin == SIZE_MAX && actual.fault_cohort_count == 0 &&
      actual.fault_pair_ordinal == SIZE_MAX && (cohort || actual.numeric_cohorts == 0),
      "Unexpected numerical cache fault metadata");
  if (cohort && eligible_pairs) Require(actual.numeric_cohorts > 0, "Numeric cohort cache was not exercised");
  if (expected) Require(DeviceFields(actual) == DeviceFields(*expected) &&
      expected->message && std::strcmp(actual.message, expected->message) == 0,
      "Device diagnostic field changed across repetitions");
}
}  // namespace native_gpu_benchmark
