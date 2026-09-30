// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../RepresentedIntervalCrossingTypes.h"

namespace tlfea::contact {
class RepresentedIntervalCrossing;
namespace represented_interval_crossing {
class DeviceExecution;
class DeviceAccess;

// Diagnostic operation counts only: never physical state or certificate work.
struct PathRosterWork {
  std::size_t authentications = 0;
  std::size_t path_rows = 0;
  std::size_t vertex_rows = 0;
  std::size_t path_sorts = 0;
  std::size_t vertex_sorts = 0;
};

struct BatchReport {
  RepresentedIntervalStatus status = RepresentedIntervalStatus::Ok;
  const char* message = "OK";
  std::size_t input_pair = SIZE_MAX;
  RepresentedIntervalReport native_report;
  bool native_called = false;
  std::size_t batch_offset = 0;
  std::size_t prior_work = 0;
  std::size_t completed_pairs = 0;
  std::size_t completed_batches = 0;
  RepresentedIntervalResultView results;
  PathRosterWork path_roster_work;
};

// Internal compound operation. The caller borrows immutable full paths/pairs
// for this entire synchronous call and supplies separate private scratch.
// No callback or prepared authority is accepted or returned. The native busy
// lease spans all slices; all workers join before this method returns.
// Successful native slices publish in order. A later failure retains the last
// successful native slice, but never exposes a complete outer scratch view.
struct BatchAccess {
  static BatchReport Certify(
      RepresentedIntervalCrossing& crossing,
      const RepresentedTrianglePath* paths, std::size_t path_count,
      const RepresentedTrianglePair* pairs, std::size_t pair_count,
      std::size_t batch_pair_capacity,
      RepresentedIntervalResult* scratch,
      std::size_t scratch_capacity) noexcept;
 private:
  friend class DeviceAccess;
  static BatchReport CertifyUsing(RepresentedIntervalCrossing&,
      const RepresentedTrianglePath*, std::size_t, const RepresentedTrianglePair*,
      std::size_t, std::size_t, RepresentedIntervalResult*, std::size_t,
      DeviceExecution*) noexcept;
};

} // namespace represented_interval_crossing
} // namespace tlfea::contact
