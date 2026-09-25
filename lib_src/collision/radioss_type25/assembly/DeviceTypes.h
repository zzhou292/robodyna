// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::assembly {
enum class IncidenceStatus { Ok, InvalidInput, ResourceLimit, DeviceFailure,
    NotInitialized, AlreadyInitialized, Unusable };
struct IncidenceLimits {
  std::size_t max_rows = 1048576, max_nodes = 1048576, max_cohorts = 1048576;
  std::size_t max_device_bytes = std::size_t{512} << 20;
};
struct IncidenceForecast { std::size_t device_bytes = 0, cub_bytes = 0, host_bytes = 0; };
// Caller-authenticated provenance, not authority to publish physical state.
struct IncidenceStamp {
  std::uint64_t source = 0, topology = 0, associations = 0, cohorts = 0, attempt = 0;
};
struct DeviceConnectivity {
  const Connectivity* rows = nullptr; // Device-readable; no host array download.
  Schedule schedule;                 // Device-readable immutable cohort ends.
  std::size_t nodes = 0;
  IncidenceStamp stamp;
};
struct IncidenceReport {
  IncidenceStatus status = IncidenceStatus::NotInitialized;
  IncidenceStamp stamp;
  // Earliest bad cohort takes priority over earliest bad occurrence.
  std::size_t bad_cohort = SIZE_MAX, bad_occurrence = SIZE_MAX;
  unsigned own_kernel_launches = 0, sort_calls = 0, host_fences = 0;
};
} // namespace tlfea::contact::radioss_type25::assembly
