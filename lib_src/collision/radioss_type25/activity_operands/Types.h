// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../activity_source/Plan.h"
#include "../current_normals/Types.h"
#include "lib_src/elements/publication/physical_activity/Types.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::activity_operands {
struct Limits {
  std::size_t max_device_bytes = 512u << 20;
  std::size_t max_host_bytes = 1u << 20;
  std::size_t max_startup_host_bytes = 256u << 20;
};
// The transaction owns slot 0. Each non-null range must outlive State and be
// disjoint from all other writable ranges. Only the nonaccepted slot is ever
// written by Stage. Main/free capacities cover the entire expanded main set;
// the initial free roster uses its actual count, not its allocation capacity.
struct BorrowedSlot {
  lifecycle::Main* mains = nullptr;
  startup::Main* normal_mains = nullptr; // Null only when current normals are disabled.
  double* normal_coefficients = nullptr;
  std::uint32_t* free_mains = nullptr; // Null/capacity0 for the fixed-normal profile.
  double* main_stiffness_si = nullptr;
  double* secondary_stiffness_si = nullptr;
  std::size_t main_capacity = 0, normal_capacity = 0, free_capacity = 0;
  std::size_t primary_capacity = 0, secondary_capacity = 0, initial_free_count = 0;
};
struct Forecast {
  TransactionReport report;
  // startup_host_bytes includes retained source Plan, our retained state,
  // upload staging and a 4 KiB preparation reservation. It excludes Plan
  // construction scratch and caller/snapshot allocations. Do not add Plan
  // retained bytes to this peak a second time.
  std::size_t owned_host_bytes = 0, startup_host_bytes = 0;
  std::size_t borrowed_device_bytes = 0, owned_device_bytes = 0;
  std::size_t scan_workspace_bytes = 0;
  // Caller retains source Plan only through Initialize; its existing host
  // allocation is a simultaneous startup dependency, not a retained duplicate.
  std::size_t source_plan_host_bytes = 0;
};
struct View {
  const lifecycle::Main* mains = nullptr;
  const startup::Main* normal_mains = nullptr;
  const double* normal_coefficients = nullptr;
  const std::uint32_t* free_mains = nullptr;
  const std::int32_t* connected_elements = nullptr; // Native NB_ELM_M, including negative values.
  const double* secondary_coefficients = nullptr; // Normalized native STFN; no contact flags.
  const double* main_stiffness_si = nullptr;
  const double* secondary_stiffness_si = nullptr;
  std::size_t main_count = 0, primary_count = 0, secondary_count = 0, free_count = 0;
};
struct StageReport {
  TransactionReport report;
  bool changed = false;
  unsigned staged_slot = 0;
  std::uint64_t affected_events = 0, removed_events = 0;
  // New nonzero->zero main coefficients, distinct from removed event multiplicity.
  std::size_t removed_mains = 0, orphan_secondaries = 0, free_count = 0;
};
} // namespace tlfea::contact::radioss_type25::activity_operands
