// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../TiedSearchTypes.h"
#include <array>
#include <cstddef>
#include <vector>

namespace tl::constraints::tied_shell {
struct SearchMasterInput {
  std::array<std::uint32_t, 4> nodes{};
  MasterTopology topology = MasterTopology::Quad;
  double bounds_thickness = 0;
  double projection_thickness = 0;
};
struct SearchDriverInput {
  const Vec3* working_positions = nullptr;
  const SearchMasterInput* masters = nullptr; // Complete IRECT order.
  const std::uint32_t* secondary_nodes = nullptr; // Complete NSV order.
  std::size_t node_count = 0;
  std::size_t master_count = 0;
  std::size_t secondary_count = 0;
  double working_length_to_m = 0;
  // This first explicit source profile requires zero secondary shell incidence.
  double maximum_secondary_shell_thickness = 0;
};
struct SearchDriverLimits {
  std::size_t max_nodes = 1048576;
  std::size_t max_masters = 524288;
  std::size_t max_secondaries = 65536;
  std::size_t max_pairs = 8388608;
  std::size_t max_host_bytes = 512 * 1024 * 1024;
  std::size_t max_device_bytes = 128 * 1024 * 1024;
  unsigned axis = 0;
};
struct SearchDriverBudget {
  std::size_t startup_host_bytes = 0; // Internal payload; excludes borrowed input.
  std::size_t device_bytes = 0;
  std::size_t sort_scratch_bytes = 0;
  std::size_t scan_scratch_bytes = 0;
  std::size_t pair_capacity = 0;
};
struct SearchDriverRow {
  SearchChoice choice; // ordered_master is one-based IRECT rank when matched.
  std::size_t candidates = 0;
  std::size_t within_native_bounds = 0;
  std::size_t admissible_candidates = 0;
  std::size_t excluded_own_node = 0;
  Status force_patch_status = Status::InvalidInput; // Not evaluated when unmatched.
  Patch force_patch;
};
struct SearchDriverResult {
  std::vector<SearchDriverRow> rows; // All NSV rows, including unmatched.
  SearchDriverBudget budget;
  std::size_t pair_count = 0;
  std::size_t matched_count = 0;
  std::size_t singular_patch_count = 0;
  unsigned axis = 0;
  // This result has no I2TID3 finalization, classifier, mechanics owner or clock.
};
enum class SearchDriverStatus {
  Success, InvalidInput, ResourceLimit, NumericalFailure, CudaFailure, AllocationFailure
};
struct SearchDriverReport {
  SearchDriverStatus status = SearchDriverStatus::Success;
  Status numerical_status = Status::Success;
  std::size_t secondary = SIZE_MAX;
  std::size_t master = SIZE_MAX;
  const char* message = "";
  explicit operator bool() const noexcept { return status == SearchDriverStatus::Success; }
};

// GPU sweep-and-prune, then shared ordered native projection values on host.
// Borrowed inputs live through this synchronous startup call only. All output
// publication is staged; every rejection preserves output and permits retry.
// Byte admission includes CUB scratch and all internal simultaneous payload,
// but the caller separately charges its borrowed source backing exactly once.
SearchDriverReport AssessSearch(const SearchDriverInput&, const SearchDriverLimits&,
                               SearchDriverResult&) noexcept;
} // namespace tl::constraints::tied_shell
