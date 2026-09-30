// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../TiedSearchTypes.h"
#include <array>
#include <cstddef>
#include <memory>
#include <vector>
namespace tl::constraints::tied_shell {
enum class FinalizationProfile { SerialFirstType2Level28 };
struct FinalizationInput {
  const std::array<std::uint32_t,4>* masters = nullptr; // Original IRECT order.
  const std::uint32_t* slaves = nullptr; // Original NSV order.
  const std::uint32_t* main_nodes = nullptr; // Original MSR order.
  const SearchChoice* choices = nullptr; // Same original NSV order.
  std::size_t node_count = 0, master_count = 0, slave_count = 0, main_node_count = 0;
  FinalizationProfile profile = FinalizationProfile::SerialFirstType2Level28;
};
struct FinalizationLimits {
  std::size_t max_nodes = 1048576, max_masters = 524288, max_slaves = 65536;
  std::size_t max_host_bytes = 64 * 1024 * 1024;
};
enum class FinalizationDisposition { Kept, Unmatched, OutsideParameters };
enum class NativeMessageAction { Accumulate, Flush, Direct };
enum class NativeMessageSeverity { Warning, Error };
struct FinalizationMessage {
  unsigned id = 0;
  NativeMessageAction action = NativeMessageAction::Accumulate;
  NativeMessageSeverity severity = NativeMessageSeverity::Warning;
  std::size_t original_slave = SIZE_MAX;
  std::uint64_t ordered_master = 0;
  double s = 0, t = 0, selection_distance = 0; // Original working units.
};
struct FinalizationMaps {
  std::vector<FinalizationDisposition> dispositions; // All original NSV rows.
  std::vector<std::uint32_t> slaves, main_nodes; // Compacted row -> original row.
  std::vector<std::uint32_t> slave_inverse, main_inverse; // UINT32_MAX absent.
  std::vector<std::uint64_t> selected_masters; // Original one-based IRECT rank.
  std::vector<std::array<double,2>> st; // CSTS; CSTS_BIS is identical for ILEV28.
  std::vector<FinalizationMessage> messages; // Native accumulation/flush/direct order.
  std::size_t cleared_dmin_entries = 0;
  std::size_t owned_payload_bytes = 0, startup_payload_bytes = 0;
};
enum class FinalizationStatus { Success, InvalidInput, ResourceLimit, AllocationFailure };
struct FinalizationReport {
  FinalizationStatus status = FinalizationStatus::Success;
  std::size_t row = SIZE_MAX;
  const char* message = "";
  explicit operator bool() const noexcept { return status == FinalizationStatus::Success; }
};
// Supplied-context value stage. The app authenticates one ordinary IS1=2
// interface, unique NSV and fresh connection tables. Physical/IRUPT buffers
// belong to the fresh-zero phase before INIEND; no arbitrary M/J is permuted.
class FinalizedSearch {
 public:
  FinalizedSearch() noexcept = default;
  const FinalizationMaps* data() const noexcept { return data_.get(); }
 private:
  std::shared_ptr<const FinalizationMaps> data_;
  friend FinalizationReport FinalizeSearch(const FinalizationInput&, FinalizedSearch*, FinalizationLimits) noexcept;
};
// Finite selected S/T outside +/-1.5 reach native warning/removal. Nonfinite
// fields and invalid ranks reject atomically; all-unmatched is a valid result.
FinalizationReport FinalizeSearch(const FinalizationInput&, FinalizedSearch*, FinalizationLimits = {}) noexcept;
} // namespace tl::constraints::tied_shell
