// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>

namespace tl::fea {
// Allocation payload only: excludes allocator/CUDA runtime overhead. Source is
// included once by each existing admission bound. A composing caller may deduct
// it only after proving the same immutable backing, never semantic equality.
struct ShellMappedFootprint {
  std::size_t device_bytes = 0;             // Exact sum of resident arenas.
  std::size_t source_host_bytes = 0;        // Inclusive retained physical source.
  std::size_t participant_host_bytes = 0;   // Conservative incremental retained bound.
  std::size_t startup_scratch_bytes = 0;    // Conservative temporary arenas/proof.
  std::size_t startup_host_bytes = 0;       // Existing complete admission bound.
};
namespace shell_mapped_detail {
// Existing startup budgets reserve initial proof and upload arenas together.
// Keep that conservative policy; tiny freed offset arrays/alignment allowances
// remain in participant_host_bytes. No claim of measured simultaneous RSS.
inline bool MakeFootprint(std::size_t complete, std::size_t source,
    std::size_t device, std::size_t proof, ShellMappedFootprint& output) noexcept {
  if (source > complete || device > complete - source ||
      proof > complete - source - device) return false;
  output = {device, source, complete - source - device - proof,
            device + proof, complete};
  return true;
}
} // namespace shell_mapped_detail
} // namespace tl::fea
