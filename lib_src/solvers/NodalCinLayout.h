// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalCinRuntime.h"
#include "cin_advance/FailureKey.h"
#include "cin_advance/ScreenSummary.h"
#include "cin_advance/GroupReport.h"
#include "cin_advance/RecoveryTypes.h"
#include "cin_advance/DriftTypes.h"
#include "../constraints/tied_shell/runtime/CinForceTransfer.h"
#include "lib_utils/BoundedArena.h"
#include "lib_utils/SourceIdentityIndex.h"

namespace tl::fea::nodal_detail {
inline bool CinOwnerHostFits(std::size_t optional_bytes, std::size_t rigid_bytes,
    std::size_t state_capacity, std::size_t constraint_capacity, std::size_t owner_bytes,
    std::size_t maximum_bytes,std::size_t* output_bytes=nullptr) noexcept {
  util::BoundedArenaLayout host(maximum_bytes);
  util::ArenaRegion unused;
  const bool fits = host.Append<std::byte>(optional_bytes, unused) &&
      host.Append<std::byte>(rigid_bytes, unused) &&
      host.Append<double>(state_capacity, unused) &&
      host.Append<std::uint8_t>(constraint_capacity, unused) &&
      host.Append<std::byte>(owner_bytes, unused);
  if (fits && output_bytes) *output_bytes = host.bytes();
  return fits;
}
struct CinLayout {
  util::ArenaRegion rows, dependent, activity, patches, work, first_witness, failure, input_failure, screen, group_reports;
  util::ArenaRegion prepared_transfers, prepared_recovery, recovery_failure;
  util::ArenaRegion prepared_drift;
  // Tail within each of the existing accepted/trial double slabs:
  // M[n], J[n], derived inverse M[n], derived inverse J[n], SMAS[r], SINER[r], DMAST.
  std::size_t nodes = 0, attachments = 0, witnesses = 0;
  std::size_t state_values = 0, device_bytes = 0, host_bytes = 0;
  std::size_t optional_device_bytes = 0;
  std::size_t scratch_values = 0;
  bool Initialize(std::size_t n, std::size_t r, std::size_t w,
      const NodalCinLimits& limits, std::size_t host_control_bytes, std::size_t group_count = 0, bool explicitly_empty = false) noexcept {
    if (!n || group_count > n/2 || n > MaxActiveNodalStateNodes ||
        (explicitly_empty ? (r!=0 || w!=0) : (!r || !w)) || r > limits.max_attachments ||
        limits.max_attachments > 65536 || w > limits.max_witnesses ||
        limits.max_witnesses > 262144 || !limits.max_host_bytes ||
        limits.max_host_bytes > (128u << 20) || !limits.max_device_bytes ||
        limits.max_device_bytes > (128u << 20)) return false;
    CinLayout next;
    next.nodes = n;
    next.attachments = r;
    next.witnesses = w;
    next.state_values = 4*n + 2*r + 1;
    // STIFN, STIFR, entry IN, current A3 and AR3. After a completed
    // advance, the last 7*n values may stage the TT0 physical main query;
    // captured A/AR already belongs to the separate force-stage buffer.
    next.scratch_values = 9*n;
    util::BoundedArenaLayout device(limits.max_device_bytes);
    if (!device.Append<constraints::tied_shell::cin::StageRow>(r, next.rows) ||
        !device.Append<std::uint8_t>(n, next.dependent) ||
        !device.Append<std::uint8_t>(w, next.activity) ||
        !device.Append<std::uint32_t>(w, next.first_witness) ||
        !device.Append<constraints::tied_shell::Patch>(r, next.patches) ||
        !device.Append<double>(next.scratch_values, next.work) ||
        !device.Append<cin_advance::FailureKey>(1, next.failure) ||
        !device.Append<cin_advance::FailureKey>(1, next.input_failure) ||
        !device.Append<cin_advance::screen::Summary>(cin_advance::screen::Blocks(n), next.screen) ||
        !device.Append<cin_advance::groups::Report>(group_count, next.group_reports) ||
        !device.Append<constraints::tied_shell::cin::detail::PreparedForceRow>(r, next.prepared_transfers) ||
        !device.Append<cin_advance::recovery::Row>(r, next.prepared_recovery) ||
        !device.Append<cin_advance::recovery::FailureRow>(1, next.recovery_failure) ||
        !device.Append<cin_advance::drift::Row>(r, next.prepared_drift)) return false;
    next.device_bytes = device.bytes();
    const auto tail_bytes = 2*next.state_values*sizeof(double);
    if (tail_bytes > limits.max_device_bytes-next.device_bytes) return false;
    next.optional_device_bytes = next.device_bytes+tail_bytes;
    util::BoundedArenaLayout host(limits.max_host_bytes);
    util::ArenaRegion unused;
    if (!host.Append<std::byte>(host_control_bytes, unused) ||
        !host.Append<constraints::tied_shell::cin::StageRow>(r, unused) ||
        !host.Append<std::uint8_t>(n, unused) ||
        !host.Append<constraints::tied_shell::cin::ActiveWitness>(w, unused) ||
        !host.Append<std::uint32_t>(w, unused) ||
        !host.Append<std::byte>(util::SourceIdentityIndex<16>::Bytes(w), unused)) return false;
    next.host_bytes = host.bytes();
    // The caller also charges both tails and the existing host state staging.
    *this = next;
    return true;
  }
};
} // namespace tl::fea::nodal_detail
