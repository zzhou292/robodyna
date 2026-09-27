// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinStorage.h"
#include <algorithm>
#include <utility>

namespace tl::fea::nodal_detail {
cudaError_t CinStorage::Upload(cudaStream_t stream) {
  auto error = cudaMalloc(&arena, layout.device_bytes);
  if (error != cudaSuccess) return error;
  auto* bytes = static_cast<std::byte*>(arena);
  auto* device_rows = layout.attachments ? reinterpret_cast<constraints::tied_shell::cin::StageRow*>(bytes+layout.rows.offset) : nullptr;
  auto* device_dependent = reinterpret_cast<std::uint8_t*>(bytes+layout.dependent.offset);
  auto* device_first = layout.witnesses ? reinterpret_cast<std::uint32_t*>(bytes+layout.first_witness.offset) : nullptr;
  activity = layout.witnesses ? reinterpret_cast<std::uint8_t*>(bytes+layout.activity.offset) : nullptr;
  patches = layout.attachments ? reinterpret_cast<constraints::tied_shell::Patch*>(bytes+layout.patches.offset) : nullptr;
  work = reinterpret_cast<double*>(bytes+layout.work.offset);
  failure = util::ArenaPointer<cin_advance::FailureKey>(arena, layout.failure);
  input_failure = util::ArenaPointer<cin_advance::FailureKey>(arena, layout.input_failure);
  screen = util::ArenaPointer<cin_advance::screen::Summary>(arena, layout.screen);
  group_reports = layout.group_reports.count
      ? util::ArenaPointer<cin_advance::groups::Report>(arena, layout.group_reports) : nullptr;
  prepared_transfers = util::ArenaPointer<constraints::tied_shell::cin::detail::PreparedForceRow>(
      arena, layout.prepared_transfers);
  prepared_recovery = util::ArenaPointer<cin_advance::recovery::Row>(arena, layout.prepared_recovery);
  recovery_failure = util::ArenaPointer<cin_advance::recovery::FailureRow>(arena, layout.recovery_failure);
  prepared_drift = util::ArenaPointer<cin_advance::drift::Row>(arena, layout.prepared_drift);
  // Recovery uses its own typed tail. Its Begin/Prepare stages initialize the
  // key and every packet before any prefix publication; no startup seed is read.
  // No packet has startup meaning. The complete force-input and entry-IN stage
  // precedes a full per-row overwrite before the ordered apply reads this tail.
  // Screen records are fully written by each node kernel before reduction.
  // Private keys have no startup meaning. Every successful prefix initializes
  // its own key before any worker or completion stage can read it.
  device = {device_rows, device_dependent, std::uint32_t(layout.nodes),
            std::uint32_t(layout.attachments), std::uint32_t(layout.witnesses), device_first, source.explicitly_empty()};
  if (layout.gather.device_bytes) {
    const auto& gathered = layout.gather;
    force_gather.source_rows = device_rows;
    force_gather.nodes = util::ArenaPointer<std::uint32_t>(arena, gathered.nodes);
    force_gather.offsets = util::ArenaPointer<std::uint32_t>(arena, gathered.offsets);
    force_gather.incidence = util::ArenaPointer<std::uint32_t>(arena, gathered.incidence);
    force_gather.values = util::ArenaPointer<cin_advance::force_gather::Master>(arena, gathered.values);
    force_gather.summary = util::ArenaPointer<cin_advance::force_gather::Summary>(arena, gathered.summary);
    for (const auto pair : {std::pair{gathered.nodes, gathered.host_nodes},
                           std::pair{gathered.offsets, gathered.host_offsets},
                           std::pair{gathered.incidence, gathered.host_incidence}}) {
      error = cudaMemcpyAsync(util::ArenaPointer<std::uint32_t>(arena, pair.first),
          util::ArenaPointer<std::uint32_t>(gather_host.data(), pair.second),
          pair.first.bytes, cudaMemcpyHostToDevice, stream);
      if (error != cudaSuccess) return error;
    }
  }
  if (layout.rows.bytes) error = cudaMemcpyAsync(device_rows, rows.data(), layout.rows.bytes, cudaMemcpyHostToDevice, stream);
  if (error != cudaSuccess) return error;
  error = cudaMemcpyAsync(device_dependent, dependent.data(), layout.dependent.bytes, cudaMemcpyHostToDevice, stream);
  if (error != cudaSuccess) return error;
  if (layout.first_witness.bytes) error = cudaMemcpyAsync(device_first, first_witness.data(), layout.first_witness.bytes,
      cudaMemcpyHostToDevice, stream);
  if (error != cudaSuccess) return error;
  return ResetTrial(stream);
}
cudaError_t CinStorage::ResetTrial(cudaStream_t stream) {
  auto error = layout.activity.bytes ? cudaMemsetAsync(activity, 0, layout.activity.bytes, stream) : cudaSuccess;
  if (error != cudaSuccess) return error;
  return cudaMemsetAsync(work, 0, layout.work.bytes, stream);
}
} // namespace tl::fea::nodal_detail
