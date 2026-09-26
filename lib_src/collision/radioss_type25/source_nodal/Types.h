// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>

namespace tlfea::contact::radioss_type25 {
// Native working quantities before ASSTIFI. Physical mass/J and structural STI
// are not these values. Source/domain/order authority belongs to the caller.
struct NativeNodalSeed {
  double volume = 0;
  double bulk_volume = 0;
  double existing_stiffness = 0;
};
struct NativeNodalSeedView {
  const NativeNodalSeed* nodes = nullptr;
  std::size_t node_count = 0;
};
struct NativeVolumeOccurrence {
  std::uint32_t node = UINT32_MAX;
  double volume = 0;
  double bulk_volume = 0;
};
struct NativeStiffnessOccurrence {
  std::uint32_t node = UINT32_MAX;
  double stiffness = 0;
};

namespace source_nodal {
enum class Status { Ok, InvalidInput, ResourceLimit, NonfiniteResult };
enum class Channel { None, Volume, Stiffness };
struct Input {
  std::size_t node_count = 0;
  // Native solid stage/EID/raw-slot order, retaining repeats and initialized
  // zero Penta slots. No unique-node reduction or sorting occurs here.
  const NativeVolumeOccurrence* volumes = nullptr;
  std::size_t volume_count = 0;
  // Native truss, beam, then combined spring stage/EID/endpoint order.
  // Keep TYPE45's source-defined zeros in the complete declared schedule.
  const NativeStiffnessOccurrence* stiffness = nullptr;
  std::size_t stiffness_count = 0;
};
struct Output {
  NativeNodalSeed* nodes = nullptr;
  std::size_t node_count = 0;
};
struct Limits {
  std::size_t nodes = 524288;
  std::size_t volume_occurrences = 1048576;
  std::size_t stiffness_occurrences = 1048576;
  std::size_t scratch_bytes = std::size_t{64} << 20;
};
struct Forecast {
  std::size_t scratch_bytes = 0;
  std::size_t output_bytes = 0;
};
struct Report {
  Status status = Status::InvalidInput;
  Channel channel = Channel::None;
  std::size_t occurrence = SIZE_MAX;
  std::uint32_t node = UINT32_MAX;
};
} // namespace source_nodal
} // namespace tlfea::contact::radioss_type25
