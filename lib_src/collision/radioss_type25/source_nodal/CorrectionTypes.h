// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::source_nodal::correction {
struct Solid {
  std::uint32_t nodes[8]{}; // Post-INITIA raw IXS order; retain repeated slots.
  int control = 0;         // Exactly one selects the correction.
  double bulk = 0;         // Native PM32 at STIFINT_ICONTROL, not a mass or STI.
  double controlled_bulk = 0; // Native PM107 at this same phase.
};
struct Input {
  const double* coefficients = nullptr; // Complete already-finalized ASSTIFI K.
  std::size_t node_count = 0;
  const Solid* solids = nullptr;
  std::size_t solid_count = 0;
  // Flattened native interface/secondary order for TYPE24 with IGSTI=-1 only.
  // Ordinals >= node_count are native virtual nodes and are skipped.
  const std::uint32_t* type24_secondaries = nullptr;
  std::size_t type24_secondary_count = 0;
};
struct Output { double* coefficients = nullptr; std::size_t node_count = 0; };
struct Limits {
  std::size_t nodes = 524288;
  std::size_t solids = 131072;
  std::size_t type24_occurrences = 1048576;
  std::size_t scratch_bytes = std::size_t{64} << 20;
};
struct Forecast { std::size_t scratch_bytes = 0, output_bytes = 0; };
enum class Status { Ok, InvalidInput, ResourceLimit, NonfiniteResult };
struct Report {
  Status status = Status::InvalidInput;
  std::size_t solid = SIZE_MAX;
  std::size_t type24_occurrence = SIZE_MAX;
  std::uint32_t node = UINT32_MAX;
};
}
