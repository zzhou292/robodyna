// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../startup/Types.h"
namespace tlfea::contact::radioss_type25::search_startup {
enum class Status { Ok, InvalidInput, UnsupportedProfile, UnsupportedArithmetic,
  NonfiniteResult, NoProgress, ResourceLimit };
enum class Census { Unspecified, CompleteDeclaredModel };
enum class LoadCards { Unspecified, Absent };
enum class Initialization { Unspecified, SerialNative, InvariantNoExpansion };
struct Contributors {
  Census census = Census::Unspecified;
  std::size_t physical_nodes = 0, physical_shells = 0;
  std::size_t tied_interfaces = 0, rigid_bodies = 0, cin_links = 0;
  std::size_t other_interfaces = 0, unsupported_elements = 0;
  // Source-declared generated rigid primaries outside the physical geometry
  // domain. Never append their mass/coordinates to the physical node ledger.
  std::size_t native_auxiliary_nodes = 0;
  // This is declared model data, not a physical authority token. The source
  // factory must supply the complete census; a contact subset is insufficient.
};
struct Profile {
  int level = -1, gap_mode = -1, neighbor_removal = -1;
  int initial_penetration = -1, edge_mode = -1, thermal_mode = -1;
  int curvature = -1, partitions = -1;
  Initialization initialization = Initialization::Unspecified;
  LoadCards gap_load_cards = LoadCards::Unspecified;
};
struct Secondary {
  std::uint32_t node = UINT32_MAX;
  double stiffness = 0, gap = 0; // Resolved native working units.
};
struct Input {
  startup::Input mesh;
  startup::Snapshot topology; // Immutable actual BuildStarter output for mesh.
  Contributors contributors;
  Profile profile;
  const Secondary* secondary = nullptr;
  std::size_t secondary_count = 0;
  const double* main_gaps = nullptr;
  std::size_t main_count = 0; // Complete expanded sides, source order.
  const std::uint64_t* auxiliary_rigid_primary_ids = nullptr;
  std::size_t auxiliary_rigid_primary_count = 0;
};
struct Limits {
  std::size_t max_nodes = 1048576, max_mains = 2097152, max_secondaries = 1048576;
  std::size_t max_removals = 16777216, max_neighbor_visits = 268435456;
  std::size_t max_margin_iterations = 128;
  std::size_t max_output_bytes = std::size_t{256} << 20;
  std::size_t max_scratch_bytes = std::size_t{512} << 20;
  // Only a scalar population count, not an additional geometry allocation.
  std::size_t max_native_model_nodes = 2147483647u;
};
struct Forecast {
  Status status = Status::InvalidInput;
  std::size_t output_bytes = 0, scratch_bytes = 0, removal_capacity = 0;
};
struct Snapshot {
  double multiplier = 0, mean_length = 0, margin = 0, maximum_extent = 0;
  const double* primary_extent = nullptr;
  std::size_t primary_count = 0;
  // Native KREMNODE / REMNODE: zero-based domain-node entries per expanded main.
  const std::uint32_t* main_offsets = nullptr;
  const std::uint32_t* removed_nodes = nullptr;
  // Native KREMNOR / REMNOR: one-based local-main entries per secondary row.
  const std::uint32_t* secondary_offsets = nullptr;
  const std::uint32_t* removed_mains = nullptr;
  const int* initial_contact = nullptr;
  std::size_t main_count = 0, secondary_count = 0, removal_count = 0;
  std::uint64_t source_generation = 0;
  std::size_t native_model_nodes = 0; // Physical geometry plus authenticated auxiliary IDs.
};
// Not a final interface-removal result. A genuine complete TYPE2 augmentation
// must consume these geometric arrays before a source factory admits them.
// There is deliberately no implicit conversion to final Snapshot.
struct GeometricSnapshot {
  Snapshot geometry;
  Contributors contributors;
};
struct Report {
  Status status = Status::InvalidInput;
  std::size_t main = SIZE_MAX, node = SIZE_MAX, required_removals = 0;
  bool removal_count_complete = false;
};
} // namespace tlfea::contact::radioss_type25::search_startup
