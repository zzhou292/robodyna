// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../CoefficientTypes.h"
#include "../source_nodal/Types.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::source_shells {
enum class Status { Ok, InvalidInput, UnsupportedProfile, ResourceLimit, NonfiniteResult };
enum class Population { Unspecified, OrdinaryShellsOnly, PhysicalShellsWithNodalSeed };
// Native working units. The caller resolves PM20 and structural THK from the
// physical material/property owner; contact thickness overrides are distinct.
struct PhysicalShell {
  std::uint64_t source_element_id = 0;
  ShellLayout layout = ShellLayout::Unspecified;
  std::uint32_t nodes[4]{UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX};
  double young = 0, structural_thickness = 0;
  double element_thickness = 0, property_thickness = 0, part_contact_thickness = 0;
};
struct Secondary {
  std::uint32_t node = UINT32_MAX;
  double existing_coefficient = 0; // Original STFNS zero is a removal mask.
};
struct Profile {
  Population population = Population::Unspecified;
  int property_type = -1, input_thickness_mode = -1;
  int level = -1, gap_mode = -1, free_edge_gap = -1, contact_thickness_update = -1;
  double stiffness_scale = 0, gap_scale = 0, maximum_secondary_gap = 0, maximum_main_gap = 0;
};
struct Input {
  Profile profile;
  std::size_t node_count = 0;
  // Original SPMD_MSIN INDEX order within each family. All Q4 rows precede T3.
  // This must include NONCONTACT physical shells too, exactly once.
  const PhysicalShell* shells = nullptr; std::size_t shell_count = 0;
  const std::uint32_t* primary_shells = nullptr; std::size_t primary_count = 0;
  const Secondary* secondary = nullptr; std::size_t secondary_count = 0;
};
struct NodeFields {
  double young_thickness_sum = 0, stiffness = 0;
  double unscaled_half_gap = 0, main_gap = 0;
  int shell_incidence_count = 0;
  bool on_main_surface = false;
};
struct MainGapFields { double corner[4]{}; double maximum = 0; };
struct SecondaryFields { double stiffness = 0, gap = 0; };
struct Limits {
  std::size_t nodes = 524288, shells = 1048576, primaries = 1048576, secondaries = 524288;
  std::size_t scratch_bytes = std::size_t{128} << 20;
};
struct Forecast { std::size_t scratch_bytes = 0, output_bytes = 0; };
struct Output {
  NodeFields* nodes = nullptr; std::size_t node_count = 0;
  double* primary_stiffness = nullptr; std::size_t primary_count = 0;
  SecondaryFields* secondary = nullptr; std::size_t secondary_count = 0;
};
struct Report {
  Status status = Status::InvalidInput;
  std::size_t physical_shell = SIZE_MAX, node = SIZE_MAX, primary = SIZE_MAX, secondary = SIZE_MAX;
};
} // namespace tlfea::contact::radioss_type25::source_shells
