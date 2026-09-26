// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../source_shells/Types.h"
#include "../startup/Types.h"
namespace tlfea::contact::radioss_type25::source_gaps {
using Status = source_shells::Status;
using PhysicalShell = source_shells::PhysicalShell;
using MainGapFields = source_shells::MainGapFields;
struct Profile {
  int property_type = -1, input_thickness_mode = -1;
  int level = -1, gap_mode = -1, free_edge_gap = -1, contact_thickness_update = -1;
  double scale = 0, maximum_secondary = 0, maximum_main = 0;
};
struct Line {
  std::uint64_t source_element_id = 0;
  std::uint32_t nodes[2]{UINT32_MAX,UINT32_MAX};
  double part_contact_thickness = 0, native_area = 0; // GEO1, not STI/STP.
};
struct Spring {
  std::uint64_t native_element_id = 0;
  int property_type = 0; // First value scope admits13,25,45, never TYPE12 node3.
  std::uint32_t nodes[2]{UINT32_MAX,UINT32_MAX};
  double part_contact_thickness = 0; // No area or stiffness fallback.
};
struct Input {
  Profile profile;
  std::size_t node_count = 0;
  // Complete physical shell roster in native storage order, Q4 then T3.
  // Only nodes/layout and the three gap thickness fields are consumed.
  const PhysicalShell* shells = nullptr; std::size_t shell_count = 0;
  const Line* trusses = nullptr; std::size_t truss_count = 0;
  const Line* beams = nullptr; std::size_t beam_count = 0;
  const Spring* springs = nullptr; std::size_t spring_count = 0;
  // Primary prefix is used for the secondary shell mask; all expanded mains
  // are used for role-zero clearing and final main fields. Only nodes/type read.
  const startup::Main* mains = nullptr;
  std::size_t main_count = 0, primary_count = 0;
  const std::uint32_t* secondary_nodes = nullptr; std::size_t secondary_count = 0;
  const std::uint32_t* main_nodes = nullptr; std::size_t main_node_count = 0;
};
struct Limits {
  std::size_t nodes = 524288, shells = 1048576, lines = 16384, springs = 32768;
  std::size_t mains = 1048576, secondaries = 524288, main_nodes = 524288;
  std::size_t scratch_bytes = std::size_t{128} << 20, output_bytes = std::size_t{64} << 20;
};
struct Forecast { std::size_t scratch_bytes = 0, output_bytes = 0; };
struct Output {
  double* secondary = nullptr; std::size_t secondary_count = 0;
  double* main_nodes = nullptr; std::size_t main_node_count = 0;
  MainGapFields* mains = nullptr; std::size_t main_count = 0;
};
enum class Field { None, Shell, Truss, Beam, Spring, Main, SecondaryRoster, MainRoster, Node };
struct Report {
  Status status = Status::InvalidInput;
  Field field = Field::None;
  std::size_t input_row = SIZE_MAX;
  // Only Build success supplies these original GAPS_MN/GAPS_MX values.
  double minimum_secondary = 0, maximum_secondary = 0;
  bool completed = false;
};
} // namespace tlfea::contact::radioss_type25::source_gaps
