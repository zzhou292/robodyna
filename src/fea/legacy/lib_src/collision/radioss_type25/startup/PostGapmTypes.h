// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::startup {
// Explicit source phase; neither zero-initialized metadata nor initial-clause
// exteriority establishes the post-I25GAPM contract.
enum class PostGapmPhase { Unspecified, FinalizedBeforeNeighbors };
enum class SolidErosion { Unspecified, Disabled, Enabled };
enum class PhysicalSupportKind { Unspecified, EightSlotSolid, ShellQuad, ShellTriangle };
struct PhysicalSupportIdentity {
  PhysicalSupportKind kind=PhysicalSupportKind::Unspecified;
  std::uint64_t source_element_id=0;
};
struct PostGapmMainSupport {
  PhysicalSupportIdentity first;
  // Nonzero only when final IELEM_M(2) names a second genuine EightSlot solid.
  // This is source EID, not an invented native storage offset or raw face EID.
  std::uint64_t second_solid_source_id=0;
};
struct PreShellSolidSupport {
  // Effective INSOL3D pair after FLAG_ELEM_INTER25 filtering, BEFORE INCOQ3.
  // None is0/0. The first narrow source profile admits0/1/2 unique matches;
  // more matches need original warning/first-two source-order semantics.
  std::uint64_t first_solid_source_id=0,second_solid_source_id=0;
  std::uint32_t unique_match_count=0;
};
struct PrimaryCornerPermutation {
  // Slots in the already-expanded primary. Q4 native identity or full reverse;
  // T3 identity{0,1,2,2} or swap{1,0,2,2}. Never regenerate the SH2 partner.
  std::uint8_t source_corner[4]{0,1,2,3};
};
struct PostGapmTopology {
  PostGapmPhase phase=PostGapmPhase::Unspecified;
  const PrimaryCornerPermutation* primary_corners=nullptr;
  std::size_t primary_count=0;
  const PreShellSolidSupport* before_shell=nullptr;
  std::size_t before_shell_count=0;
  const PostGapmMainSupport* final_support=nullptr;
  std::size_t main_count=0;
  std::size_t pre_shell_internal_count=SIZE_MAX; // Native NSOL_INT, not final slot2 count.
  SolidErosion incoming_solid_erosion=SolidErosion::Unspecified;
  SolidErosion final_solid_erosion=SolidErosion::Unspecified;
  std::uint64_t source_generation=0;
};
// Original source node view for an explicit disjoint domain extension. The
// complete Input must preserve these exact node IDs and coordinate bits as its
// prefix. Existing sides retain their original node_count. App/domain binding
// separately authenticates the actual added nodes; this is a numerical view.
struct NodePrefixExtension {
  const std::uint64_t* node_source_ids=nullptr;
  VectorView positions;
  std::size_t node_count=0;
  Coordinates coordinates=Coordinates::Native;
  UnitScale units;
};
// The DTO is numerical source input, not authority. App source construction and
// runtime physical binding separately authenticate every support against the
// actual complete physical ledger and preserve all raw-origin identities.
} // namespace tlfea::contact::radioss_type25::startup
