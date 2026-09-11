// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedPatchTypes.h"
#include <cstdint>

namespace tl::constraints::tied_shell {
enum class MasterTopology { Quad, TriangleRepeatedThird };
struct SearchInput {
  PatchInput geometry_m;
  MasterTopology topology=MasterTopology::Quad;
  double master_thickness_m=0;
  double secondary_shell_thickness_m=0;
  // Explicit original working length: .001 for the Yaris millimetre deck.
  // Native dimensional floors are evaluated before converting results to SI.
  double working_length_to_m=0;
};
// Original source coordinates/thicknesses, before SI rounding. Startup source
// adapters must use this packet when a source->SI->source round trip loses bits.
// Geometry is in working units; all CandidateProjection *_m outputs remain SI.
struct WorkingSearchInput {
  PatchInput geometry;
  MasterTopology topology=MasterTopology::Quad;
  double master_thickness=0, secondary_shell_thickness=0;
  double working_length_to_m=0;
};
struct CandidateProjection {
  double gap_m=0, penetration_m=0, distance_m=0;
  // Native comparison stays in original units; SI conversion can merge two
  // distinct representable distances and must not invent a search tie.
  double selection_distance=0, working_length_to_m=0;
  double s=0, t=0;
  unsigned fan=0;
  bool admissible=false, outside_warning=false;
};
struct SearchChoice {
  bool matched=false;
  std::uint64_t ordered_master=0;
  CandidateProjection projection;
};
} // namespace tl::constraints::tied_shell
