// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::selection {
// Additional original COR3_21 operands. References describe native ADMSR and
// already prepared ISLIDE rows; they are not inferred from boundary IDs.
struct NativeContinuationInput {
  NativePairInput pair;
  int normal_reference[4]{}, sliding_reference[4]{};
  int segment_count = 0; // Native NRTM; segment_type keeps MSEGTYP's encoding.
  int secondary_constraint = 0, secondary_skew = 0; // ICODT / ISKEW
  int main_constraint[4]{}, main_skew[4]{};
};
struct NativeContinuationResult {
  NativeGeometryHistory history;
  CandidateCache cache;
  SectorValues sector[4]{};
  double classification_product = 0, distance_squared = 0;
  int selected_subtriangle = 0;
  int sliding_match[4]{}, cylindrical_gap[4]{};
  // Axis bits x=1,y=2,z=4 report the native common-constraint decision.
  unsigned constrained_axis_mask = 0;
  bool active = false, row_replaced = false;
  // Inactive/unused channels follow the same explicit masks as retained.
};
// The complete scalar stage includes the native row winner update. A future
// one-writer row fold reuses the independently classified pair observations;
// original phase snapshots and winner comparisons remain explicit.
} // namespace tlfea::contact::radioss_type25::selection
