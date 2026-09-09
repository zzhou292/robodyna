#pragma once

#include "Q4ContactIntegrationTypes.h"

namespace tlfea::contact {
// QUALIFICATION ONLY: no production C4 selection. Each leaf is a dyadic
// rectangle. Existing scalar cell carries column/row/kind/positive integral
// bounds; its depth is max(u_depth,v_depth), NOT the binary tree path length.
// Each axis uses the existing depth cap. Path length can be u_depth+v_depth.
struct Q4RectangularCell {
  Q4IntegrationCell bounds;
  std::uint32_t u_depth=0,v_depth=0;
};
static_assert(sizeof(Q4RectangularCell)==104,"Revisit rectangular scratch forecast");
constexpr std::size_t MaxQ4RectangularScratchBytes=
    MaxQ4IntegrationLeaves*(sizeof(Q4RectangularCell)+sizeof(std::uint32_t));
static_assert(MaxQ4RectangularScratchBytes==442368,"Fixed rectangular scratch budget");
static_assert(MaxQ4RectangularScratchBytes-MaxQ4IntegrationScratchBytes==32768,
              "Declare extra independent-axis metadata");
struct Q4RectangularScratch {
  Q4RectangularCell* leaves=nullptr;
  std::uint32_t* heap=nullptr;
  std::uint32_t leaf_capacity=0,heap_capacity=0;
};
struct Q4RectangularResult {
  Q4IntegrationResult integration;
  std::uint32_t deepest_u=0,deepest_v=0;
};
// Same immutable input, absolute N/J targets, per-axis depth16, leaves4096 and
// visits16384 as scalar C2. Pointees/scratch/result must be disjoint, live in
// the execution memory space, with declared lengths. No allocation, state,
// history, clock, frame modification or accepted publication. Scratch may be
// overwritten on failure; EVERY caller result field remains unchanged.
} // namespace tlfea::contact
