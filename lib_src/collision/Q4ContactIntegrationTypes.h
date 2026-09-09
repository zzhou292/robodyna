#pragma once

#include "Q4SurfaceMapping.h"
#include "Q4SurfaceMass.h"

#include <cstddef>

namespace tlfea::contact {

constexpr std::uint32_t MaxQ4IntegrationLeaves = 4096;
constexpr std::uint32_t MaxQ4IntegrationDepth = 16;
constexpr std::uint32_t MaxQ4IntegrationVisits = 16384;

struct Q4IntegralInterval { double lower = 0, upper = 0; };
struct Q4CertifiedIntegral {
  double value = 0, lower = 0, upper = 0, error = 0;
};
enum class Q4IntegrationCellKind : std::uint32_t { Inactive, Active, Mixed };

// A dyadic parameter-square leaf, including active/inactive leaves for replay.
// Five positive integrals: four normal nodal force magnitudes, then potential.
// Do not put a full-capacity array of these on a CUDA thread's stack.
struct Q4IntegrationCell {
  std::uint32_t column = 0, row = 0, depth = 0;
  Q4IntegrationCellKind kind = Q4IntegrationCellKind::Inactive;
  Q4IntegralInterval integrals[5]{};
};
static_assert(sizeof(Q4IntegrationCell) == 96, "Revisit the contact allocation forecast");
constexpr std::size_t MaxQ4IntegrationScratchBytes =
    MaxQ4IntegrationLeaves * (sizeof(Q4IntegrationCell) + sizeof(std::uint32_t));
static_assert(2 * MaxQ4IntegrationScratchBytes < 1024 * 1024,
              "Two-parent integration scratch must leave room for finite-wall data");

struct Q4IntegrationScratch {
  Q4IntegrationCell* leaves = nullptr;
  std::uint32_t* heap = nullptr;
  std::uint32_t leaf_capacity = 0, heap_capacity = 0;
};
struct Q4IntegrationLimits {
  std::uint32_t max_leaves = MaxQ4IntegrationLeaves;
  std::uint32_t max_depth = MaxQ4IntegrationDepth;
  std::uint32_t max_visited = MaxQ4IntegrationVisits;
  // Absolute N/J error targets, declared before evaluation. No silent floor,
  // tolerance change or relative division by the current contact force.
  double force_error = 0, energy_error = 0;
};
struct Q4NormalIntegrationInput {
  Q4SurfaceView surface;
  Q4FixedYZMassView mass;
  std::uint32_t parent_index = 0;
  std::uint64_t attempt = 0;
  double wall_x = 0, projected_area = 0, stiffness_per_area = 0;
  double max_penetration = 0;
};

// Distinct prescribed input, never a conversion into constrained C2 masses.
// The actual physical nodes are free in XYZ or fully fixed; partial component
// constraints are unsupported by LumpedTranslationMassView. The material
// measure remains the immutable rectangular reference area. Moving projected
// coverage/Jacobian safety belongs to CheckQ4PlanarSweep/the host adapter, not
// to the raw integral. Keeping this POD distinct preserves the legacy API.
struct Q4PrescribedNormalIntegrationInput {
  Q4SurfaceView surface;
  LumpedTranslationMassView mass;
  std::uint32_t parent_index = 0;
  std::uint64_t attempt = 0;
  double wall_x = 0, projected_area = 0, stiffness_per_area = 0;
  double max_penetration = 0;
};

enum class Q4IntegrationStatus : std::uint8_t {
  Ok, InvalidInput, UnsupportedInput, NonFiniteArithmetic, NoDynamicDofs,
  PenetrationLimit, LeafLimit, VisitLimit, DepthLimit, UnattainableAccuracy
};
struct Q4IntegrationReport {
  Q4IntegrationStatus status = Q4IntegrationStatus::InvalidInput;
  Status cause = Status::kInvalidArgument;
  std::uint32_t cell = UINT32_MAX, depth = 0, visited = 0, leaves = 0;
};
struct Q4IntegrationResult {
  Q4NodalForces nodal;
  Q4CertifiedIntegral force[4];  // Positive magnitudes before -X scatter.
  Q4CertifiedIntegral resultant, potential;
  Q4IntegralInterval active_area;
  std::uint64_t feature_id = 0, parent_element_id = 0;
  std::uint64_t base_epoch = 0, attempt = 0;
  std::uint32_t leaf_count = 0, visited = 0, deepest_leaf = 0;
  bool valid = false;
};

// Pure prescribed-contact integration contract (implemented in the companion
// header). Geometry preparation must separately establish the fixed rectangular
// YZ footprint, supplied projected area and complete finite-wall coverage.
// This operation certifies the positive-part bilinear normal integral only.
// It reuses the C1 physical parent, mass, interpolation and force transpose;
// no source binding, owner state, accepted history, assembly or clock is owned.
// Input pointees, scratch and output must be disjoint, live in the executing
// memory space and have the declared lengths. Scratch is temporary and may be
// overwritten on failure. Every caller result field is preserved on failure.

}  // namespace tlfea::contact
