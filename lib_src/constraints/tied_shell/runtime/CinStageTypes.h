// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../TiedCinAttachmentModel.h"
#include "../TiedPatchCoefficients.h"
#include <cstdint>

namespace tl::constraints::tied_shell::cin {
enum class StageStatus {
  Success, InvalidInput, ResourceLimit, SourceMismatch, ConflictingRole,
  PendingReleaseEligibility, InvalidPatch, NonfiniteResult
};
struct StageReport {
  StageStatus status = StageStatus::Success;
  std::uint32_t row = UINT32_MAX;
  std::uint32_t node = UINT32_MAX;
  TL_TIED_PATCH_HD explicit operator bool() const noexcept {
    return status == StageStatus::Success;
  }
};
enum class WitnessFamily : std::uint32_t { ShellQuad, ShellTriangle };
// Source authentication belongs to the composing producer. The stage checks
// positive identity, family topology and the complete physical-node match.
struct ActiveWitness {
  std::uint64_t source_element_id = 0;
  std::uint32_t native_parent_index = 0;
  WitnessFamily family = WitnessFamily::ShellQuad;
  std::uint32_t nodes[4]{};
};
struct WitnessRange { std::uint32_t offset = 0, count = 0; };
struct StageRow {
  std::uint32_t secondary = 0;
  std::uint32_t masters[4]{};
  WitnessRange witnesses;
};
struct StageView {
  const StageRow* rows = nullptr;
  const std::uint8_t* dependent_nodes = nullptr;
  std::uint32_t node_count = 0, row_count = 0, witness_count = 0;
  const std::uint32_t* first_witness = nullptr;
};
// Owner-private trial destinations. No independently published state or clock.
// Force is SoA Fx/Fy/Fz/Cx/Cy/Cz; positions and accelerations are packed xyz.
struct ForceTrial {
  const double* position_xyz = nullptr;
  double* loads = nullptr;
  double* mass = nullptr;
  double* inertia = nullptr;
  double* translational_stiffness = nullptr;
  double* rotational_stiffness = nullptr;
  double* saved_secondary_mass = nullptr;
  double* saved_secondary_inertia = nullptr;
  double* numerical_mass = nullptr;
  double* entry_inertia = nullptr;
  Patch* patches = nullptr;
  // Zero means missing, one means positively active, two means inactive.
  // Every declared witness must have an explicit flag in this attempt.
  const std::uint8_t* witness_activity = nullptr;
};
struct MotionTrial {
  const Patch* patches = nullptr;
  double* velocity_xyz = nullptr;
  double* angular_velocity_xyz = nullptr;
  double* acceleration_xyz = nullptr;
  double* angular_acceleration_xyz = nullptr;
};
} // namespace tl::constraints::tied_shell::cin
