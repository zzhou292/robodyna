#pragma once

#include "ReissnerShellForce.h"
#include "ReissnerShellMass.h"
#include "../solvers/FENodalState.h"
#include <memory>

namespace tl::fea::reissner {

constexpr std::size_t MaxReissnerShellBatchElements = 2;
constexpr std::size_t MaxReissnerShellBatchDeviceBytes = 1024 * 1024;

struct ReissnerShellBatchElement {
  ShellReference reference;
  ElasticSection section;
  std::size_t nodes[4]{};
};
struct ReissnerShellBatchConfig {
  NodalStamp owner;
  std::uint64_t configuration_id = 0;
  std::size_t element_count = 0;
  std::size_t max_device_bytes = MaxReissnerShellBatchDeviceBytes;
  ShellDrillingInertiaPolicy drilling_policy = ShellDrillingInertiaPolicy::kNone;
};
enum class ShellBatchStatus {
  kSuccess, kInvalidInput, kNotInitialized, kResourceLimit, kWrongOwner,
  kStaleTrial, kInvalidMass, kElementFailure, kAssemblyFailure,
  kInvalidGeometry, kNonfiniteResult, kDeviceFailure
};
struct ShellBatchReport {
  ShellBatchStatus status = ShellBatchStatus::kInvalidInput;
  const char* message = "Invalid batch request";
  std::uint32_t element = UINT32_MAX, node = UINT32_MAX;
  ShellStatus element_status = ShellStatus::kSuccess;
};
enum class ShellBatchPhase { kUnspecified, kAcceptedBase, kPreparedCandidate };

// Small scalar readback, never an accepted physical state or authority to
// commit. Kinetic energy sums each element's row-sum mass contribution once
// using the owner's single shared velocity field. This is equivalent to
// assembled nodal masses, not independent element velocity state.
struct ShellBatchDiagnostics {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0, configuration_id = 0;
  ShellBatchPhase phase = ShellBatchPhase::kUnspecified;
  bool valid = false;
  double elastic_energy = 0, bending_energy = 0;
  double kinetic_translation = 0, kinetic_physical_rotation = 0, kinetic_artificial_drilling = 0;
  double maximum_displacement = 0, maximum_director_departure = 0, maximum_pair_angle = 0;
  double maximum_membrane_strain = 0, maximum_thickness_curvature = 0;
  double minimum_signed_area_ratio = 0, minimum_area_norm_ratio = 0, maximum_area_norm_ratio = 0;
  double minimum_display_triangle_area_ratio = 0;
  // Candidate-only interval diagnostics. Work includes THIS batch's force
  // contribution. The no-load coupon has exactly this one conservative batch.
  double base_elastic_energy = 0, base_kinetic_energy = 0;
  double elastic_energy_increment = 0, kinetic_energy_increment = 0;
  double kinetic_midpoint_work = 0, kinetic_work_residual = 0;
  double force_coordinate_work = 0, conservative_force_coordinate_defect = 0;
  double mass_weighted_increment_squared = 0;
};

// Resident reference/connectivity/mass and force scratch only. This batch owns
// no x/v/q state, dynamics clock, accepted history or integration policy. The
// current gate admits at most two Q4s covering the owner's complete node space,
// <=64 nodes, with explicit equal physical/artificial drilling inertia policy.
// Every shared node's reference frames/position must agree. No Chrono objects
// are retained or allocated here.
//
// Initialize copies immutable host inputs. Assemble reads accepted owner views,
// validates actual inverse masses/inertia/constraints, evaluates all elements,
// and adds to the existing force assembly. Its failure is sticky in every
// otherwise-valid assembly view. Missing/invalid failure-channel pointers
// cannot be repaired: the caller must discard after ANY unsuccessful report.
// An assembly view may be consumed only once per attempt to prevent duplicate
// contributions. EvaluateCandidate uses the owner's prepared and base views;
// it neither advances nor commits. Epoch/attempt/configuration identity reject
// stale diagnostics but cannot authenticate omitted participants or raw writes.
//
// Calls are serialized, use the supplied owner stream, and finish before
// returning. No readback/allocation of complete kinematics occurs per step.
// Caller diagnostics remain unchanged on every failure. A detected CUDA error
// poisons this batch; its report also requires discarding/handling the owner.
class ReissnerShellBatch {
 public:
  ReissnerShellBatch();
  ~ReissnerShellBatch();
  ReissnerShellBatch(const ReissnerShellBatch&) = delete;
  ReissnerShellBatch& operator=(const ReissnerShellBatch&) = delete;
  ShellBatchReport Initialize(const ReissnerShellBatchConfig&, const ReissnerShellBatchElement*);
  ShellBatchReport Assemble(const NodalAssemblyView&, ShellBatchDiagnostics*);
  ShellBatchReport EvaluateCandidate(const NodalPreparedView&, ShellBatchDiagnostics*);
  // Output-cadence result scratch, tagged by the most recent successful
  // diagnostic identity/phase. The case stages candidate results before Commit
  // and publishes them only after that commit succeeds. A failed or intervening
  // evaluation invalidates this readback; no accepted history is held here.
  ShellBatchReport CopyElementResults(const ShellBatchDiagnostics& expected,
                                      ShellResult* output, std::size_t capacity);
  NodalAllocationInfo allocations() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tl::fea::reissner
