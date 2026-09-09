#pragma once

#include "Q4PlanarGeometry.h"
#include "lib_src/solvers/FENodalState.h"

#include <memory>

namespace tlfea::contact {

// Immutable execution selection. Rectangular refinement is opt-in staging;
// selecting it does not qualify a new guided or vehicle trajectory.
enum class Q4PlanarIntegrationBackend { ScalarDyadicSquares, RectangularDyadic };

struct Q4PlanarContactConfig {
  tl::fea::NodalStamp owner;
  std::uint64_t configuration_id = 0, wall_binding_id = 0;
  double stiffness_per_area = 0, maximum_penetration = 0;
  double exposed_clearance = 1e-6;
  Q4IntegrationLimits integration;
  std::size_t max_device_bytes = MaxPlanarContactDeviceBytes;
  Q4PlanarIntegrationBackend integration_backend = Q4PlanarIntegrationBackend::ScalarDyadicSquares;
};
enum class Q4PlanarContactStatus {
  Ok, InvalidInput, NotInitialized, ResourceLimit, WrongOwner, StaleAttempt,
  InvalidMass, GeometryFailure, IntegrationFailure, AssemblyFailure,
  NonFiniteArithmetic, DeviceFailure
};
struct Q4PlanarContactReport {
  Q4PlanarContactStatus status = Q4PlanarContactStatus::InvalidInput;
  const char* message = "Invalid Q4 contact request";
  std::uint32_t parent = UINT32_MAX, node = UINT32_MAX;
  PlanarContactStatus geometry = PlanarContactStatus::Ok;
  Q4IntegrationReport integration;
};
enum class Q4PlanarContactPhase { Unspecified, AcceptedBase, PreparedCandidate };

struct Q4PlanarParentResult {
  bool covered = false;
  // Uncovered parents have an invalid/default integration result. Covered
  // results retain C2's numerical values, with certificates expanded through
  // C3's area enclosure to the exact coordinate-derived reference measure.
  Q4IntegrationResult integration;
  Q4PlanarIntegrationBackend integration_backend = Q4PlanarIntegrationBackend::ScalarDyadicSquares;
  // Maximum depth per natural axis; both equal integration.deepest_leaf for
  // scalar squares. Uncovered parents retain zero depths and invalid integral.
  std::uint32_t deepest_u = 0, deepest_v = 0;
};
struct Q4PlanarContactDiagnostics {
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  std::uint64_t configuration_id = 0, wall_binding_id = 0;
  Q4PlanarContactPhase phase = Q4PlanarContactPhase::Unspecified;
  bool valid = false;
  std::uint32_t parent_count = 0, covered_count = 0;
  std::uint32_t leaves = 0, visited = 0, deepest_leaf = 0;
  Q4IntegralInterval active_area;
  Q4CertifiedIntegral potential;
  Vec3 force_on_surface, wall_reaction, wall_moment;
  Vec3 force_error, wall_moment_error;
  double surface_power = 0, surface_power_error = 0;
  double maximum_penetration = 0;
  double stiffness_rate_bound = 0;
  // Candidate interval work uses this module's retained BASE numerical force.
  // No physical/artificial kinetic energy is owned or counted a second time.
  double base_potential = 0, base_potential_error = 0;
  double potential_increment = 0;
  double kinetic_midpoint_work = 0, kinetic_midpoint_roundoff = 0;
  double force_coordinate_work = 0, force_coordinate_roundoff = 0;
  double conservative_force_coordinate_defect = 0;
  double continuum_work_uncertainty = 0, quadratic_work_upper = 0;
  Q4PlanarIntegrationBackend integration_backend = Q4PlanarIntegrationBackend::ScalarDyadicSquares;
  // Maxima across covered parents, not binary-tree path lengths.
  std::uint32_t deepest_u = 0, deepest_v = 0;
};

// Finite-wall Q4 contribution, at most two parents/eight incident physical
// nodes. Startup copies geometry, source identities, and incident inverse
// masses/constraints; each Assemble checks those against the actual owner.
// One selected C2 scratch array is reused serially for parents and base/candidate
// evaluation. Common control/heap and selected leaves share a single allocation;
// complete module device storage is <=1 MiB. No runtime backend fallback occurs.
// The original checked wall metadata is retained on the host for source binding;
// full footprint coverage plus fixed Y/Z removes per-Gauss wall searches.
//
// This component owns no physical x/v/q state, accepted history, clock, stream,
// advance or commit. Orientations do not enter zero-offset normal contact and
// direct contact couples are zero. The owner/coordinator provides rotations,
// shell inertia/energy, one reset, additive contributors, and combined admission.
// Legacy translational stability rows remain untouched.
//
// All parents, certificates and additive-overflow checks finish before any
// physical assembly write. A failure marks an otherwise-valid assembly failed;
// the caller must Discard after EVERY unsuccessful report. Candidate evaluation
// never changes the stored base force or potential. Result scratch is invalid
// after any failed/intervening evaluation. Caller outputs stay unchanged on
// failure. Calls serialize and finish on the borrowed owner stream. A detected
// CUDA error poisons this batch, with no claim of owner/context recovery.
class Q4PlanarContact {
 public:
  Q4PlanarContact();
  ~Q4PlanarContact();
  Q4PlanarContact(const Q4PlanarContact&) = delete;
  Q4PlanarContact& operator=(const Q4PlanarContact&) = delete;
  Q4PlanarContactReport Initialize(const Q4PlanarContactConfig&, PlanarWallView,
                                    const Q4SurfaceView&, const Q4FixedYZMassView&);
  Q4PlanarContactReport Assemble(const tl::fea::NodalAssemblyView&, Q4PlanarContactDiagnostics*);
  Q4PlanarContactReport EvaluateCandidate(const tl::fea::NodalPreparedView&, Q4PlanarContactDiagnostics*);
  // Output cadence only. Stage before commit and publish only after that commit.
  // For recovery after rejected scratch, reevaluate accepted state in a fresh
  // temporary assembly, copy results, then Discard without advancing its clock.
  Q4PlanarContactReport CopyParentResults(const Q4PlanarContactDiagnostics& expected,
                                         Q4PlanarParentResult*, std::size_t capacity);
  tl::fea::NodalAllocationInfo allocations() const noexcept;
  double stiffness_rate_bound() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace tlfea::contact
