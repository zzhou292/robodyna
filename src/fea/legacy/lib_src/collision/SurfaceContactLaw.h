#pragma once

#include "SurfaceContactTypes.h"

namespace tlfea {
namespace contact {

struct NormalContactParameters {
  double stiffness = 0;       // Local, already weighted tangent stiffness N/m.
  double damping_ratio = 0;   // c = 2*zeta*sqrt(k*m_eff); dimensionless.
  double timestep_safety = 0.8;  // Strictly between zero and one.
};

struct NormalContactInput {
  double gap = 0;             // Negative means overlap, including shell offsets.
  double normal_velocity = 0; // (v_A-v_B).n; negative means approaching.
  double inverse_effective_mass = 0;  // J_relative M^-1 J_relative^T, kg^-1.
};

struct NormalContactResponse {
  double force = 0;                  // Repulsive magnitude; +n on A, -n on B.
  double elastic_energy = 0;         // Joules, 0.5*k*penetration^2.
  double dissipated_power = 0;       // Watts, nonnegative dashpot energy loss.
  double damping_coefficient = 0;    // N*s/m.
  double stable_timestep = 0;        // Seconds; LOCAL linear-mode bound only.
  bool active = false;
};

// Shared force/energy arithmetic after the caller resolves damping. The zero
// damping physical-wall profile does not require a fabricated effective mass.
namespace normal_contact_detail {
TL_SURFACE_HD inline Status ApplyPenalty(double stiffness, double gap,
    double normal_velocity, NormalContactResponse& response) {
  if (gap <= 0) {
    response.active = true;
    const double penetration = -gap;
    const double elastic_force = stiffness * penetration;
    const double raw_force = elastic_force -
                             response.damping_coefficient * normal_velocity;
    if (!IsFinite(raw_force) || !IsFinite(elastic_force))
      return Status::kNonFiniteResult;
    response.force = raw_force > 0 ? raw_force : 0;
    response.elastic_energy = 0.5 * elastic_force * penetration;
    // Includes the unilateral clamp. During fast separation the dashpot can
    // cancel the spring but can never make the normal resultant attractive.
    response.dissipated_power = -(response.force - elastic_force) *
                                normal_velocity;
    if (!IsFinite(response.elastic_energy) || !IsFinite(response.dissipated_power))
      return Status::kNonFiniteResult;
  }
  return Status::kOk;
}
} // namespace normal_contact_detail

// The unilateral Hooke/dashpot convention follows Chrono ChContactSMC.cpp:
// max(0, k*penetration - c*normal_velocity). This function owns no geometry,
// dynamics, friction, history, or independent simulation clock.
//
// Both endpoints with disjoint DOFs contribute to inverse_effective_mass. A
// fixed wall contributes zero. Shared DOFs require assembling the relative
// Jacobian BEFORE squaring; summing endpoint inverse masses would be incorrect.
// Consistent/generalized FE mass requires the appropriate mass solve.
TL_SURFACE_HD inline Status EvaluateNormalContact(
    const NormalContactParameters& parameters, const NormalContactInput& input,
    NormalContactResponse* out) {
  if (!out)
    return Status::kInvalidArgument;
  *out = {};
  if (!IsFinite(parameters.stiffness) || parameters.stiffness <= 0 ||
      !IsFinite(parameters.damping_ratio) || parameters.damping_ratio < 0 ||
      !IsFinite(parameters.timestep_safety) || parameters.timestep_safety <= 0 ||
      parameters.timestep_safety >= 1 || !IsFinite(input.gap) ||
      !IsFinite(input.normal_velocity) || !IsFinite(input.inverse_effective_mass) ||
      input.inverse_effective_mass < 0)
    return Status::kInvalidArgument;
  if (input.inverse_effective_mass == 0)
    return Status::kNoDynamicDofs;

  const double omega = ::sqrt(parameters.stiffness) *
                       ::sqrt(input.inverse_effective_mass);
  if (!IsFinite(omega) || omega <= 0)
    return Status::kNonFiniteResult;
  const double zeta = parameters.damping_ratio;
  // hypot avoids squaring/overflow for large damping. This is the stability
  // limit of velocity-first (symplectic) Euler with velocity-explicit damping:
  // h*omega < 2/(sqrt(1+zeta^2)+zeta). Other integrators must provide their bound.
  // Multiple contacts/internal stiffness can tighten the GLOBAL bound; a min
  // of isolated pair bounds alone does not establish coupled-system stability.
  const double damping_factor = ::hypot(1.0, zeta) + zeta;
  NormalContactResponse response;
  response.stable_timestep = (2.0 * parameters.timestep_safety / damping_factor) / omega;
  response.damping_coefficient = (2.0 * zeta * ::sqrt(parameters.stiffness)) /
                                 ::sqrt(input.inverse_effective_mass);
  if (!IsFinite(response.stable_timestep) || response.stable_timestep <= 0 ||
      !IsFinite(response.damping_coefficient))
    return Status::kNonFiniteResult;
  const auto penalty = normal_contact_detail::ApplyPenalty(parameters.stiffness,
      input.gap, input.normal_velocity, response);
  if (penalty != Status::kOk) return penalty;
  *out = response;
  return Status::kOk;
}

// Minimal accepted/trial contract for normal-contact diagnostics. These PODs
// demonstrate one accepted physical step and safe repeated trial evaluation.
// They are NOT a tangential friction-history implementation; C2 must add its
// objectively transported stick/slip state under this acceptance contract.
// Storage is owned per stable contact ID by the future contact backend. All
// trials for one point must use that point's committed state; no concurrent
// commit or cross-contact mixing is allowed.
struct NormalContactState {
  std::uint64_t revision = 0;
  double accepted_time = 0;
  double dissipated_energy = 0;
};

struct NormalContactTrial {
  NormalContactState candidate;
  std::uint64_t base_revision = 0;
  bool valid = false;
};

TL_SURFACE_HD inline Status MakeNormalContactTrial(
    const NormalContactState& committed, const NormalContactResponse& response,
    double dt, NormalContactTrial* out) {
  if (!out)
    return Status::kInvalidArgument;
  *out = {};
  if (!IsFinite(dt) || dt <= 0 || !IsFinite(committed.accepted_time) ||
      committed.accepted_time < 0 || !IsFinite(committed.dissipated_energy) ||
      committed.dissipated_energy < 0 || !IsFinite(response.dissipated_power) ||
      response.dissipated_power < 0 || committed.revision == UINT64_MAX)
    return Status::kInvalidArgument;
  NormalContactTrial trial;
  trial.base_revision = committed.revision;
  trial.candidate.revision = committed.revision + 1;
  trial.candidate.accepted_time = committed.accepted_time + dt;
  // First-order diagnostic quadrature, not an exact energy-conserving update.
  trial.candidate.dissipated_energy = committed.dissipated_energy +
                                      dt * response.dissipated_power;
  if (!IsFinite(trial.candidate.accepted_time) ||
      !IsFinite(trial.candidate.dissipated_energy))
    return Status::kNonFiniteResult;
  trial.valid = true;
  *out = trial;
  return Status::kOk;
}

TL_SURFACE_HD inline Status CommitNormalContactTrial(
    NormalContactState* committed, NormalContactTrial* trial) {
  if (!committed || !trial)
    return Status::kInvalidArgument;
  if (!trial->valid)
    return Status::kNoTrial;
  if (committed->revision != trial->base_revision)
    return Status::kStaleTrial;
  *committed = trial->candidate;
  trial->valid = false;
  return Status::kOk;
}

TL_SURFACE_HD inline void DiscardNormalContactTrial(NormalContactTrial* trial) {
  if (trial)
    trial->valid = false;
}

}  // namespace contact
}  // namespace tlfea
