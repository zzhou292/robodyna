#pragma once

#include "Q4SurfaceMapping.h"
#include "SurfaceContactMass.h"

namespace tlfea::contact {

// Narrow, explicitly constrained normal subspace. Borrowed physical inverse
// masses retain their units; mask 6 fixes Y/Z while mask 7 fixes all translations.
// This is not a relabelled unrestricted LumpedTranslationMassView and carries
// no rotational inertia. The later batch validates actual owner provenance.
struct Q4FixedYZMassView {
  const double* inverse_mass = nullptr;
  const std::uint8_t* translation_fixed_bits = nullptr;
  std::uint32_t node_count = 0;
  std::uint64_t base_epoch = 0;
};

// Stationary -X wall: J_i=(-N_i,0,0), inverse_effective_mass=sum_free_X N_i^2/m_i.
// Uses existing NormalJacobian layout and conservative upward normalization.
// Parent node IDs are unique and sorted for mass accumulation; force projection
// separately retains natural order. Zero-weight nodes are still validated.
// No output writes on failure, including no supported dynamic normal DOFs.
// No artificial positive inverse mass, force law, timestep or global row bound
// is supplied. Inputs/output must not overlap and occupy one memory space.
TL_SURFACE_HD inline Status BuildQ4NormalXJacobian(
    const Q4FixedYZMassView& mass, const SurfaceQ4& parent, double u, double v,
    std::uint64_t attempt, NormalJacobian* output) {
  if (!output || !mass.inverse_mass || !mass.translation_fixed_bits || !attempt)
    return Status::kInvalidArgument;
  auto status = q4_detail::ValidateParent(parent, mass.node_count);
  if (status != Status::kOk) return status;
  double shape[4];
  status = EvaluateQ4Shape(u, v, shape);
  if (status != Status::kOk) return status;
  unsigned order[4] = {0, 1, 2, 3};
  for (unsigned n = 0; n < 4; ++n) {
    const auto node = parent.nodes[n];
    const auto bits = mass.translation_fixed_bits[node];
    const double inverse = mass.inverse_mass[node];
    if (bits != 6 && bits != 7) return Status::kUnsupportedInterpolation;
    if (!IsFinite(inverse) || (bits == 7 ? inverse != 0 : inverse <= 0)) return Status::kInvalidArgument;
    for (unsigned i = n; i > 0 && parent.nodes[order[i]] < parent.nodes[order[i-1]]; --i) {
      const auto previous = order[i-1]; order[i-1] = order[i]; order[i] = previous;
    }
  }
  NormalJacobian candidate;
  candidate.base_epoch = mass.base_epoch; candidate.attempt = attempt;
  for (unsigned i = 0; i < 4; ++i) {
    const unsigned local = order[i];
    const double weight = shape[local];
    if (weight == 0) continue;
    const auto node = parent.nodes[local];
    const double inverse = mass.inverse_mass[node], root = ::sqrt(inverse);
    const double normalized = weight * root, term = normalized * normalized;
    if (!IsFinite(normalized) || !IsFinite(term) || (inverse > 0 && (normalized == 0 || term == 0)))
      return Status::kNonFiniteResult;
    candidate.inverse_effective_mass += term;
    double padded_root = 0, padded_norm = 0;
    if (!IsFinite(candidate.inverse_effective_mass) || !mass_detail::Upper(root, &padded_root) ||
        !mass_detail::UpperProduct(weight, padded_root, &padded_norm)) return Status::kNonFiniteResult;
    const auto entry = candidate.count++;
    candidate.nodes[entry] = node; candidate.values[entry] = {-weight, 0, 0};
    candidate.normalized_norm[entry] = padded_norm;
  }
  if (candidate.inverse_effective_mass == 0) return Status::kNoDynamicDofs;
  candidate.valid = true; *output = candidate;
  return Status::kOk;
}

}  // namespace tlfea::contact
