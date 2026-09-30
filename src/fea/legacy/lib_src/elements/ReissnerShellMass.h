#pragma once

// Row-sum mass of one centered uniform Reissner layer. The physical director
// inertia follows the independent thickness integral integral(rho*z*z dz),
// not Chrono's quarter-tile box-inertia heuristic. No state, assembly, step or
// material history is owned here. Existing fixed-size frame/math values are
// reused for checked host/device arithmetic.
#include "ReissnerShellData.h"
#include "ReissnerRotation.h"

#if defined(__CUDACC__)
#define TL_SHELL_MASS_HD __host__ __device__
#else
#define TL_SHELL_MASS_HD
#endif

namespace tl::fea::reissner {

enum class ShellDrillingInertiaPolicy {
  kNone,  // Physical rank-two tensor; not an unconstrained free-spin integrator.
  kEqualPhysicalTangential  // Explicit coupon regularization J_d = J.
};
enum class ShellMassStatus {
  kSuccess, kInvalidReference, kInvalidSection, kUnsupportedPolicy,
  kInvalidMass, kInvalidKinematics, kNonfiniteResult
};
struct ShellNodalMass {
  double area = 0;                         // integral(N_i dA), square metres.
  double mass = 0;                         // rho * thickness * area, kg.
  double physical_tangential_inertia = 0;  // rho * thickness^3 * area / 12, kg m^2.
  double artificial_drilling_inertia = 0;  // Numerical, never physical mass.
};
struct ShellMass {
  ShellNodalMass node[4]{};
  ShellDrillingInertiaPolicy drilling_policy = ShellDrillingInertiaPolicy::kNone;
};
struct ShellInertiaTensor {
  Matrix3 physical;    // J * (I - d*d^T), WORLD coordinates.
  Matrix3 artificial;  // J_d * d*d^T, WORLD coordinates.
};
struct ShellKineticEnergy {
  double translation = 0;
  double physical_rotation = 0;
  double artificial_drilling = 0;
};

namespace mass_detail {
TL_SHELL_MASS_HD inline bool ValidPolicy(ShellDrillingInertiaPolicy policy) {
  return policy == ShellDrillingInertiaPolicy::kNone ||
         policy == ShellDrillingInertiaPolicy::kEqualPhysicalTangential;
}
TL_SHELL_MASS_HD inline bool ValidMass(const ShellNodalMass& node) {
  return detail::Finite(node.area) && node.area > 0 &&
         detail::Finite(node.mass) && node.mass > 0 &&
         detail::Finite(node.physical_tangential_inertia) && node.physical_tangential_inertia > 0 &&
         detail::Finite(node.artificial_drilling_inertia) && node.artificial_drilling_inertia >= 0;
}
TL_SHELL_MASS_HD inline bool UnitDirector(Vec3 director) {
  return detail::Finite(director) && ::fabs(detail::Dot(director, director) - 1) <= 1e-10;
}
}  // namespace mass_detail

// Caller supplies immutable setup-adapter-prepared data in one memory space.
// Validate the mass-bearing tables, not the unrelated elastic stiffness/ANS
// fields: prepared is a default-data guard, not authentication of edited data
// or a replacement for rectangle/material admission. A malformed shape row or
// unrepresentable positive mass/inertia is rejected. No geometry is repaired.
// Policy is mandatory; artificial drilling inertia is never silently enabled.
// Inputs and output must not overlap. Failure leaves the whole output unchanged.
TL_SHELL_MASS_HD inline ShellMassStatus ComputeShellMass(
    const ShellReference& reference, const ElasticSection& section,
    ShellDrillingInertiaPolicy policy, ShellMass& output) {
  if (!reference.prepared) return ShellMassStatus::kInvalidReference;
  if (!section.prepared || !detail::Finite(section.thickness) || section.thickness <= 0 ||
      !detail::Finite(section.density) || section.density <= 0) return ShellMassStatus::kInvalidSection;
  if (!mass_detail::ValidPolicy(policy)) return ShellMassStatus::kUnsupportedPolicy;
  ShellMass candidate;
  candidate.drilling_policy = policy;
  for (const auto& point : reference.gauss) {
    if (!detail::Finite(point.area_weight) || point.area_weight <= 0)
      return ShellMassStatus::kInvalidReference;
    double partition = 0;
    for (unsigned n = 0; n < 4; ++n) {
      if (!detail::Finite(point.shape[n]) || point.shape[n] < 0 || point.shape[n] > 1)
        return ShellMassStatus::kInvalidReference;
      partition += point.shape[n];
      candidate.node[n].area += point.shape[n] * point.area_weight;
    }
    if (::fabs(partition - 1) > 1e-12) return ShellMassStatus::kInvalidReference;
  }
  for (auto& node : candidate.node) {
    node.mass = section.density * section.thickness * node.area;
    node.physical_tangential_inertia = node.mass * section.thickness * section.thickness / 12;
    node.artificial_drilling_inertia = policy == ShellDrillingInertiaPolicy::kEqualPhysicalTangential
                                           ? node.physical_tangential_inertia : 0;
    if (!mass_detail::ValidMass(node)) return ShellMassStatus::kNonfiniteResult;
  }
  output = candidate;
  return ShellMassStatus::kSuccess;
}

// director is the unit WORLD physical shell director, including the immutable
// right frame offset; it is not necessarily the nodal quaternion's local z.
// No director normalization occurs. The caller serializes writes and supplies
// nonoverlapping input/output objects. J_d = J yields isotropic total J*I;
// keeping its two terms separate preserves the physical/artificial ledger.
TL_SHELL_MASS_HD inline ShellMassStatus ComputeShellInertiaTensor(
    const ShellNodalMass& mass, Vec3 director, ShellInertiaTensor& output) {
  if (!mass_detail::ValidMass(mass)) return ShellMassStatus::kInvalidMass;
  if (!mass_detail::UnitDirector(director)) return ShellMassStatus::kInvalidKinematics;
  const auto dyad = detail::Outer(director, director);
  ShellInertiaTensor candidate;
  candidate.physical = detail::Scale(detail::Subtract(detail::Identity(), dyad), mass.physical_tangential_inertia);
  candidate.artificial = detail::Scale(dyad, mass.artificial_drilling_inertia);
  if (!detail::Finite(candidate.physical) || !detail::Finite(candidate.artificial))
    return ShellMassStatus::kNonfiniteResult;
  output = candidate;
  return ShellMassStatus::kSuccess;
}

// Diagnostic for one element's row-sum contribution. It excludes translational
// orbital inertia from director inertia: orbital motion is already in v_i.
// v/omega are WORLD velocity-level values and all four directors must be unit.
// A global ledger sums each element contribution once OR sums assembled nodal
// mass/tensors once; never sum both. At sharp seams, scalar physical J values
// cannot share one director unless that convention is actually common: retain
// contributor tensors/energies instead. This helper does not authorize such a
// shared rotational topology. No input pointers are retained; failure preserves
// all output fields, including when the last node fails validation.
TL_SHELL_MASS_HD inline ShellMassStatus ComputeShellKineticEnergy(
    const ShellMass& mass, const Vec3 director[4], const Vec3 velocity[4],
    const Vec3 angular_velocity[4], ShellKineticEnergy& output) {
  if (!director || !velocity || !angular_velocity) return ShellMassStatus::kInvalidKinematics;
  if (!mass_detail::ValidPolicy(mass.drilling_policy)) return ShellMassStatus::kUnsupportedPolicy;
  ShellKineticEnergy candidate;
  for (unsigned n = 0; n < 4; ++n) {
    const auto& node = mass.node[n];
    if (!mass_detail::ValidMass(node)) return ShellMassStatus::kInvalidMass;
    const double expected_drilling = mass.drilling_policy == ShellDrillingInertiaPolicy::kEqualPhysicalTangential
                                        ? node.physical_tangential_inertia : 0;
    if (node.artificial_drilling_inertia != expected_drilling) return ShellMassStatus::kInvalidMass;
    if (!mass_detail::UnitDirector(director[n]) || !detail::Finite(velocity[n]) ||
        !detail::Finite(angular_velocity[n])) return ShellMassStatus::kInvalidKinematics;
    const auto tangent_speed = detail::Cross(angular_velocity[n], director[n]);
    const double drilling_speed = detail::Dot(angular_velocity[n], director[n]);
    candidate.translation += .5 * node.mass * detail::Dot(velocity[n], velocity[n]);
    candidate.physical_rotation += .5 * node.physical_tangential_inertia * detail::Dot(tangent_speed, tangent_speed);
    candidate.artificial_drilling += .5 * node.artificial_drilling_inertia * drilling_speed * drilling_speed;
  }
  if (!detail::Finite(candidate.translation) || !detail::Finite(candidate.physical_rotation) ||
      !detail::Finite(candidate.artificial_drilling)) return ShellMassStatus::kNonfiniteResult;
  output = candidate;
  return ShellMassStatus::kSuccess;
}

}  // namespace tl::fea::reissner
#undef TL_SHELL_MASS_HD
