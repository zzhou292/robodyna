// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidGroupStorage.h"
#include <cstring>

namespace tl::fea::nodal_detail {
NodalReport ValidateRigidAssemblyOwner(const NodalRigidAssemblyBinding& binding,
    HostNodalKinematicsView input, const double* inverse_mass, const NodalDofConfig& dofs,
    bool cin) noexcept {
  const auto nodes = binding.domain()->nodes();
  const auto coefficients = binding.coefficients()->nodes();
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const auto& value = coefficients[i].coefficients;
    const double xyz[]{nodes[i].position.x,nodes[i].position.y,nodes[i].position.z};
    for (unsigned axis = 0; axis < 3; ++axis) {
      if (std::memcmp(&xyz[axis],input.position_xyz+3*i+axis,sizeof(double)) != 0)
        return {NodalStatus::InvalidInput,"Owner position differs from complete rigid source domain",std::uint32_t(i)};
    }
    const bool absent_rotation = dofs.rotation_present && !dofs.rotation_present[i];
    // CIN's exact dependent roster is validated separately before allocation.
    // A zero inverse alone is never sufficient for ordinary role admission.
    const double expected_mass = dofs.translation_fixed_bits[i] == 7 ||
        (cin && inverse_mass[i] == 0) || value.mass == 0 ? 0 : 1/value.mass;
    const double expected_inertia = dofs.rotation_fixed[i] || absent_rotation ||
        (cin && dofs.inverse_inertia[i] == 0) || value.isotropic_inertia == 0
        ? 0 : 1/value.isotropic_inertia;
    if (inverse_mass[i] != expected_mass || dofs.inverse_inertia[i] != expected_inertia ||
        (absent_rotation && value.isotropic_inertia != 0))
      return {NodalStatus::InvalidInput,"Owner inverses differ from complete rigid coefficient ledger",std::uint32_t(i)};
  }
  return {NodalStatus::Ok,"Complete rigid owner source association validated"};
}
} // namespace tl::fea::nodal_detail
