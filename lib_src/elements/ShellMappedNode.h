// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchFields.h"
#include "ShellBatchStartup.h"

#if defined(__CUDACC__)
#define TL_SHELL_MAPPED_HD __host__ __device__
#else
#define TL_SHELL_MAPPED_HD
#endif
namespace tl::fea::shell_mapped_detail {
template<class Model>
TL_SHELL_MAPPED_HD bool ValidNode(const Model& model,const NodalAssemblyView& view,
    std::size_t node,bool initial) noexcept {
  if (node>=model.config.owner.node_count || view.mass.fixed[node] ||
      view.translation_fixed_bits[node] || view.rotation_fixed[node] ||
      (view.rotation_present && view.rotation_present[node]!=1)) return false;
  const double inverse_mass=view.mass.inverse_mass[node];
  const double inverse_inertia=view.inverse_inertia[node];
  if (!tl::math::Finite(inverse_mass) || inverse_mass<0 ||
      !tl::math::Finite(inverse_inertia) || inverse_inertia<0) return false;
  const auto position=shell_batch_fields::ReadVector(view.accepted.position_xyz,node);
  const auto velocity=shell_batch_fields::ReadVector(view.accepted.velocity_xyz,node);
  const auto omega=shell_batch_fields::ReadVector(view.accepted.angular_velocity_xyz,node);
  const auto* q=view.accepted.orientation_wxyz+4*node;
  return shell_batch_fields::FiniteVector(position) && shell_batch_fields::FiniteVector(velocity) &&
      shell_batch_fields::FiniteVector(omega) && tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]}) &&
      (!initial || shell_startup_detail::MatchesInitialNode(model.config.startup,position,
          model.initial_position[node],velocity,omega,q));
}
} // namespace tl::fea::shell_mapped_detail
#undef TL_SHELL_MAPPED_HD
