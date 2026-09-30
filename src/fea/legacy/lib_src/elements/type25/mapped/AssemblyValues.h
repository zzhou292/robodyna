// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Stiffness.h"
#include "../Type25BatchArena.h"
#include "../../ShellBatchFields.h"
#include "../../mapped_connector/Values.h"
#include "../../../solvers/NodalCinRuntime.h"
#if defined(__CUDACC__)
#define TL_CONNECTOR25_HD __host__ __device__
#else
#define TL_CONNECTOR25_HD
#endif
namespace tl::fea::type25::batch_detail {
TL_CONNECTOR25_HD inline bool MappedNode(const DeviceModel& model, const NodalAssemblyView& view,
    std::size_t node, bool initial) {
  if (node >= model.config.owner.node_count || view.mass.fixed[node] ||
      view.translation_fixed_bits[node] || view.rotation_fixed[node] ||
      (view.rotation_present && view.rotation_present[node] != 1)) return false;
  const double inverse_mass = view.mass.inverse_mass[node];
  const double inverse_inertia = view.inverse_inertia[node];
  if (!tl::math::Finite(inverse_mass) || inverse_mass < 0 ||
      !tl::math::Finite(inverse_inertia) || inverse_inertia < 0) return false;
  const auto position = shell_batch_fields::ReadVector(view.accepted.position_xyz, node);
  const auto velocity = shell_batch_fields::ReadVector(view.accepted.velocity_xyz, node);
  const auto omega = shell_batch_fields::ReadVector(view.accepted.angular_velocity_xyz, node);
  const auto* q = view.accepted.orientation_wxyz + 4 * node;
  return tl::math::fixed3::Finite(position) && tl::math::fixed3::Finite(velocity) && tl::math::fixed3::Finite(omega) &&
      tl::math::UnitQuaternion({q[0], q[1], q[2], q[3]}) &&
      (!initial || shell_startup_detail::MatchesInitialFreePhysicalNode(model.config.startup, position,
          model.nodes[node].reference, velocity, omega, q));
}
struct AssemblyFamily {
  using Storage = batch_detail::Storage;
  using Accepted = const Slab*;
  static constexpr auto Ordering = mapped_connector::Order::ParentThenAssembly;
  TL_CONNECTOR25_HD static auto Success() { return BatchStatus::Success; }
  TL_CONNECTOR25_HD static auto AssemblyFailure() { return BatchStatus::AssemblyFailure; }
  TL_CONNECTOR25_HD static std::size_t Count(const Storage& state) { return state.model.config.element_count; }
  TL_CONNECTOR25_HD static auto Status(mapped_connector::Failure failure) {
    return failure == mapped_connector::Failure::Endpoint ? BatchStatus::InvalidInput :
        failure == mapped_connector::Failure::Coefficient ? BatchStatus::NonfiniteResult : BatchStatus::AssemblyFailure;
  }
  TL_CONNECTOR25_HD static mapped_connector::Parent Prepare(const Storage& state, const Slab* accepted,
      std::size_t parent, NodalAssemblyView view, NodalCinAssemblyView cin, bool initial) {
    using namespace mapped_connector;
    Parent next;
    const auto& element = state.model.elements[parent];
    for (auto node : element.nodes) if (!MappedNode(state.model, view, node, initial)) {
      next.failure = Failure::Endpoint; next.node = node; return next;
    }
    mapped::NodalStiffness stiffness;
    if (!mapped::AcceptedStiffness(state.model.properties[element.property_index], accepted->element[parent], stiffness))
      next.failure = Failure::Coefficient;
    else if (!ForceViewValid(view.forces, element.nodes) ||
        !StiffnessViewValid(cin.translational_stiffness, cin.rotational_stiffness, cin.node_count, element.nodes))
      next.failure = Failure::Assembly;
    next.translation = stiffness.translation; next.rotation = stiffness.rotation;
    return next;
  }
  struct Access {
    const Evaluation* accepted;
    TL_CONNECTOR25_HD bool Add(std::size_t parent, unsigned slot, const mapped_connector::Parent& record,
        mapped_shell::AssemblyNode& next) const {
      return mapped_connector::AddEndpoint<false>(record, accepted[parent], slot, next);
    }
  };
  TL_CONNECTOR25_HD static Access Values(const Storage&, const Slab* accepted) { return {accepted->element}; }
};
} // namespace tl::fea::type25::batch_detail
#undef TL_CONNECTOR25_HD
