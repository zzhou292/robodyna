// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"
#include "../../../solvers/NodalCinRuntime.h"
#include "Kinematics.h"
#include "../../ShellBatchFields.h"
#include "../../mapped_connector/Values.h"
namespace tl::fea::type13::batch_detail {
TL_TYPE13_HD inline bool ValidEndpoint(const DeviceModel& model, const DeviceElement& element,
    unsigned local, const NodalAssemblyView& view, bool initial, bool mapped) {
  const auto node = element.nodes[local];
  if (mapped && (node >= model.config.owner.node_count ||
      view.mass.fixed[node] || view.translation_fixed_bits[node] || view.rotation_fixed[node] ||
      (view.rotation_present && view.rotation_present[node] != 1) ||
      !detail::Nonnegative(view.mass.inverse_mass[node]) ||
      !detail::Nonnegative(view.inverse_inertia[node]))) return false;
  const auto x = shell_batch_fields::ReadVector(view.accepted.position_xyz, node);
  const auto v = shell_batch_fields::ReadVector(view.accepted.velocity_xyz, node);
  const auto w = shell_batch_fields::ReadVector(view.accepted.angular_velocity_xyz, node);
  const auto* q = view.accepted.orientation_wxyz + 4 * node;
  return tl::math::fixed3::Finite(x) && tl::math::fixed3::Finite(v) && tl::math::fixed3::Finite(w) &&
      tl::math::UnitQuaternion({q[0], q[1], q[2], q[3]}) &&
      (!initial || (mapped ? shell_startup_detail::MatchesInitialFreePhysicalNode(
          model.config.startup, x, element.reference.position_m[local], v, w, q) :
          shell_startup_detail::MatchesInitialNode(
          model.config.startup, x, element.reference.position_m[local], v, w, q)));
}
struct AssemblyFamily {
  using Storage = batch_detail::Storage;
  using Accepted = unsigned;
  static constexpr auto Ordering = mapped_connector::Order::AllEndpointsFirst;
  TL_TYPE13_HD static auto Success() { return BatchStatus::Success; }
  TL_TYPE13_HD static auto AssemblyFailure() { return BatchStatus::AssemblyFailure; }
  TL_TYPE13_HD static std::size_t Count(const Storage& state) { return state.model.element_count; }
  TL_TYPE13_HD static auto Status(mapped_connector::Failure failure) {
    return failure == mapped_connector::Failure::Endpoint ? BatchStatus::InvalidInput : BatchStatus::AssemblyFailure;
  }
  TL_TYPE13_HD static mapped_connector::Parent Prepare(const Storage& state, unsigned accepted,
      std::size_t parent, NodalAssemblyView view, NodalCinAssemblyView cin, bool initial) {
    using namespace mapped_connector;
    Parent next;
    const auto& element = state.model.elements[parent];
    for (unsigned local = 0; local < 2; ++local) if (!ValidEndpoint(state.model, element, local, view, initial, true)) {
      next.failure = Failure::Endpoint; next.node = element.nodes[local]; return next;
    }
    if (!EndpointStiffness(state.slab[accepted][parent], next.translation, next.rotation))
      next.failure = Failure::Coefficient;
    else if (!StiffnessViewValid(cin.translational_stiffness, cin.rotational_stiffness, cin.node_count, element.nodes) ||
        !ForceViewValid(view.forces, element.nodes)) next.failure = Failure::Assembly;
    return next;
  }
  struct Access {
    const Evaluation* accepted;
    TL_TYPE13_HD bool Add(std::size_t parent, unsigned slot, const mapped_connector::Parent& record,
        mapped_shell::AssemblyNode& next) const {
      return mapped_connector::AddEndpoint<true>(record, accepted[parent], slot, next);
    }
  };
  TL_TYPE13_HD static Access Values(const Storage& state, unsigned accepted) { return {state.slab[accepted]}; }
};
} // namespace tl::fea::type13::batch_detail
