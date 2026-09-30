// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../SurfaceJacobianMajorantValues.h"

namespace tlfea::contact::penalty_pair_detail {
inline TL_SURFACE_HD SurfacePenaltyStatus MappingStatus(Status status) {
  if (status == Status::kOk) return SurfacePenaltyStatus::Ok;
  if (status == Status::kOutOfRange) return SurfacePenaltyStatus::OutOfRange;
  if (status == Status::kNonFiniteResult) return SurfacePenaltyStatus::NonFiniteResult;
  return SurfacePenaltyStatus::InvalidInput;
}
inline TL_SURFACE_HD SurfacePenaltyStatus MajorantStatus(SurfaceMajorantStatus status) {
  if (status == SurfaceMajorantStatus::Ok) return SurfacePenaltyStatus::Ok;
  if (status == SurfaceMajorantStatus::OutOfRange) return SurfacePenaltyStatus::OutOfRange;
  if (status == SurfaceMajorantStatus::InconsistentMask) return SurfacePenaltyStatus::InconsistentMask;
  if (status == SurfaceMajorantStatus::Unrepresentable) return SurfacePenaltyStatus::Unrepresentable;
  return SurfacePenaltyStatus::InvalidInput;
}
inline TL_SURFACE_HD SurfacePenaltyStatus ValidateEndpoint(
    const SurfacePenaltyEndpoint& endpoint, std::uint32_t node_count) {
  if (!IsFinite(endpoint.reference_half_thickness_m) || endpoint.reference_half_thickness_m < 0)
    return SurfacePenaltyStatus::InvalidInput;
  const auto status = ValidateWeightedSurfacePoint(endpoint.point, node_count);
  if (status != Status::kOk) return MappingStatus(status);
  for (std::uint32_t i = 0; i < endpoint.point.count; ++i)
    if (endpoint.translation_fixed_bits[i] > 7) return SurfacePenaltyStatus::InvalidInput;
  return SurfacePenaltyStatus::Ok;
}

// Rounded per-occurrence J rows, A source slots then B source slots. Do not
// merge scalar weights first: that is a different represented operator.
inline TL_SURFACE_HD SurfacePenaltyStatus AppendTerms(const SurfacePenaltyEndpoint& endpoint,
    Vec3 signed_normal, RepresentedJacobianTerm* terms, std::uint32_t& count) {
  for (std::uint32_t i = 0; i < endpoint.point.count; ++i) {
    auto& term = terms[count++];
    term.node = endpoint.point.nodes[i];
    term.translation_fixed_bits = endpoint.translation_fixed_bits[i];
    const double weight = endpoint.point.weights[i];
    term.value = Scale(signed_normal, weight);
    if (!surface_majorant::RepresentedProduct(signed_normal.x, weight, term.value.x) ||
        !surface_majorant::RepresentedProduct(signed_normal.y, weight, term.value.y) ||
        !surface_majorant::RepresentedProduct(signed_normal.z, weight, term.value.z))
      return SurfacePenaltyStatus::Unrepresentable;
  }
  return SurfacePenaltyStatus::Ok;
}

inline TL_SURFACE_HD SurfacePenaltyStatus MergeForces(const SurfacePenaltyEndpoint& endpoint,
    const WeightedNodalForces& forces, SurfacePenaltyPacket& packet) {
  for (std::uint32_t i = 0; i < forces.count; ++i) {
    const auto index = surface_majorant::FindNode(packet.nodes, packet.count, forces.nodes[i]);
    if (index == packet.count) {
      auto& node = packet.nodes[packet.count++];
      node.node = forces.nodes[i];
      node.translation_fixed_bits = endpoint.translation_fixed_bits[i];
      node.force_n = forces.forces[i];
    } else {
      auto& node = packet.nodes[index];
      if (node.translation_fixed_bits != endpoint.translation_fixed_bits[i])
        return SurfacePenaltyStatus::InconsistentMask;
      node.force_n = Add(node.force_n, forces.forces[i]);
      if (!IsFinite(node.force_n)) return SurfacePenaltyStatus::NonFiniteResult;
    }
  }
  return SurfacePenaltyStatus::Ok;
}
} // namespace tlfea::contact::penalty_pair_detail
