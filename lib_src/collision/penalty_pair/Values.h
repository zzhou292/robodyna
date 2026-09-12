// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Endpoint.h"
#include "../SurfaceContactGeometry.h"
#include "../SurfaceContactLaw.h"

namespace tlfea::contact {
// Pure fixed-feature algebra. No closest-feature search, area, effective mass,
// timestep, history or owner admission. The declared gap must equal represented
// RN (distance - h_A) - h_B; signed zero compares equal. No approximation error
// inflates these radii. Zero distance is rejected rather than inventing a normal.
// Inputs/output must not overlap; all borrowed values share one execution space.
// Failure preserves *output. Binary64 RN, no FMA/fast-math/reassociation/FTZ.
TL_SURFACE_HD inline SurfacePenaltyStatus EvaluateSurfacePenaltyPair(
    const SurfacePenaltyInput& input, SurfacePenaltyPacket* output) {
  using S = SurfacePenaltyStatus;
  using namespace penalty_pair_detail;
  if (!output || !input.positions.valid() || !input.velocities.valid() ||
      input.positions.node_count != input.velocities.node_count ||
      !IsFinite(input.declared_gap_m) || !IsFinite(input.stiffness_n_m) ||
      input.stiffness_n_m <= 0)
    return S::InvalidInput;
  auto status = ValidateEndpoint(input.a, input.positions.node_count);
  if (status != S::Ok) return status;
  status = ValidateEndpoint(input.b, input.positions.node_count);
  if (status != S::Ok) return status;
  SurfacePenaltyPacket result;
  auto mapped = EvaluateWeightedSurfacePoint(input.positions, input.velocities, input.a.point, &result.a);
  if (mapped != Status::kOk) return MappingStatus(mapped);
  mapped = EvaluateWeightedSurfacePoint(input.positions, input.velocities, input.b.point, &result.b);
  if (mapped != Status::kOk) return MappingStatus(mapped);
  const Vec3 separation = Subtract(result.a.position, result.b.position);
  if (!IsFinite(separation)) return S::NonFiniteResult;
  result.distance_m = geometry_detail::Length(separation);
  if (!IsFinite(result.distance_m)) return S::NonFiniteResult;
  if (result.distance_m == 0) return S::ZeroDistance;
  result.normal = geometry_detail::Divide(separation, result.distance_m);
  const double normal_length = geometry_detail::Length(result.normal);
  if (!IsFinite(result.normal) || !IsFinite(normal_length) || ::fabs(normal_length - 1) > 1e-12)
    return S::Unrepresentable;
  result.gap_m = (result.distance_m - input.a.reference_half_thickness_m) -
                 input.b.reference_half_thickness_m;
  if (!IsFinite(result.gap_m)) return S::NonFiniteResult;
  if (result.gap_m != input.declared_gap_m) return S::GapMismatch;
  const Vec3 relative_velocity = Subtract(result.a.velocity, result.b.velocity);
  if (!IsFinite(relative_velocity)) return S::NonFiniteResult;
  result.normal_velocity_m_s = Dot(relative_velocity, result.normal);
  if (!IsFinite(result.normal_velocity_m_s)) return S::NonFiniteResult;
  NormalContactResponse response; // Zero damping only; no mass/dt fields exported.
  mapped = normal_contact_detail::ApplyPenalty(input.stiffness_n_m, result.gap_m,
                                              result.normal_velocity_m_s, response);
  if (mapped != Status::kOk) return MappingStatus(mapped);
  result.active = response.active;
  result.normal_force_n = response.force;
  result.elastic_energy_j = response.elastic_energy;
  result.force_a_n = Scale(result.normal, response.force);
  result.force_b_n = Scale(result.force_a_n, -1);
  mapped = ProjectWeightedSurfaceForce(input.a.point, input.positions.node_count,
                                       result.force_a_n, &result.endpoint_a);
  if (mapped != Status::kOk) return MappingStatus(mapped);
  mapped = ProjectWeightedSurfaceForce(input.b.point, input.positions.node_count,
                                       result.force_b_n, &result.endpoint_b);
  if (mapped != Status::kOk) return MappingStatus(mapped);
  status = MergeForces(input.a, result.endpoint_a, result);
  if (status != S::Ok) return status;
  status = MergeForces(input.b, result.endpoint_b, result);
  if (status != S::Ok) return status;
  surface_majorant::SortNodes(result.nodes, result.count);
  for (std::uint32_t i = 0; i < result.count; ++i) {
    auto& node = result.nodes[i];
    node.free_force_n = surface_majorant::Project(node.force_n, node.translation_fixed_bits);
  }
  RepresentedJacobianTerm terms[8]{};
  std::uint32_t count = 0;
  status = AppendTerms(input.a, result.normal, terms, count);
  if (status != S::Ok) return status;
  status = AppendTerms(input.b, Scale(result.normal, -1), terms, count);
  if (status != S::Ok) return status;
  const auto bounded = BuildRepresentedJacobianMajorant(terms, count, input.positions.node_count,
      result.active ? input.stiffness_n_m : 0, &result.normal_majorant);
  if (bounded != SurfaceMajorantStatus::Ok) return MajorantStatus(bounded);
  result.valid = true;
  *output = result;
  return S::Ok;
}
} // namespace tlfea::contact
