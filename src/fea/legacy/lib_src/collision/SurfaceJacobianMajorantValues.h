// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "surface_majorant/Bounds.h"

namespace tlfea::contact {
// Finite represented vectors, not an ideal geometric Jacobian. Merge each
// component by RN additions in occurrence order; only then sort/project/bound.
// Inputs and output occupy one execution memory space and do not overlap.
// A failure leaves *output unchanged. No allocation, owner or mass is accessed.
// Required arithmetic: IEEE binary64 RN basic operations/sqrt, no FMA
// contraction, reassociation, fast-math or FTZ. Existing outward helpers reject
// lost positive products/norm terms, including some finite extreme/subnormal
// inputs. Unrepresentable is a refusal, never permission to use a zero bound.
TL_SURFACE_HD inline SurfaceMajorantStatus BuildRepresentedJacobianMajorant(
    const RepresentedJacobianTerm* terms, std::uint32_t count, std::uint32_t node_count,
    double stiffness_n_m, SurfaceJacobianMajorant* output) {
  using S = SurfaceMajorantStatus;
  if (!output || !terms || !count || !node_count || !IsFinite(stiffness_n_m) || stiffness_n_m < 0)
    return S::InvalidInput;
  if (count > MaxSurfaceMajorantNodes) return S::OutOfRange;
  SurfaceJacobianMajorant next;
  next.stiffness_n_m = stiffness_n_m;
  for (std::uint32_t i = 0; i < count; ++i) {
    const auto& term = terms[i];
    const auto status = surface_majorant::Check(term.node, node_count, term.translation_fixed_bits);
    if (status != S::Ok) return status;
    if (!IsFinite(term.value)) return S::InvalidInput;
    const auto index = surface_majorant::FindNode(next.nodes, next.count, term.node);
    if (index == next.count) {
      auto& node = next.nodes[next.count++];
      node.node = term.node;
      node.translation_fixed_bits = term.translation_fixed_bits;
      node.jacobian = term.value;
    } else {
      auto& node = next.nodes[index];
      if (node.translation_fixed_bits != term.translation_fixed_bits) return S::InconsistentMask;
      node.jacobian = Add(node.jacobian, term.value);
      if (!IsFinite(node.jacobian)) return S::Unrepresentable;
    }
  }
  const auto status = surface_majorant::Bound(next);
  if (status == S::Ok) *output = next;
  return status;
}

// Common-normal adapter preserves the legacy scalar schedule: signed weights
// merge in source order BEFORE one Scale(normal, merged_weight) per node. This
// is intentionally distinct from merging already rounded vector occurrences.
// The normal uses the legacy admission tolerance; it is never renormalized and
// the majorant uses actual represented components, never |weight| alone.
// translation_fixed_bits is a borrowed node_count-sized array; zero-weight and
// fully fixed entries still undergo complete input/mask validation.
TL_SURFACE_HD inline SurfaceMajorantStatus BuildSignedNormalMajorant(
    const SignedNodeWeight* weights, std::uint32_t count,
    const std::uint8_t* translation_fixed_bits, std::uint32_t node_count,
    Vec3 normal, double stiffness_n_m, SurfaceJacobianMajorant* output) {
  using S = SurfaceMajorantStatus;
  if (!output || !weights || !translation_fixed_bits || !count || !node_count ||
      !IsFinite(stiffness_n_m) || stiffness_n_m < 0)
    return S::InvalidInput;
  if (count > MaxSurfaceMajorantNodes) return S::OutOfRange;
  const double length = mass_detail::Norm(normal);
  if (!IsFinite(normal) || !IsFinite(length) || ::fabs(length - 1) > 1e-12) return S::InvalidInput;
  SignedNodeWeight merged[MaxSurfaceMajorantNodes]{};
  std::uint32_t size = 0;
  for (std::uint32_t i = 0; i < count; ++i) {
    const auto& term = weights[i];
    if (term.node >= node_count) return S::OutOfRange;
    if (!IsFinite(term.weight) || translation_fixed_bits[term.node] > 7) return S::InvalidInput;
    const auto index = surface_majorant::FindNode(merged, size, term.node);
    if (index == size) merged[size++] = term;
    else {
      merged[index].weight += term.weight;
      if (!IsFinite(merged[index].weight)) return S::Unrepresentable;
    }
  }
  SurfaceJacobianMajorant next;
  next.count = size;
  next.stiffness_n_m = stiffness_n_m;
  for (std::uint32_t i = 0; i < size; ++i) {
    auto& node = next.nodes[i];
    const auto& term = merged[i];
    node.node = term.node;
    node.translation_fixed_bits = translation_fixed_bits[term.node];
    node.jacobian = Scale(normal, term.weight);
    if (!surface_majorant::RepresentedProduct(normal.x, term.weight, node.jacobian.x) ||
        !surface_majorant::RepresentedProduct(normal.y, term.weight, node.jacobian.y) ||
        !surface_majorant::RepresentedProduct(normal.z, term.weight, node.jacobian.z))
      return S::Unrepresentable;
  }
  const auto status = surface_majorant::Bound(next);
  if (status == S::Ok) *output = next;
  return status;
}
} // namespace tlfea::contact
