// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Support.h"

namespace tlfea::contact::surface_majorant {
TL_SURFACE_HD inline SurfaceMajorantStatus Bound(SurfaceJacobianMajorant& value) {
  SortNodes(value.nodes, value.count);
  for (std::uint32_t i = 0; i < value.count; ++i) {
    auto& node = value.nodes[i];
    node.jacobian = Project(node.jacobian, node.translation_fixed_bits);
    if (!mass_detail::UpperNorm(node.jacobian, &node.norm_upper) ||
        !mass_detail::UpperSum(value.norm_sum_upper, node.norm_upper, &value.norm_sum_upper))
      return SurfaceMajorantStatus::Unrepresentable;
  }
  for (std::uint32_t i = 0; i < value.count; ++i) {
    auto& node = value.nodes[i];
    double first = 0;
    if (!mass_detail::UpperProduct(value.stiffness_n_m, node.norm_upper, &first) ||
        !mass_detail::UpperProduct(first, value.norm_sum_upper, &node.diagonal_n_m))
      return SurfaceMajorantStatus::Unrepresentable;
  }
  value.valid = true;
  return SurfaceMajorantStatus::Ok;
}
} // namespace tlfea::contact::surface_majorant
