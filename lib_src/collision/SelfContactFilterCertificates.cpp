// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SelfContactFilterCertificates.h"
#include "self_contact_filters/Prism.h"

namespace tlfea::contact {

bool CertifiedLinearFacetPrismSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    SelfContactFacetPrismAxisLimit axis_limit,
    SelfContactFacetPrismSeparationAxis* separated_axis,
    bool* valid) noexcept {
  return self_contact_filters::detail::CertifiedLinearFacetPrismSeparationImpl<true, false>(
      first_base, first_current, first_thickness,
      second_base, second_current, second_thickness,
      axis_limit, separated_axis, valid, nullptr);
}

self_contact_filters::PrismComparison self_contact_filters::ComparePrismHullCoincidence(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    SelfContactFacetPrismAxisLimit axis_limit) noexcept {
  PrismComparison result;
  result.original.separated = self_contact_filters::detail::CertifiedLinearFacetPrismSeparationImpl<false, true>(
      first_base, first_current, first_thickness,
      second_base, second_current, second_thickness,
      axis_limit, &result.original.axis, &result.original.valid, &result.original.counts);
  result.current.separated = self_contact_filters::detail::CertifiedLinearFacetPrismSeparationImpl<true, true>(
      first_base, first_current, first_thickness,
      second_base, second_current, second_thickness,
      axis_limit, &result.current.axis, &result.current.valid, &result.current.counts);
  return result;
}

bool CertifiedLinearFacetPrismSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    bool include_edge_axes,
    SelfContactFacetPrismSeparationAxis* separated_axis,
    bool* valid) noexcept {
  return CertifiedLinearFacetPrismSeparation(
      first_base, first_current, first_thickness,
      second_base, second_current, second_thickness,
      include_edge_axes
          ? SelfContactFacetPrismAxisLimit::EdgeCross
          : SelfContactFacetPrismAxisLimit::FaceNormal,
      separated_axis, valid);
}

SelfContactFacetFilterResult ClassifyAcceptedFacetPair(
    const CurrentFixedTriangle& first, double first_thickness,
    std::uint32_t first_complete_rigid_group,
    const CurrentFixedTriangle& second, double second_thickness,
    std::uint32_t second_complete_rigid_group) noexcept {
  return self_contact_filters::detail::ClassifyAcceptedFacetPairImpl(
      first, first_thickness, first_complete_rigid_group,
      second, second_thickness, second_complete_rigid_group);
}

}  // namespace tlfea::contact
