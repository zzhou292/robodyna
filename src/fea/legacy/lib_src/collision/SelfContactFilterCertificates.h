// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "FixedTriangleFeatureTypes.h"

#include <cstdint>

namespace tlfea::contact {

// Disjoint disposition of one represented accepted-geometry facet pair.
// Classification order is documented below; numeric values preserve the
// original ABI rather than encoding that order. ExactRemaining means that
// these conservative certificates proved no exclusion; it does not run
// feature discovery or interval crossing.
enum class SelfContactFacetFilterCategory : std::uint8_t {
  ExcludedSameRigidGroup = 0,
  CoordinateAabbSeparated = 1,
  FaceAxisSeparated = 2,
  EdgeCrossAxisSeparated = 3,
  ExactRemaining = 4,
  // Appended values preserve the numeric ABI of the original categories.
  VertexEdgeAxisSeparated = 5,
  VertexVertexAxisSeparated = 6,
};

enum class SelfContactFacetFilterStatus : std::uint8_t {
  Ok,
  InvalidInput,
};

struct SelfContactFacetFilterResult {
  SelfContactFacetFilterStatus status =
      SelfContactFacetFilterStatus::Ok;
  SelfContactFacetFilterCategory category =
      SelfContactFacetFilterCategory::ExactRemaining;
};

enum class SelfContactFacetPrismSeparationAxis : std::uint8_t {
  None = 0,
  FaceNormal = 1,
  EdgeCross = 2,
  VertexEdge = 3,
  VertexVertex = 4,
};

// Ordered optional-axis limit. Face normals are always tested; each value
// additionally enables every preceding family.
enum class SelfContactFacetPrismAxisLimit : std::uint8_t {
  FaceNormal = 0,
  EdgeCross = 1,
  VertexEdge = 2,
  VertexVertex = 3,
};

// Production conservative prism certificate used by candidate filtering.
// Strict separation is the only successful certificate. Touching, degenerate
// axes, and finite arithmetic overflow remain unresolved; nonfinite geometry
// or nonpositive/nonfinite thickness is invalid.
bool CertifiedLinearFacetPrismSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    SelfContactFacetPrismAxisLimit axis_limit,
    SelfContactFacetPrismSeparationAxis* axis,
    bool* valid) noexcept;

// Source- and binary-compatible original entry point. false tests face
// normals; true additionally tests edge-cross axes.
bool CertifiedLinearFacetPrismSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    bool include_edge_axes,
    SelfContactFacetPrismSeparationAxis* axis,
    bool* valid) noexcept;

// Production accepted-geometry filter. This is the same ordered certificate
// path used before accepted feature discovery: same rigid group, coordinate
// AABB, face, edge-cross, vertex-edge and vertex-vertex axes, then exact work
// remaining.
SelfContactFacetFilterResult ClassifyAcceptedFacetPair(
    const CurrentFixedTriangle& first, double first_thickness,
    std::uint32_t first_complete_rigid_group,
    const CurrentFixedTriangle& second, double second_thickness,
    std::uint32_t second_complete_rigid_group) noexcept;

}  // namespace tlfea::contact
