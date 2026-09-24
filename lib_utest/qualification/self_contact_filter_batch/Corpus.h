// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/self_contact_filters/Types.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

// Generic synthetic qualification geometry; no vehicle-model provenance claim.
namespace filter_batch_test {
namespace c = tlfea::contact;
namespace f = c::self_contact_filters;
struct Corpus {
  std::vector<f::TriangleGeometry> accepted, prepared;
  std::vector<f::FacetProperties> properties;
  std::vector<c::FixedTrianglePair> pairs;
  void Add(f::TriangleGeometry a, f::TriangleGeometry b, c::Vec3 motion = {},
           double thickness = .01, std::uint32_t group = UINT32_MAX) {
    const auto start = static_cast<std::uint32_t>(accepted.size());
    accepted.push_back(a); accepted.push_back(b);
    prepared.push_back(a);
    for (auto& point : b.vertices) point = c::Add(point, motion);
    prepared.push_back(b);
    properties.push_back({thickness, group}); properties.push_back({thickness, group});
    pairs.push_back({start, start + 1});
  }
  f::SceneView scene() const { return {accepted.data(), prepared.data(), properties.data(), accepted.size()}; }
  f::PairView input() const { return {pairs.data(), pairs.size()}; }
};
inline c::CurrentFixedTriangle Full(const f::TriangleGeometry& geometry) {
  c::CurrentFixedTriangle result;
  std::copy_n(geometry.vertices, 3, result.vertices);
  return result;
}
inline f::PairResult Reference(const Corpus& corpus, c::FixedTrianglePair pair,
    bool accepted, c::SelfContactFacetPrismAxisLimit limit = c::SelfContactFacetPrismAxisLimit::VertexVertex) {
  const auto a = Full(corpus.accepted[pair.first]), b = Full(corpus.accepted[pair.second]);
  const auto& ap = corpus.properties[pair.first];
  const auto& bp = corpus.properties[pair.second];
  f::PairResult result;
  if (accepted) {
    const auto value = c::ClassifyAcceptedFacetPair(a, ap.half_thickness, ap.complete_rigid_group,
                                                  b, bp.half_thickness, bp.complete_rigid_group);
    result.status = value.status; result.category = value.category;
  } else {
    bool valid = false;
    result.separated = c::CertifiedLinearFacetPrismSeparation(
        a, Full(corpus.prepared[pair.first]), ap.half_thickness,
        b, Full(corpus.prepared[pair.second]), bp.half_thickness, limit, &result.axis, &valid);
    result.status = valid ? c::SelfContactFacetFilterStatus::Ok : c::SelfContactFacetFilterStatus::InvalidInput;
  }
  return result;
}
inline bool Same(const f::PairResult& a, const f::PairResult& b) {
  return a.status == b.status && a.category == b.category && a.separated == b.separated && a.axis == b.axis;
}
inline Corpus MakeCorpus(bool malformed = true) {
  Corpus corpus;
  const f::TriangleGeometry base{{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}};
  for (const double gap : {0., .019999999999999997, .02, .020000000000000004, .125, 2.}) {
    auto second = base;
    for (auto& vertex : second.vertices) vertex.z = gap;
    corpus.Add(base, second);
    corpus.Add(base, second, {0, 0, -2 * gap});
  }
  corpus.Add(base, base, {}, .01, 7); // Existing same-rigid classification.
  auto coincident = base;
  coincident.vertices[1] = {0, -2, 1}; coincident.vertices[2] = {0, -2, -1};
  corpus.Add(base, coincident); // Distinct facets sharing a coordinate.
  auto degenerate = base; degenerate.vertices[2] = degenerate.vertices[1];
  corpus.Add(base, degenerate, {0, 0, 1});
  auto zeros = base; zeros.vertices[0] = {-0., +0., -0.};
  corpus.Add(base, zeros, {0, 0, std::numeric_limits<double>::denorm_min()});
  for (int exponent : {-1070, -512, -100, 0, 100, 512, 1000}) {
    auto first = base, second = base;
    for (auto& vertex : first.vertices) vertex = c::Scale(vertex, std::ldexp(1., exponent));
    for (auto& vertex : second.vertices) vertex = c::Scale(c::Add(vertex, {.25, .25, .125}), std::ldexp(1., exponent));
    corpus.Add(first, second, {}, std::max(std::numeric_limits<double>::denorm_min(), std::ldexp(.01, exponent)));
  }
  // A fixed integer generator gives reproducible dyadic geometry without a
  // platform-dependent random-distribution implementation.
  std::uint64_t state = 0x73636f6e74616374ULL;
  const auto scalar = [&]() {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return (static_cast<int>((state >> 32) & 4095) - 2048) / 512.;
  };
  for (unsigned i = 0; i < 128; ++i) {
    f::TriangleGeometry a, b;
    for (auto* triangle : {&a, &b}) for (auto& vertex : triangle->vertices) vertex = {scalar(), scalar(), scalar()};
    corpus.Add(a, b, {scalar() / 16, scalar() / 16, scalar() / 16});
  }
  if (malformed) {
    for (double value : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()}) {
      auto second = base; second.vertices[2].x = value;
      corpus.Add(base, second);
    }
    corpus.Add(base, base, {}, 0);
    corpus.Add(base, base, {}, -1);
    corpus.Add(base, base, {}, std::numeric_limits<double>::infinity());
  }
  return corpus;
}
}  // namespace filter_batch_test
