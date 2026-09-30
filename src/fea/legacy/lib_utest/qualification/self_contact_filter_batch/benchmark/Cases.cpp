// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#ifdef TL_FILTER_YARIS_FIXTURES
#include "YarisGeometry.h"
#endif
#include <stdexcept>

namespace filter_batch_benchmark {
const std::array<Class, ClassCount> Classes{{
    {"synthetic_core", "unmodified no-GTest Corpus.h, finite dyadic/gap/degenerate/exponent families"},
    {"yaris_translation_geometry", "maintained YarisGeometry.h; synthetic ordinal clones, half-thickness 0.001 m and no rigid group"},
    {"yaris_shared_vertex_geometry", "maintained observed failure 8 coordinate bits; synthetic ordinal clones, half-thickness 0.001 m and no rigid group"},
}};

Cases MakeCases(std::size_t count) {
  if (!count || count > MaximumPairs)
    throw std::invalid_argument("Filter benchmark pairs must be 1..65536");
  const auto seed = filter_batch_test::MakeCorpus(false);
  if (seed.pairs.empty()) throw std::logic_error("Empty core filter corpus");
  Cases result;
  result.seed_pairs = seed.pairs.size();
  auto& data = result.values;
  data.accepted.reserve(2 * count); data.prepared.reserve(2 * count);
  data.properties.reserve(2 * count); data.pairs.reserve(count);
  result.kind.reserve(count);
#ifdef TL_FILTER_YARIS_FIXTURES
  result.yaris_geometry = true;
#endif
  for (std::size_t row = 0; row < count; ++row) {
    unsigned kind = 0;
    const auto offset = static_cast<std::uint32_t>(data.accepted.size());
#ifdef TL_FILTER_YARIS_FIXTURES
    if (row % 4 < 2) {
      kind = 1 + row % 4;
      const auto first_eid = 1000 + 2 * row, first_vertex = 10000 + 6 * row;
      const auto paths = kind == 1
          ? represented_interval_test::YarisTranslationPaths(
                first_eid, first_eid + 1, first_vertex, first_vertex + 3)
          : represented_interval_test::YarisSharedVertexPaths(
                first_eid, first_eid + 1, first_vertex, first_vertex + 3);
      for (const auto& path : paths) {
        f::TriangleGeometry base, next;
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
          base.vertices[vertex] = path.vertices[vertex].endpoint[0];
          next.vertices[vertex] = path.vertices[vertex].endpoint[1];
        }
        data.accepted.push_back(base); data.prepared.push_back(next);
        data.properties.push_back({.001, UINT32_MAX});
      }
    } else
#endif
    {
      const auto pair = seed.pairs[row % seed.pairs.size()];
      for (const auto facet : {pair.first, pair.second}) {
        data.accepted.push_back(seed.accepted[facet]);
        data.prepared.push_back(seed.prepared[facet]);
        data.properties.push_back(seed.properties[facet]);
      }
    }
    data.pairs.push_back({offset, offset + 1});
    result.kind.push_back(kind); ++result.counts[kind];
  }
  result.accepted.reserve(data.accepted.size());
  result.prepared.reserve(data.prepared.size());
  for (std::size_t facet = 0; facet < data.accepted.size(); ++facet) {
    result.accepted.push_back(filter_batch_test::Full(data.accepted[facet]));
    result.prepared.push_back(filter_batch_test::Full(data.prepared[facet]));
  }
  return result;
}

std::size_t Cases::HostPayloadBytes() const noexcept {
  return (values.accepted.capacity() + values.prepared.capacity()) * sizeof(f::TriangleGeometry) +
      values.properties.capacity() * sizeof(f::FacetProperties) +
      values.pairs.capacity() * sizeof(c::FixedTrianglePair) +
      (accepted.capacity() + prepared.capacity()) * sizeof(c::CurrentFixedTriangle) +
      kind.capacity() * sizeof(unsigned);
}
f::Limits Limits(const Cases& cases) noexcept {
  f::Limits result;
  result.max_facets = cases.values.accepted.size();
  result.max_pairs = cases.values.pairs.size();
  return result;
}
void EvaluateCpu(const Cases& cases, bool accepted,
                 c::SelfContactFacetPrismAxisLimit limit, f::PairResult* output) noexcept {
  for (std::size_t row = 0; row < cases.values.pairs.size(); ++row) {
    const auto pair = cases.values.pairs[row];
    const auto& a = cases.values.properties[pair.first];
    const auto& b = cases.values.properties[pair.second];
    f::PairResult result;
    if (accepted) {
      const auto value = c::ClassifyAcceptedFacetPair(
          cases.accepted[pair.first], a.half_thickness, a.complete_rigid_group,
          cases.accepted[pair.second], b.half_thickness, b.complete_rigid_group);
      result.status = value.status; result.category = value.category;
    } else {
      bool valid = false;
      result.separated = c::CertifiedLinearFacetPrismSeparation(
          cases.accepted[pair.first], cases.prepared[pair.first], a.half_thickness,
          cases.accepted[pair.second], cases.prepared[pair.second], b.half_thickness,
          limit, &result.axis, &valid);
      result.status = valid ? c::SelfContactFacetFilterStatus::Ok
                            : c::SelfContactFacetFilterStatus::InvalidInput;
    }
    output[row] = result;
  }
}
}  // namespace filter_batch_benchmark
