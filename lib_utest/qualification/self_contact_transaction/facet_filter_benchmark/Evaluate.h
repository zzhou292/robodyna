// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include <type_traits>

namespace facet_filter_benchmark {
// Only a signature bridge. The old and new compiled adapter implementations
// own their actual query scheduling; the benchmark copies neither algorithm.
template <class Adapter>
void BeginChunk(Adapter& adapter, const Scene& scene) {
  if constexpr (std::is_void_v<decltype(adapter.BeginCandidateChunk(scene.pairs.data(), scene.pairs.size()))>)
    adapter.BeginCandidateChunk(scene.pairs.data(), scene.pairs.size());
  else {
    const auto report = adapter.BeginCandidateChunk(scene.pairs.data(), scene.pairs.size());
    Require(report.status == f::Status::Ok, report.message);
  }
}
template <class Adapter>
constexpr const char* BeginApi() {
  return std::is_void_v<decltype(std::declval<Adapter&>().BeginCandidateChunk(nullptr, 0))>
      ? "void_begin" : "report_begin";
}
inline void Evaluate(const Scene& scene, sct::FacetFilters* adapter,
                     std::vector<Row>& output) {
  Require(output.size() == scene.pairs.size(), "Benchmark output capacity changed");
  if (adapter) BeginChunk(*adapter, scene);
  for (std::size_t i = 0; i < scene.pairs.size(); ++i) {
    if (!adapter) { output[i] = scene.Scalar(i); continue; }
    const auto pair = scene.pairs[i];
    Row row;
    row.action = sct::ClassifyCandidatePairMotion(scene.motion[pair.first], scene.bounds[pair.first],
        scene.motion[pair.second], scene.bounds[pair.second]);
    if (row.action == sct::PairMotionAction::LinearNodalV1) {
      const auto reply = adapter->PrismAt(i);
      Require(reply.report.status == f::Status::Ok, reply.report.message);
      Require(reply.supplied, "Adapter benchmark unexpectedly selected CPU arithmetic");
      row.numerical = reply.value;
    }
    output[i] = row;
  }
}
}  // namespace facet_filter_benchmark
