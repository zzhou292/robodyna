// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Corpus.h"
#include "lib_src/collision/self_contact_filters/Batch.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace filter_batch_benchmark {
namespace c = tlfea::contact;
namespace f = c::self_contact_filters;
inline constexpr std::size_t ClassCount = 3;
inline constexpr std::size_t DefaultPairs = 4096;
inline constexpr std::size_t MaximumPairs = 65536;
struct Class {
  const char* name;
  const char* provenance;
};
extern const std::array<Class, ClassCount> Classes;
struct Cases {
  filter_batch_test::Corpus values;
  // Pre-expanded once so timed public CPU calls do not construct full facets.
  std::vector<c::CurrentFixedTriangle> accepted, prepared;
  std::vector<unsigned> kind;
  std::array<std::size_t, ClassCount> counts{};
  std::size_t seed_pairs = 0;
  bool yaris_geometry = false;
  std::size_t HostPayloadBytes() const noexcept;
};
Cases MakeCases(std::size_t pairs);
f::Limits Limits(const Cases&) noexcept;
void EvaluateCpu(const Cases&, bool accepted,
                 c::SelfContactFacetPrismAxisLimit, f::PairResult* output) noexcept;
}  // namespace filter_batch_benchmark
