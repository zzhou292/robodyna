// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossingTypes.h"
#include <utility>
namespace tlfea::contact::represented_interval_crossing {
struct CanonicalPair {
  std::uint32_t first = 0, second = 0;
  std::size_t input_pair = SIZE_MAX;
  RepresentedIntervalPairKey key;
};
// Value construction only, after the native owner authenticates the referenced
// indices. Both ordinary slices and private numerical cohorts use this same
// exact source-key normalization; it does not issue validation authority.
template <class Compare>
CanonicalPair CanonicalizePair(const RepresentedTrianglePath* paths,
    RepresentedTrianglePair input, std::size_t ordinal, const Compare& compare) noexcept {
  CanonicalPair pair;
  pair.first = input.first;
  pair.second = input.second;
  pair.input_pair = ordinal;
  if (compare(paths[pair.second].key, paths[pair.first].key) < 0)
    std::swap(pair.first, pair.second);
  pair.key.paths[0] = paths[pair.first].key;
  pair.key.paths[1] = paths[pair.second].key;
  return pair;
}
}  // namespace tlfea::contact::represented_interval_crossing
