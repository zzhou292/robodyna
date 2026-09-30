// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include "Copies.h"
#include <cstring>
#include <tuple>
namespace facet_filter_benchmark {
inline void SameCopies(const Copies& a, const Copies& b) {
  Require(std::tie(a.host_to_device_calls, a.device_to_host_calls,
      a.host_to_device_bytes, a.device_to_host_bytes, a.minimum_query_pairs,
      a.maximum_query_pairs, a.unexpected_copy_shape) ==
      std::tie(b.host_to_device_calls, b.device_to_host_calls,
      b.host_to_device_bytes, b.device_to_host_bytes, b.minimum_query_pairs,
      b.maximum_query_pairs, b.unexpected_copy_shape), "Actual query-copy accounting changed");
}
inline void Verify(const Scene& scene, const std::vector<Row>& expected,
    const std::vector<Row>& actual, Copies copies, bool adapter) {
  Require(expected.size() == actual.size() && actual.size() == scene.pairs.size(),
          "Benchmark result shape changed");
  for (std::size_t i = 0; i < expected.size(); ++i)
    Require(expected[i].action == actual[i].action &&
        filter_batch_test::Same(expected[i].numerical, actual[i].numerical),
        "Benchmark original-ordinal motion/filter result changed");
  Require(!copies.unexpected_copy_shape, "Unexpected copy occurred inside query observation");
  if (adapter && scene.linear_rows) {
    Require(copies.host_to_device_calls && copies.host_to_device_calls == copies.device_to_host_calls,
            "Query upload/readback counts disagree");
    Require(copies.host_to_device_bytes == scene.linear_rows * sizeof(c::FixedTrianglePair) &&
        copies.device_to_host_bytes == scene.linear_rows * sizeof(f::PairResult),
        "A linear row was repeated, dropped or not observed in numerical transfers");
  } else {
    Require(!copies.host_to_device_calls && !copies.device_to_host_calls &&
        !copies.host_to_device_bytes && !copies.device_to_host_bytes, "Nonquery path issued a CUDA copy");
  }
}
class Hash {
 public:
  void Add(std::uint64_t value) noexcept {
    for (unsigned i = 0; i < 8; ++i) { state ^= static_cast<unsigned char>(value >> (8 * i)); state *= 1099511628211ull; }
  }
  void Number(double value) noexcept { std::uint64_t bits; std::memcpy(&bits, &value, sizeof(bits)); Add(bits); }
  void Point(c::Vec3 v) noexcept { Number(v.x); Number(v.y); Number(v.z); }
  std::uint64_t state = 1469598103934665603ull;
};
inline std::uint64_t InputDigest(const Scene& scene) {
  Hash hash;
  hash.Add(scene.accepted.size()); hash.Add(scene.pairs.size());
  const auto parents = scene.source.uses.parents();
  for (std::size_t i = 0; i < scene.accepted.size(); ++i) {
    const auto& key = scene.accepted[i].key;
    hash.Add(key.source_instance_id); hash.Add(key.parent_eid); hash.Add(key.level); hash.Add(key.local_facet);
    for (const auto p : scene.accepted[i].vertices) hash.Point(p);
    for (const auto p : scene.prepared[i].vertices) hash.Point(p);
    hash.Add(scene.motion[i].parent); hash.Add(static_cast<unsigned>(scene.motion[i].motion));
    hash.Add(scene.motion[i].certified_affine); hash.Add(scene.motion[i].complete_rigid_group);
    hash.Point(scene.bounds[i].lower); hash.Point(scene.bounds[i].upper);
    hash.Number(parents[scene.motion[i].parent].reference_half_thickness_m);
  }
  for (const auto pair : scene.pairs) { hash.Add(pair.first); hash.Add(pair.second); }
  return hash.state;
}
inline std::uint64_t ResultDigest(const std::vector<Row>& rows) {
  Hash hash;
  for (const auto& row : rows) {
    hash.Add(static_cast<unsigned>(row.action)); hash.Add(static_cast<unsigned>(row.numerical.status));
    hash.Add(static_cast<unsigned>(row.numerical.category)); hash.Add(row.numerical.separated);
    hash.Add(static_cast<unsigned>(row.numerical.axis));
  }
  return hash.state;
}
}  // namespace facet_filter_benchmark
