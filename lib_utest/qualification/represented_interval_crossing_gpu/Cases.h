// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../represented_interval_crossing/Fixture.h"
#include "../represented_interval_crossing/ResultAssertions.h"
#include "lib_src/collision/RepresentedIntervalCrossingGpu.h"
#include "lib_src/collision/represented_interval_crossing/NativeStorageDomain.h"
#include "WideStorageCases.h"
#include <array>
#include <vector>

namespace native_gpu_test {
namespace c = tlfea::contact;
namespace fixture = represented_interval_test;
using S = c::RepresentedIntervalStatus;
using D = c::RepresentedIntervalDeviceStatus;
struct Cases {
  std::vector<c::RepresentedTrianglePath> paths;
  std::vector<c::RepresentedTrianglePair> pairs;
};
inline auto Positive(double z = 2) {
  auto values = fixture::BaseTriangle(z);
  for (auto& point : values) { point.x += 2; point.y += 2; }
  return values;
}
inline void Add(Cases& cases, const std::array<c::Vec3, 3>& a,
    const std::array<c::Vec3, 3>& b0, const std::array<c::Vec3, 3>& b1,
    c::RepresentedMotion motion = c::RepresentedMotion::LinearNodalV1) {
  const auto index = static_cast<std::uint32_t>(cases.paths.size());
  const auto eid = 100 + index, vertex = 1000 + 3 * index;
  cases.paths.push_back(fixture::Static(eid, a, vertex));
  cases.paths.push_back(fixture::Path(eid + 1, b0, b1, vertex + 3, motion));
  cases.pairs.push_back({index, index + 1});
}
inline Cases Mixed() {
  Cases cases;
  Add(cases, Positive(), Positive(), Positive());
  Add(cases, Positive(), Positive(3), Positive(3));
  Add(cases, {{{4,6,2},{2,3,2},{6,3,2}}}, {{{4,2,2},{2,5,2},{6,5,2}}},
      {{{4,2,2},{2,5,2},{6,5,2}}});
  Add(cases, Positive(), Positive(3), Positive(1));
  Add(cases, {{{4,4,4},{6,4,4},{4,6,4}}}, {{{2,3.5,3},{2,6.5,3},{2,5,5}}},
      {{{8,3.5,3},{8,6.5,3},{8,5,5}}});
  Add(cases, fixture::BaseTriangle(), fixture::BaseTriangle(1), fixture::BaseTriangle(1));
  Add(cases, Positive(), Positive(3), Positive(3), c::RepresentedMotion::RigidArc);
  const std::array<c::Vec3,3> collapsed{{{2,2,2},{2,2,2},{2,2,2}}};
  Add(cases, Positive(), collapsed, collapsed);
  Add(cases, Positive(), Positive(3), Positive(1.5));
  Cases wide;
  Add(wide, fixture::BaseTriangle(), fixture::BaseTriangle(1), fixture::BaseTriangle(1));
  RequireNonzeroWideStorage(wide.paths);
  std::array<c::Vec3,3> a, b;
  for (unsigned i=0;i<3;++i) {
    a[i]=wide.paths[0].vertices[i].endpoint[0];
    b[i]=wide.paths[1].vertices[i].endpoint[0];
  }
  Add(cases,a,b,b);
  return cases;
}
inline c::RepresentedIntervalGpuLimits Limits() {
  c::RepresentedIntervalGpuLimits result;
  result.native.max_paths = 128;
  result.native.max_input_pairs = 128;
  result.native.max_results = 128;
  result.native.max_work_per_pair = 64;
  result.native.max_total_work = 8192;
  result.native.max_depth = 20;
  result.native.worker_count = 2;
  return result;
}
inline std::size_t DevicePairs(const Cases& cases, unsigned depth) {
  std::size_t result = 0;
  for (const auto pair : cases.pairs)
    result += c::represented_interval_crossing::NativeStorageDomain::FromPaths(
        cases.paths[pair.first], cases.paths[pair.second], depth).eligible();
  return result;
}
inline std::vector<c::RepresentedIntervalResult> Copy(c::RepresentedIntervalResultView view) {
  EXPECT_TRUE(view.complete);
  return view.count ? std::vector<c::RepresentedIntervalResult>(view.data, view.data + view.count)
                    : std::vector<c::RepresentedIntervalResult>{};
}
inline void Preserved(c::RepresentedIntervalResultView before,
    const std::vector<c::RepresentedIntervalResult>& values, c::RepresentedIntervalResultView after) {
  ASSERT_EQ(after.data, before.data); ASSERT_EQ(after.count, before.count);
  ASSERT_EQ(after.complete, before.complete);
  for (std::size_t i = 0; i < values.size(); ++i) fixture::SameResult(after.data[i], values[i]);
}
}  // namespace native_gpu_test
