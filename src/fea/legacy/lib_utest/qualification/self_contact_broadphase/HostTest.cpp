#include "Oracle.h"
#include "ManySource.h"
#include "../self_contact_surface/Fixture.h"
#include "lib_src/collision/self_contact_broadphase/Layout.h"
#include <gtest/gtest.h>
#include <array>
#include <climits>
#include <random>

namespace surface_broadphase_test {
namespace ct = tlfea::contact;
namespace bp = ct::self_contact_broadphase;
using S = ct::SelfContactBroadphaseStatus;
namespace {
struct Save {
  std::vector<Key>& out;
  void operator()(int a, int b) { out.push_back(Canonical(a, b)); }
};
std::vector<AABB> Sorted(std::vector<AABB> boxes, unsigned axis) {
  std::stable_sort(boxes.begin(), boxes.end(), [&](const auto& a, const auto& b) {
    const double x[]{a.min.x, a.min.y, a.min.z}, y[]{b.min.x, b.min.y, b.min.z};
    return x[axis] < y[axis];
  });
  return boxes;
}
}
TEST(SelfContactBroadphaseHost, SharedSweepMatchesExhaustiveAllAxesWithoutTruncation) {
  std::mt19937 generator(98231);
  for (unsigned n : {1u, 2u, 17u, 257u}) {
    std::vector<AABB> boxes;
    for (unsigned i = 0; i < n; ++i) {
      const auto value = [&] { return (int(generator() % 31) - 15) * .125; };
      const double x = value(), y = value(), z = value();
      boxes.push_back({{x, y, z}, {x + .75, y + .5, z + .25}, static_cast<int>(n - i - 1)});
    }
    const auto expected = Exhaustive(boxes);
    for (unsigned axis = 0; axis < 3; ++axis) {
      const auto sorted = Sorted(boxes, axis);
      std::vector<Key> actual;
      for (unsigned i = 0; i < n; ++i) {
        broadphase_detail::CountPairs count;
        broadphase_detail::VisitLater(sorted.data(), n, i, axis, broadphase_detail::KeepAll{}, count);
        const auto before = actual.size();
        Save save{actual};
        broadphase_detail::VisitLater(sorted.data(), n, i, axis, broadphase_detail::KeepAll{}, save);
        EXPECT_EQ(actual.size() - before, count.count);
      }
      std::sort(actual.begin(), actual.end());
      EXPECT_EQ(actual, expected);
      EXPECT_EQ(std::adjacent_find(actual.begin(), actual.end()), actual.end());
    }
  }
}
TEST(SelfContactBroadphaseHost, LegacyMeshAndNeighborPoliciesStayExplicitAndUnchanged) {
  std::vector<AABB> boxes(8);
  for (int i = 0; i < 8; ++i) boxes[i] = {{-0., 0, 0}, {0, 1, 1}, i};
  const int bodies[]{0, 0, 1, 1, 2, 2, 3, 3};
  const long long neighbors[]{static_cast<long long>(Canonical(0, 2)), static_cast<long long>(Canonical(3, 6))};
  for (int self : {0, 1}) {
    std::vector<Key> actual, expected;
    for (int a = 0; a < 8; ++a) {
      Save save{actual};
      broadphase_detail::VisitLater(boxes.data(), 8, a, 0,
          broadphase_detail::LegacyFilter{neighbors, 2, bodies, self}, save);
      for (int b = a + 1; b < 8; ++b)
        if ((self || bodies[a] != bodies[b]) && !(a == 0 && b == 2) && !(a == 3 && b == 6))
          expected.push_back(Canonical(a, b));
    }
    EXPECT_EQ(actual, expected);
  }
  // New broadphase has no blanket shared-node/body/adjacency policy.
  EXPECT_EQ(Exhaustive(boxes).size(), 28u);
}
TEST(SelfContactBroadphaseHost, MultipleBlocksRetainEveryCoincidentSourceLayer) {
  const auto source = ManySource(513);
  ASSERT_TRUE(source.prepared());
  EXPECT_EQ(source.parents().size(), 513u);
  EXPECT_EQ(source.physical()->domain()->node_count(), 4u);
}
TEST(SelfContactBroadphaseHost, ExactParentMapsAndCompleteAlignedWorkspaceForecast) {
  self_contact_test::Fixture fixture(true, 77, true);
  const auto selected = fixture.Selection();
  ct::SelfContactSurfaceBinding source;
  ASSERT_EQ(source.Initialize(fixture.physical, self_contact_test::Input(selected)).status,
            ct::SelfContactSurfaceStatus::Ok);
  std::vector<bp::Parent> maps(source.parents().size());
  ASSERT_TRUE(bp::CopyParents(source, maps.data(), maps.size()));
  bool triangle = false, quad = false;
  for (std::size_t i = 0; i < maps.size(); ++i) {
    const auto& parent = source.parents()[i];
    EXPECT_EQ(maps[i].arity, parent.arity);
    EXPECT_EQ(maps[i].half_thickness, parent.reference_half_thickness_m);
    triangle |= parent.arity == 3; quad |= parent.arity == 4;
    for (unsigned j = 0; j < parent.arity; ++j)
      EXPECT_EQ(maps[i].nodes[j], parent.arity == 3 ? parent.t3.nodes[j] : parent.q4.nodes[j]);
  }
  EXPECT_TRUE(triangle); EXPECT_TRUE(quad);
  ct::SelfContactBroadphaseLimits limits;
  bp::Layout layout;
  const bp::ScratchRequirements temp{31, 4096, 777};
  ASSERT_EQ(bp::MakeLayout(source, limits, temp, 512, layout).status, S::Ok);
  EXPECT_EQ(layout.cub_temp.offset % 256, 0u);
  EXPECT_EQ(layout.cub_temp.bytes, 4096u);
  EXPECT_EQ(layout.forecast.device_bytes, layout.cub_temp.offset + layout.cub_temp.bytes);
  EXPECT_EQ(layout.forecast.startup_scratch_bytes, layout.parents.bytes + bp::QueryStagingBytes);
  EXPECT_EQ(layout.forecast.startup_host_bytes,
            layout.forecast.owned_host_bytes + layout.forecast.startup_scratch_bytes);
  limits.max_device_bytes = layout.forecast.device_bytes;
  limits.max_host_bytes = layout.forecast.startup_host_bytes;
  EXPECT_EQ(bp::MakeLayout(source, limits, temp, 512, layout).status, S::Ok);
  const auto before = layout;
  --limits.max_device_bytes;
  EXPECT_EQ(bp::MakeLayout(source, limits, temp, 512, layout).status, S::ResourceLimit);
  EXPECT_EQ(layout.forecast.device_bytes, before.forecast.device_bytes);
  ++limits.max_device_bytes; --limits.max_host_bytes;
  EXPECT_EQ(bp::MakeLayout(source, limits, temp, 512, layout).status, S::ResourceLimit);
  EXPECT_EQ(layout.forecast.startup_host_bytes, before.forecast.startup_host_bytes);
  ++limits.max_host_bytes;
  EXPECT_EQ(bp::MakeLayout(source, limits, temp, 512, layout).status, S::Ok);
  limits.max_pairs = std::size_t(INT_MAX) + 1;
  EXPECT_EQ(bp::MakeLayout(source, limits, temp, 512, layout).status, S::ResourceLimit);
}
} // namespace surface_broadphase_test
