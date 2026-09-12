#include "Compare.h"
#include "FrozenStability.h"
#include "lib_src/solvers/nodal_seal/RowLayout.h"
#include "lib_src/solvers/NodalStateLayout.h"
#include <algorithm>
#include <array>
#include <limits>
#include <vector>

namespace tl::fea::seal_test {
namespace ns = nodal_seal;
namespace old = seal_frozen_stability;
namespace {
ns::RowSummary Reduce(const std::vector<double>& k, const std::vector<double>& c) {
  // Different association than both serial order and the CUDA tree. Exact max
  // identity must survive reversed partitions, including ties and signed zero.
  std::array<ns::RowSummary, 17> parts;
  for (auto& part : parts) part = ns::EmptyRows();
  for (std::size_t i = k.size(); i-- > 0;)
    ns::IncludeRow(parts[i % parts.size()], k[i], c[i], i);
  auto result = ns::EmptyRows();
  for (auto i = parts.size(); i-- > 0;) ns::CombineRows(result, parts[i]);
  return result;
}
void Compare(const std::vector<double>& k, const std::vector<double>& c,
    double safety = .8, double minimum = 1e-12, double h = 1) {
  const auto n = static_cast<std::uint32_t>(k.size());
  stability::RowBounds rows{const_cast<double*>(k.data()), const_cast<double*>(c.data()), n, n, 7, 11, true, true, false};
  auto serial = rows;
  old::RowBounds frozen{rows.stiffness, rows.damping, n, n, 7, 11, true, true, false};
  stability::StepLimit out, direct;
  old::StepLimit expected;
  const auto reference = old::FinalizeRows(&frozen, safety, minimum, h, &expected);
  EXPECT_EQ(ns::FinalizeReducedRows(&rows, safety, minimum, h, Reduce(k, c), &out), reference);
  EXPECT_EQ(stability::FinalizeRows(&serial, safety, minimum, h, &direct), reference);
  SameRows(rows, frozen); SameRows(serial, frozen);
  SameLimit(out, expected); SameLimit(direct, expected);
}
}
TEST(SealRowsHost, ExactTiesSignedZerosAndScalarFormula) {
  std::vector<double> k(513), c(513, -0.);
  Compare(k, c);
  EXPECT_EQ(Bits(Reduce(k, c).stiffness), Bits(0.));
  EXPECT_EQ(Bits(Reduce(k, c).damping), Bits(0.));
  for (auto index : {3u, 256u, 512u}) { k[index] = 4; c[index] = 5; }
  auto maximum = Reduce(k, c);
  EXPECT_EQ(maximum.stiffness_node, 3u); EXPECT_EQ(maximum.damping_node, 3u);
  Compare(k, c);
  c[0] = 6; Compare(k, c);
  k[0] = 8; Compare(k, c, std::nextafter(1., 0.));
}
TEST(SealRowsHost, SubnormalExtremesAndMinimumFailureKeepExactOutputs) {
  const auto denorm = std::numeric_limits<double>::denorm_min();
  const auto maximum = std::numeric_limits<double>::max();
  for (double k : {0., denorm, 1e-300, 1., 1e300, maximum})
    for (double c : {0., denorm, 1., maximum}) {
      SCOPED_TRACE(k);
      SCOPED_TRACE(c);
      Compare({0, k, k}, {c, 0, c});
      Compare({0, k, k}, {c, 0, c}, .8, .5, 1.);
    }
}
TEST(SealRowsHost, InvalidRowsUseLegacyFailureFlagsAndFreshRetry) {
  std::vector<double> k(513), c(513);
  k[512] = -1; c[256] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(Reduce(k, c).invalid_node, 256u);
  Compare(k, c);
  k[512] = 4; c[256] = 8; Compare(k, c);
  for (double bad : {-1., std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::quiet_NaN()}) {
    k[512] = bad; Compare(k, c);
  }
}
TEST(SealRowsHost, PrefixErrorsClearOutputWithOriginalPriority) {
  double k[]{1}, c[]{1};
  for (unsigned fault = 0; fault != 10; ++fault) {
    SCOPED_TRACE(fault);
    stability::RowBounds a{k, c, 1, 1, 3, 7, true, true, false};
    if (fault == 0) a.valid = false;
    if (fault == 1) a.initialized = false;
    if (fault == 2) a.sealed = true;
    if (fault == 3) a.capacity = 0;
    if (fault == 4) a.stiffness = nullptr;
    if (fault == 5) a.node_count = 0;
    old::RowBounds b{a.stiffness, a.damping, a.node_count, a.capacity,
        a.base_epoch, a.attempt, a.initialized, a.valid, a.sealed};
    stability::StepLimit actual; actual.dt = 123;
    old::StepLimit expected; expected.dt = 123;
    const double safety = fault == 6 ? 1 : .8, minimum = fault == 7 ? -1 : 1e-12;
    const double h = fault == 8 ? 0 : 1;
    auto out = fault == 9 ? nullptr : &actual;
    auto reference = fault == 9 ? nullptr : &expected;
    EXPECT_EQ(ns::FinalizeReducedRows(&a, safety, minimum, h, Reduce({1}, {1}), out),
              old::FinalizeRows(&b, safety, minimum, h, reference));
    SameRows(a, b); SameLimit(actual, expected);
  }
  stability::StepLimit a; old::StepLimit b;
  EXPECT_EQ(ns::FinalizeReducedRows(nullptr, .8, 1e-12, 1, ns::EmptyRows(), &a),
            old::FinalizeRows(nullptr, .8, 1e-12, 1, &b));
  SameLimit(a, b);
}
TEST(SealRowsHost, OwnerLayoutIsRequiredAndTailHasAnExactCap) {
  std::vector<double> scratch(11 * 513);
  stability::RowBounds rows{scratch.data() + 6 * 513, scratch.data() + 7 * 513,
      513, 513, 0, 1, true, true, false};
  EXPECT_TRUE(ns::CanReduceRows(rows, scratch.data(), 513, .8, 1e-12, 1));
  ++rows.stiffness;
  EXPECT_FALSE(ns::CanReduceRows(rows, scratch.data(), 513, .8, 1e-12, 1));
  EXPECT_EQ(ns::RowBlocks(0), 0u); EXPECT_EQ(ns::RowBlocks(1), 1u);
  EXPECT_EQ(ns::RowBlocks(256), 1u); EXPECT_EQ(ns::RowBlocks(257), 2u);
  EXPECT_EQ(ns::RowBlocks(MaxActiveNodalStateNodes), 256u);
  EXPECT_EQ(ns::ControlBytes(256, MaxActiveNodalStateNodes), 256u + 8192u);
  EXPECT_EQ(ns::ControlBytes(255, 1), 0u);
  EXPECT_EQ(ns::ControlBytes(SIZE_MAX - 7, 1), 0u);
  EXPECT_EQ(ns::RowBlocks(std::size_t(UINT32_MAX) + 1), 0u);
  nodal_detail::StateLayout layout;
  const auto bytes = ns::ControlBytes(256, 513);
  ASSERT_TRUE(layout.Initialize(513, true, 0, 0, 0, bytes, MaxActiveNodalStateDeviceBytes));
  const auto cap = layout.bytes;
  EXPECT_EQ(layout.control.bytes, bytes);
  EXPECT_FALSE(layout.Initialize(513, true, 0, 0, 0, bytes, cap - 1));
  EXPECT_TRUE(layout.Initialize(513, true, 0, 0, 0, bytes, cap));
}
} // namespace tl::fea::seal_test
