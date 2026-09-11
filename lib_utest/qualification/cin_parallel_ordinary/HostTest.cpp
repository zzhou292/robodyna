// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/solvers/cin_advance/Node.h"
#include "lib_src/solvers/NodalCinStorage.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstring>

namespace {
namespace fe = tl::fea;
namespace ca = fe::cin_advance;
using Code = fe::NodalStatus;

TEST(CinParallelOrdinary, FailureKeyPreservesNodeTraversalForEveryArrivalOrder) {
  std::array<ca::FailureKey, 4> failures{
    ca::EncodeFailure(127, Code::StepTooLarge),
    ca::EncodeFailure(128, Code::InvalidOutput),
    ca::EncodeFailure(256, Code::StepTooLarge),
    ca::EncodeFailure(fe::MaxActiveNodalStateNodes-1, Code::InvalidOutput)};
  do {
    auto first = ca::NoFailure;
    for (const auto failure : failures) first = std::min(first, failure);
    EXPECT_EQ(ca::FailureNode(first), 127u);
    EXPECT_EQ(ca::FailureStatus(first), Code::StepTooLarge);
  } while (std::next_permutation(failures.begin(), failures.end()));
  EXPECT_LT(ca::EncodeFailure(fe::MaxActiveNodalStateNodes-1, Code::InvalidOutput), ca::NoFailure);
}

TEST(CinParallelOrdinary, FullCountLayoutChargesOnlyOneKeyAndPreservesLateRetry) {
  constexpr std::size_t nodes = 372435, rows = 11165, witnesses = 13173;
  fe::NodalCinLimits limits;
  fe::nodal_detail::CinLayout layout;
  ASSERT_TRUE(layout.Initialize(nodes, rows, witnesses, limits, sizeof(fe::nodal_detail::CinStorage)));
  RecordProperty("full_optional_device_bytes", std::to_string(layout.optional_device_bytes));
  RecordProperty("layout_host_bytes", std::to_string(layout.host_bytes));
  RecordProperty("storage_metadata_bytes", std::to_string(sizeof(fe::nodal_detail::CinStorage)));
  EXPECT_EQ(layout.failure.bytes, sizeof(ca::FailureKey));
  EXPECT_EQ(layout.failure.count, 1u);
  EXPECT_EQ(layout.failure.offset % alignof(ca::FailureKey), 0u);
  EXPECT_EQ(layout.work.offset + layout.work.bytes, layout.failure.offset);
  EXPECT_EQ(layout.device_bytes, layout.failure.offset + sizeof(ca::FailureKey));
  EXPECT_EQ(layout.scratch_values, 9*nodes);
  EXPECT_EQ(layout.state_values, 4*nodes+2*rows+1);
  EXPECT_EQ(layout.optional_device_bytes, layout.device_bytes+2*layout.state_values*sizeof(double));
  auto exact = limits;
  exact.max_device_bytes = layout.optional_device_bytes;
  exact.max_host_bytes = layout.host_bytes;
  fe::nodal_detail::CinLayout retried;
  ASSERT_TRUE(retried.Initialize(nodes, rows, witnesses, exact, sizeof(fe::nodal_detail::CinStorage)));
  const auto old_bytes = retried.device_bytes;
  --exact.max_device_bytes;
  EXPECT_FALSE(retried.Initialize(nodes, rows, witnesses, exact, sizeof(fe::nodal_detail::CinStorage)));
  EXPECT_EQ(retried.device_bytes, old_bytes);
  ++exact.max_device_bytes;
  --exact.max_host_bytes;
  EXPECT_FALSE(retried.Initialize(nodes, rows, witnesses, exact, sizeof(fe::nodal_detail::CinStorage)));
  EXPECT_EQ(retried.device_bytes, old_bytes);
  ++exact.max_host_bytes;
  EXPECT_TRUE(retried.Initialize(nodes, rows, witnesses, exact, sizeof(fe::nodal_detail::CinStorage)));
}

struct NodePacket {
  static constexpr std::uint32_t N = 5;
  std::array<double, 19*N> accepted{}, trial{};
  std::array<double, 6*N> loads{};
  std::array<double, 4*N> coefficients{};
  std::array<double, 9*N> work{};
  std::array<std::uint8_t, 3*N> fixed{};
  std::array<std::uint8_t, N> dependent{}, members{}, present{};
  ca::Input input;
  NodePacket() {
    present.fill(1);
    for (std::uint32_t i=0; i<N; ++i) {
      accepted[9*N+4*i] = 1;
      coefficients[i] = 2;
      coefficients[N+i] = .5;
      loads[i] = 4;
      loads[3*N+i] = 1;
    }
    input.accepted = accepted.data();
    input.trial = trial.data();
    input.loads = loads.data();
    input.fixed = fixed.data();
    input.model.node_count = N;
    input.model.dependent_nodes = dependent.data();
    input.tail = coefficients.data();
    input.work = work.data();
    input.rotation_present = present.data();
    input.groups.member_nodes = members.data();
    input.durations = {0, .01, .02};
    input.maximum_angle = .2;
  }
};

TEST(CinParallelOrdinary, DependentZeroCoefficientsFixedBitsAndAbsentRotationKeepTheirRoles) {
  NodePacket p;
  p.dependent[0] = 1;
  p.members[1] = fe::rigid::PartMemberNode;
  p.members[2] = fe::rigid::PhysicalPlainMemberNode;
  for (unsigned i=0; i<3; ++i) {
    p.coefficients[i] = 0;
    p.coefficients[NodePacket::N+i] = 0;
  }
  p.fixed[NodePacket::N+3] = 7;
  p.fixed[2*NodePacket::N+3] = 1;
  p.coefficients[3] = 0;
  p.coefficients[NodePacket::N+3] = 0;
  p.present[4] = 0;
  p.coefficients[NodePacket::N+4] = 0;
  p.loads[3*NodePacket::N+4] = 0;
  const auto accepted = p.accepted;
  for (unsigned i=0; i<NodePacket::N; ++i) EXPECT_EQ(ca::AdvanceNode(p.input, i), Code::Ok);
  EXPECT_EQ(p.accepted, accepted);
  for (unsigned i=0; i<4; ++i) EXPECT_EQ(p.coefficients[2*NodePacket::N+i], 0);
  for (unsigned i=0; i<NodePacket::N; ++i) EXPECT_EQ(p.coefficients[3*NodePacket::N+i], 0);
  EXPECT_EQ(p.trial[13*NodePacket::N+3*3], -4);
  EXPECT_EQ(p.trial[16*NodePacket::N+3*3], -1);
  EXPECT_EQ(p.trial[16*NodePacket::N+3*4], 0);
}

TEST(CinParallelOrdinary, NodeReportsAngleBeforeLaterInverseAndRejectsAbsentCouple) {
  NodePacket p;
  p.accepted[6*NodePacket::N] = 100;
  p.coefficients[1] = 0;
  EXPECT_EQ(ca::AdvanceNode(p.input, 0), Code::StepTooLarge);
  EXPECT_EQ(ca::AdvanceNode(p.input, 1), Code::InvalidOutput);
  p.present[4] = 0;
  EXPECT_EQ(ca::AdvanceNode(p.input, 4), Code::InvalidOutput);
}
} // namespace
