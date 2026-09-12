// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/elements/beam_common/EndpointFields.h"
#include "lib_src/elements/beam_common/EndpointAssembly.h"
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <limits>
namespace {
namespace fe = tl::fea;
namespace beam = fe::beam_endpoint;
struct Motion {
  double x[9]{0, 0, 0, 1, 0, 0, 9, 8, 7};
  double v[9]{}, w[9]{};
  double q[12]{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
  fe::DeviceNodalKinematicsView View() { return {x, v, w, 3, 0, q}; }
};
TEST(BeamEndpoints, GatherMechanicalNodesOnlyAndPreserveOutputOnLateInvalidRotation) {
  Motion source;
  source.x[6] = std::numeric_limits<double>::quiet_NaN();
  const std::size_t nodes[2]{0, 1};
  beam::Motion gathered;
  ASSERT_TRUE(beam::Gather(nodes, source.View(), gathered));
  EXPECT_EQ(gathered.position[1].x, 1.);
  const auto saved = gathered;
  source.q[4] = 2.;
  EXPECT_FALSE(beam::Gather(nodes, source.View(), gathered));
  EXPECT_EQ(std::memcmp(&saved, &gathered, sizeof(saved)), 0);
  source.q[4] = 1.;
  ASSERT_TRUE(beam::Gather(nodes, source.View(), gathered));
  EXPECT_EQ(std::memcmp(&saved, &gathered, sizeof(saved)), 0);
}
TEST(BeamEndpoints, RhsWorkUsesTranslationAndRotationWithSuppliedHalfKick) {
  Motion base, next;
  next.v[0] = 4.; next.w[1] = 6.; next.x[0] = .4;
  fe::NodalPreparedView view;
  view.base_kinematics = base.View(); view.kinematics = next.View(); view.kick_dt = .05;
  const std::size_t nodes[2]{0, 1};
  const tl::math::Vec3 force[2]{{2, 0, 0}, {}}, couple[2]{{0, 3, 0}, {}};
  double kick = 1., drift = 2.;
  ASSERT_TRUE(beam::AccumulateRhsWork(nodes, force, couple, view, .1, kick, drift));
  EXPECT_DOUBLE_EQ(kick, 1. + .05 * (2. * 2. + 3. * 3.));
  EXPECT_DOUBLE_EQ(drift, 2. + 2. * .4 + 3. * (.1 * 6.));
  const auto saved_kick = kick, saved_drift = drift;
  next.w[4] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(beam::AccumulateRhsWork(nodes, force, couple, view, .1, kick, drift));
  EXPECT_EQ(kick, saved_kick); EXPECT_EQ(drift, saved_drift);
}
struct Forces {
  double value[8][3]{};
  fe::DeviceNodalForceView View() {
    return {value[0], value[1], value[2], value[3], value[4], value[5], 3};
  }
};
TEST(BeamEndpoints, OrderedForceCoupleAndStiffnessPreserveExistingContributorsAndN3) {
  Forces output;
  for (unsigned c = 0; c < 8; ++c)
    for (unsigned n = 0; n < 3; ++n) output.value[c][n] = 10. + c + n;
  const auto original = output;
  const std::size_t nodes[2]{1, 0};
  const tl::math::Vec3 force[2]{{1, 2, 3}, {-1, -2, -3}}, couple[2]{{4, 5, 6}, {7, 8, 9}};
  ASSERT_EQ(beam::Accumulate(nodes, force, couple, 2., 3., output.View(), output.value[6], output.value[7]),
      fe::NodalForceAssemblyStatus::Success);
  EXPECT_EQ(output.value[0][1], original.value[0][1] + 1.);
  EXPECT_EQ(output.value[5][0], original.value[5][0] + 9.);
  for (unsigned n = 0; n < 2; ++n) {
    EXPECT_EQ(output.value[6][n], original.value[6][n] + 2.);
    EXPECT_EQ(output.value[7][n], original.value[7][n] + 3.);
  }
  for (unsigned c = 0; c < 8; ++c) EXPECT_EQ(output.value[c][2], original.value[c][2]);
}
TEST(BeamEndpoints, LateForceFailureAndBadScalarOrMappingLeaveAllEightArraysUnchanged) {
  Forces output;
  const std::size_t nodes[2]{0, 1}, duplicate[2]{0, 0};
  const tl::math::Vec3 force[2]{{1, 2, 3}, {4, 5, 6}};
  tl::math::Vec3 couple[2]{{1, 2, 3}, {4, 5, std::numeric_limits<double>::infinity()}};
  const auto saved = output;
  EXPECT_EQ(beam::Accumulate(nodes, force, couple, 2., 3., output.View(), output.value[6], output.value[7]),
      fe::NodalForceAssemblyStatus::NonfiniteResult);
  EXPECT_EQ(std::memcmp(&output, &saved, sizeof(output)), 0);
  couple[1].z = 6.;
  EXPECT_EQ(beam::Accumulate(duplicate, force, couple, 2., 3., output.View(), output.value[6], output.value[7]),
      fe::NodalForceAssemblyStatus::InvalidConnectivity);
  EXPECT_EQ(std::memcmp(&output, &saved, sizeof(output)), 0);
  EXPECT_EQ(beam::Accumulate(nodes, force, couple, -1., 3., output.View(), output.value[6], output.value[7]),
      fe::NodalForceAssemblyStatus::InvalidView);
  EXPECT_EQ(std::memcmp(&output, &saved, sizeof(output)), 0);
  ASSERT_EQ(beam::Accumulate(nodes, force, couple, 2., 3., output.View(), output.value[6], output.value[7]),
      fe::NodalForceAssemblyStatus::Success);
}
}
