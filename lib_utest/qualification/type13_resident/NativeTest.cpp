// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeCompare.h"

namespace type13_resident_test {
TEST(Type13ResidentNative, CompleteEndpointLoopKeepsPrefloorValuesAndCurrentOFF) {
  const double zero = 0, one = 1, mass[2]{0, 9}, inertia[2]{0, 11};
  for (double kt : {1e-30, 4., 1e15}) {
    SCOPED_TRACE(kt);
    const double kr = 2 * kt;
    double out[4];
    type13_native_endpoint_stiffness(&kt, &kr, &zero, &zero, mass, inertia,
                                     &one, &one, &one, out);
    EXPECT_EQ(out[0], kt);
    EXPECT_EQ(out[1], kt);
    EXPECT_EQ(out[2], kr);
    EXPECT_EQ(out[3], kr);
    type13_native_endpoint_stiffness(&kt, &kr, &zero, &zero, mass, inertia,
                                     &one, &one, &zero, out);
    for (double value : out) {
      EXPECT_EQ(value, 0);
    }
  }
  double damped[4];
  type13_native_endpoint_stiffness(&one, &one, &one, &one, mass, inertia,
                                   &one, &one, &one, damped);
  EXPECT_GT(damped[0], one);
  EXPECT_GT(damped[2], one);
}

TEST(Type13ResidentNative, RemovalAndTwoContinuingIntervalsKeepNativeForceHistory) {
  type13_recurrence_test::Case packet(true, .01);
  t::Evaluation actual;
  ASSERT_EQ(t::InitializeForce(packet.property, packet.reference, packet.nodes, actual), t::Status::Success);
  auto native = type13_recurrence_test::NativeEvaluate(packet.property, packet.reference,
      Virgin(packet.reference), packet.nodes, 0, true);
  Agreement(actual, native);
  const double dt = 1e-5;
  double previous = 0;
  for (double amplitude : {.2, .3, .4}) {
    SCOPED_TRACE(amplitude);
    type13_recurrence_test::Mode(packet, 0, amplitude, previous, dt);
    ASSERT_EQ(t::Evaluate(packet.property, packet.reference, actual.native_history,
                          packet.nodes, dt, actual), t::Status::Success);
    native = type13_recurrence_test::NativeEvaluate(packet.property, packet.reference,
                                                    native.native_history, packet.nodes, dt);
    Agreement(actual, native);
    double kt = -1, kr = -1;
    ASSERT_TRUE(detail::EndpointStiffness(actual, kt, kr));
    const auto stiffness = NativeStiffness(packet.property, native);
    EXPECT_EQ(kt, stiffness[0]);
    EXPECT_EQ(kr, stiffness[2]);
    previous = amplitude;
  }
  EXPECT_FALSE(actual.native_history.active);
}
} // namespace type13_resident_test
