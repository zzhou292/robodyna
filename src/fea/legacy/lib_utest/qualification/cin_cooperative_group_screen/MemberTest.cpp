// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/solvers/cin_limiter/Capture.h"

namespace tl::fea::cooperative_test {
TEST(CinCooperativeMember, CompleteOriginalResponseAndTraceBits) {
  tlfea::contact::RigidContactBody body;
  body.current_frame.axes = {{1,0,0,0,1,0,0,0,1}};
  body.mass = 3;
  body.current_frame.inertia = {.25, 1.5, 7};
  body.center = {10000, -.25, .125};
  double nominal = 0;
  ASSERT_TRUE(frozen::AddRigidMemberTrace(body, {10000.125,-.234375,.3125}, 1, 1, nominal));
  const double values[]{0., -0., 1e-100, .25, 1e100,
      std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::max()};
  for (auto k : values) for (auto r : values) for (auto trace : values) {
    const tl::math::Vec3 x{10000.125, -.234375, .3125};
    auto a = trace, b = trace, c = trace;
    const bool expected = frozen::AddRigidMemberTrace(body, x, k, r, a);
    EXPECT_EQ(cin_timestep::AddRigidMemberTrace(body, x, k, r, b), expected);
    const auto responses = staged::PrepareResponses(body, x, k, r);
    EXPECT_EQ(cin_timestep::detail::AddRigidMemberTraceWithResponse(body, x, k, r, c,
        staged::ReadPreparedResponse{responses}), expected);
    Exact(a, b);
    Exact(a, c);
    if (!expected) Exact(c, trace);
  }
}
TEST(CinCooperativeMember, UnavailableSuffixAndEarlierOverflowShortCircuit) {
  tlfea::contact::RigidContactBody body;
  body.current_frame.axes = {{1,0,0,0,1,0,0,0,1}};
  body.mass = 1;
  body.current_frame.inertia = {1, 2, 3};
  auto packet = staged::PrepareResponses(body, {0,0,0}, 1, 1);
  ASSERT_TRUE(packet.axis[0].valid);
  packet.axis[1] = {};
  packet.axis[2] = {std::numeric_limits<double>::quiet_NaN(), -1, true};
  struct Read {
    staged::ReadPreparedResponse read;
    unsigned* count;
    bool operator()(const tlfea::contact::RigidContactBody& b, tl::math::Vec3 p,
        tl::math::Vec3 n, tlfea::contact::RigidNormalResponse& result) {
      ++*count;
      return read(b, p, n, result);
    }
  };
  unsigned calls = 0;
  double trace = std::numeric_limits<double>::max();
  EXPECT_FALSE(cin_timestep::detail::AddRigidMemberTraceWithResponse(body, {0,0,0},
      trace, 1, trace, Read{{packet}, &calls}));
  EXPECT_EQ(calls, 1u);
  Exact(trace, std::numeric_limits<double>::max());
  calls = 0;
  trace = 0;
  EXPECT_FALSE(cin_timestep::detail::AddRigidMemberTraceWithResponse(body, {0,0,0},
      1, 1, trace, Read{{packet}, &calls}));
  EXPECT_EQ(calls, 2u);
  Exact(trace, 0);
  const auto bad = staged::PrepareResponses(body, {0,0,0}, -1, 1);
  for (const auto& axis : bad.axis) EXPECT_FALSE(axis.valid);
}
TEST(CinCooperativeMember, InvalidInputsPreserveTraceAndEveryResponseValidation) {
  tlfea::contact::RigidContactBody good;
  good.current_frame.axes = {{1,0,0,0,1,0,0,0,1}};
  good.mass = 1;
  good.current_frame.inertia = {1, 2, 3};
  for (unsigned fault = 0; fault < 9; ++fault) {
    auto body = good;
    tl::math::Vec3 x{.5, -.25, .125};
    double k = 1, r = 2, trace = 3;
    if (fault == 0) body.mass = 0;
    if (fault == 1) body.current_frame.inertia.y = 0;
    if (fault == 2) body.current_frame.axes.v[2] = .1;
    if (fault == 3) x.x = std::numeric_limits<double>::quiet_NaN();
    if (fault == 4) body.center.z = std::numeric_limits<double>::infinity();
    if (fault == 5) k = -1;
    if (fault == 6) r = -1;
    if (fault == 7) trace = -1;
    if (fault == 8) x.z = std::numeric_limits<double>::max();
    auto old = trace, now = trace;
    const auto values = staged::PrepareResponses(body, x, k, r);
    EXPECT_FALSE(frozen::AddRigidMemberTrace(body, x, k, r, old));
    EXPECT_FALSE(cin_timestep::detail::AddRigidMemberTraceWithResponse(body, x, k, r, now,
        staged::ReadPreparedResponse{values}));
    Exact(old, now);
    Exact(now, trace);
  }
}
} // namespace tl::fea::cooperative_test
