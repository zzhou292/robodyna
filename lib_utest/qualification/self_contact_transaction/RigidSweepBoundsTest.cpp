// SPDX-License-Identifier: MIT
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
namespace c = tlfea::contact;
namespace fe = tl::fea;
namespace sct = tlfea::contact::self_contact_transaction;

constexpr auto Trajectory =
    fe::NodalRigidMemberTrajectory::
        EndpointCorrectedSecondOrderDriftV1;

fe::NodalRigidGroupSnapshot Group(
    c::Vec3 center, c::Vec3 omega = {}) {
  fe::NodalRigidGroupSnapshot result;
  result.source_group_id = 41;
  result.source_node_set_id = 43;
  result.source_kind = fe::RigidBindingSourceKind::Part;
  result.state.center = {center.x, center.y, center.z};
  result.state.omega = {omega.x, omega.y, omega.z};
  return result;
}

c::Vec3 TestCross(c::Vec3 first, c::Vec3 second) {
  return {
      first.y * second.z - first.z * second.y,
      first.z * second.x - first.x * second.z,
      first.x * second.y - first.y * second.x};
}

c::Vec3 TestSubtract(c::Vec3 first, c::Vec3 second) {
  return {
      first.x - second.x,
      first.y - second.y,
      first.z - second.z};
}

c::VectorView View(const double* values, std::uint32_t nodes) {
  return {values, nodes, 3, 1};
}

c::WeightedSurfacePoint Point(
    std::uint32_t a, double wa,
    std::uint32_t b, double wb,
    std::uint32_t c_node, double wc) {
  c::WeightedSurfacePoint point;
  point.count = 3;
  point.nodes[0] = a;
  point.nodes[1] = b;
  point.nodes[2] = c_node;
  point.weights[0] = wa;
  point.weights[1] = wb;
  point.weights[2] = wc;
  return point;
}

double Component(c::Vec3 value, unsigned component) {
  return component == 0 ? value.x :
      (component == 1 ? value.y : value.z);
}

c::Vec3 OwnerQuadraticEndpoint(
    c::Vec3 first, c::Vec3 first_center,
    c::Vec3 second_center, c::Vec3 omega, double duration) {
  const auto arm = TestSubtract(first, first_center);
  const auto tangent = TestCross(omega, arm);
  const auto curvature = TestCross(omega, tangent);
  return {
      second_center.x + arm.x + duration * tangent.x +
          .5 * duration * duration * curvature.x,
      second_center.y + arm.y + duration * tangent.y +
          .5 * duration * duration * curvature.y,
      second_center.z + arm.z + duration * tangent.z +
          .5 * duration * duration * curvature.z};
}

long double PathComponent(
    c::Vec3 first, c::Vec3 second, c::Vec3 first_center,
    c::Vec3 omega, double duration, long double u,
    unsigned component) {
  const auto arm = TestSubtract(first, first_center);
  const auto curvature =
      TestCross(omega, TestCross(omega, arm));
  const long double chord =
      (1 - u) * Component(first, component) +
      u * Component(second, component);
  return chord - .5L * u * (1 - u) *
      static_cast<long double>(duration) * duration *
      Component(curvature, component);
}

TEST(SelfContactRigidSweepBounds,
     ZeroSpinIsTightOutwardRoundedTranslation) {
  const c::Vec3 first{100, -2, 7};
  const c::Vec3 second{100.25, -1.5, 6.75};
  const auto accepted = Group({90, 3, 4});
  const auto prepared = Group({90.25, 3.5, 3.75});
  c::SelfContactSweptParentBounds bounds;
  ASSERT_EQ(sct::BuildRigidMemberSweepBounds(
      first, second, accepted, prepared, Trajectory, .01, &bounds),
      sct::RigidMemberSweepStatus::Ok);
  for (unsigned component = 0; component < 3; ++component) {
    const double lower = std::min(
        Component(first, component), Component(second, component));
    const double upper = std::max(
        Component(first, component), Component(second, component));
    EXPECT_EQ(Component(bounds.lower, component),
              std::nextafter(
                  lower, -std::numeric_limits<double>::infinity()));
    EXPECT_EQ(Component(bounds.upper, component),
              std::nextafter(
                  upper, std::numeric_limits<double>::infinity()));
  }
}

TEST(SelfContactRigidSweepBounds,
     TinyAngleInflationScalesQuadraticallyPerMember) {
  const c::Vec3 first{2, 1, 0};
  constexpr double Duration = .1;
  const auto accepted = Group({});
  const auto make = [&](double spin) {
    const c::Vec3 omega{0, 0, spin};
    const auto prepared = Group({}, omega);
    const auto second = OwnerQuadraticEndpoint(
        first, {}, {}, omega, Duration);
    c::SelfContactSweptParentBounds bounds;
    EXPECT_EQ(sct::BuildRigidMemberSweepBounds(
        first, second, accepted, prepared,
        Trajectory, Duration, &bounds),
        sct::RigidMemberSweepStatus::Ok);
    return bounds.upper.x - std::max(first.x, second.x);
  };
  const double first_excess = make(.01);
  const double second_excess = make(.02);
  EXPECT_GT(first_excess, 0);
  EXPECT_LT(first_excess, 3e-7);
  EXPECT_NEAR(second_excess / first_excess, 4, 2e-7);
}

TEST(SelfContactRigidSweepBounds,
     EndpointsAndCompleteCertifiedQuadraticStayContained) {
  const c::Vec3 first{4.125, -2.75, 1.5};
  const c::Vec3 first_center{1, -1, .25};
  const c::Vec3 second_center{1.01, -.98, .23};
  const c::Vec3 omega{.031, -.017, .043};
  constexpr double Duration = .02;
  const auto accepted = Group(first_center);
  const auto prepared = Group(second_center, omega);
  const auto recurrence_endpoint = OwnerQuadraticEndpoint(
      first, first_center, second_center, omega, Duration);
  // Retain a representable recurrence residual as the two-member/binary64
  // endpoint-correction term of the authenticated trajectory.
  const c::Vec3 second{
      std::nextafter(recurrence_endpoint.x, 10.),
      std::nextafter(recurrence_endpoint.y, -10.),
      std::nextafter(recurrence_endpoint.z, 10.)};
  c::SelfContactSweptParentBounds bounds;
  ASSERT_EQ(sct::BuildRigidMemberSweepBounds(
      first, second, accepted, prepared,
      Trajectory, Duration, &bounds),
      sct::RigidMemberSweepStatus::Ok);
  for (unsigned component = 0; component < 3; ++component) {
    EXPECT_LT(Component(bounds.lower, component),
              std::min(
                  Component(first, component),
                  Component(second, component)));
    EXPECT_GT(Component(bounds.upper, component),
              std::max(
                  Component(first, component),
                  Component(second, component)));
  }
  for (unsigned sample = 0; sample <= 1000; ++sample) {
    const long double u =
        static_cast<long double>(sample) / 1000;
    for (unsigned component = 0; component < 3; ++component) {
      const long double value = PathComponent(
          first, second, first_center, omega,
          Duration, u, component);
      EXPECT_LE(
          static_cast<long double>(Component(bounds.lower, component)),
          value);
      EXPECT_GE(
          static_cast<long double>(Component(bounds.upper, component)),
          value);
    }
  }
}

TEST(SelfContactRigidSweepBounds,
     UnsupportedRotationAndNonfiniteInputsLeaveOutputUnchanged) {
  const auto accepted = Group({});
  auto prepared = Group({}, {0, 0, 40});
  const c::SelfContactSweptParentBounds unchanged{
      {11, 12, 13}, {14, 15, 16}};
  auto output = unchanged;
  EXPECT_EQ(sct::BuildRigidMemberSweepBounds(
      {1, 0, 0}, {1, 0, 0}, accepted, prepared,
      Trajectory, .1, &output),
      sct::RigidMemberSweepStatus::RotationLimit);
  EXPECT_EQ(output.lower.x, unchanged.lower.x);
  EXPECT_EQ(output.upper.z, unchanged.upper.z);

  prepared = Group({}, {0, 0, .1});
  EXPECT_EQ(sct::BuildRigidMemberSweepBounds(
      {1, 0, 0}, {1, 0, 0}, accepted, prepared,
      fe::NodalRigidMemberTrajectory::None, .1, &output),
      sct::RigidMemberSweepStatus::UnsupportedTrajectory);
  auto nonfinite = accepted;
  nonfinite.state.center.x =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(sct::BuildRigidMemberSweepBounds(
      {1, 0, 0}, {1, 0, 0}, nonfinite, prepared,
      Trajectory, .1, &output),
      sct::RigidMemberSweepStatus::Ok);
  EXPECT_NE(sct::BuildRigidMemberSweepBounds(
      {std::numeric_limits<double>::infinity(), 0, 0},
      {1, 0, 0}, accepted, prepared,
      Trajectory, .1, &output),
      sct::RigidMemberSweepStatus::Ok);
  EXPECT_EQ(output.lower.x, unchanged.lower.x);
  EXPECT_EQ(output.upper.z, unchanged.upper.z);
}

TEST(SelfContactRigidSweepBounds,
     MotionCertificateIsPartOfPreparedOwnerIdentity) {
  fe::NodalPreparedView certified;
  certified.rigid_member_trajectory = Trajectory;
  auto forged = certified;
  forged.rigid_member_trajectory =
      fe::NodalRigidMemberTrajectory::None;
  EXPECT_TRUE(fe::trial_identity::SamePrepared(
      certified, certified));
  EXPECT_FALSE(fe::trial_identity::SamePrepared(
      certified, forged));
}

TEST(SelfContactRigidSweepBounds,
     ExactRepresentedAffineCertificateComposesWeightedCurvature) {
  // Nodes 0 and 1 have opposite nonzero rigid curvature about the x axis;
  // node 2 is ordinary.  Their weighted represented point cancels exactly.
  const double accepted_values[]{
      0, 1, 0,
      0, -1, 0,
      4, 0, 0,
      2, 0, 0};
  const double prepared_values[]{
      .01, 1, 0,
      .01, -1, 0,
      4.01, 0, 0,
      2.01, 0, 0};
  const std::uint32_t node_groups[]{0, 0, UINT32_MAX, 0};
  const auto accepted_group = Group({});
  const auto prepared_group = Group({.01, 0, 0}, {1, 0, 0});
  bool affine = false;

  const auto weighted =
      Point(0, .5, 1, .5, 2, 0);
  ASSERT_EQ(sct::CertifyRigidPointAffineMotion(
      weighted, View(accepted_values, 4), View(prepared_values, 4),
      node_groups, &accepted_group, &prepared_group, 1,
      Trajectory, .01, &affine), sct::RigidMemberSweepStatus::Ok);
  EXPECT_TRUE(affine);

  // A nonzero spin with an exactly parallel arm also has q=0.
  const auto parallel = Point(3, 1, 2, 0, 1, 0);
  ASSERT_EQ(sct::CertifyRigidPointAffineMotion(
      parallel, View(accepted_values, 4), View(prepared_values, 4),
      node_groups, &accepted_group, &prepared_group, 1,
      Trajectory, .01, &affine), sct::RigidMemberSweepStatus::Ok);
  EXPECT_TRUE(affine);

  // One uncancelled component remains nonlinear and must fail closed.
  const auto curved = Point(0, 1, 1, 0, 2, 0);
  ASSERT_EQ(sct::CertifyRigidPointAffineMotion(
      curved, View(accepted_values, 4), View(prepared_values, 4),
      node_groups, &accepted_group, &prepared_group, 1,
      Trajectory, .01, &affine), sct::RigidMemberSweepStatus::Ok);
  EXPECT_FALSE(affine);

  c::FixedContactFacet facet;
  facet.vertices[0] = weighted;
  facet.vertices[1] = parallel;
  // Ordinary + zero-spin/mixed composition is affine.  Use a zero-spin
  // snapshot for the third represented point.
  facet.vertices[2] = Point(3, .25, 2, .75, 1, 0);
  auto zero_spin = prepared_group;
  zero_spin.state.omega = {};
  ASSERT_EQ(sct::CertifyRigidFacetAffineMotion(
      facet, View(accepted_values, 4), View(prepared_values, 4),
      node_groups, &accepted_group, &zero_spin, 1,
      Trajectory, .01, &affine), sct::RigidMemberSweepStatus::Ok);
  EXPECT_TRUE(affine);

  facet.vertices[2] = curved;
  ASSERT_EQ(sct::CertifyRigidFacetAffineMotion(
      facet, View(accepted_values, 4), View(prepared_values, 4),
      node_groups, &accepted_group, &prepared_group, 1,
      Trajectory, .01, &affine), sct::RigidMemberSweepStatus::Ok);
  EXPECT_FALSE(affine);
}

}  // namespace
