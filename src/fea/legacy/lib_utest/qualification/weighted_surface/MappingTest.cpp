#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include "lib_src/collision/weighted_surface/Adapters.h"

namespace {
using namespace tlfea::contact;
void Same(Vec3 a, Vec3 b) {
  EXPECT_EQ(std::memcmp(&a, &b, sizeof(a)), 0);
}
struct Fixture {
  double x[12]{1, 2, 3, -2, 4, 7, 5, -3, 9, 8, 6, -4};
  double v[12]{2, -1, 3, 5, 7, 4, -2, 8, 1, 0, -4, 6};
  double inverse[4]{1, 2, 3, 4};
  VectorView positions() { return {x, 4, 3, 1}; }
  VectorView velocities() { return {v, 4, 1, 4}; }
};
TEST(WeightedSurface, NativeAdaptersPreserveExistingArithmeticAndWork) {
  Fixture f;
  SurfaceQ4 parent{{2, 0, 3, 1}, 1, 2, 0, 0};
  const Q4SurfaceView surface{f.positions(), f.velocities(), &parent, 1};
  for (double u : {-1., -.3, 0., .7, 1.}) {
    for (double v : {-1., -.2, 0., .6, 1.}) {
      WeightedSurfacePoint point;
      ASSERT_EQ(MakeWeightedQ4Point(parent.nodes, 4, u, v, &point), Status::kOk);
      WeightedPointKinematics current;
      Q4PointKinematics old;
      ASSERT_EQ(EvaluateWeightedSurfacePoint(f.positions(), f.velocities(), point, &current), Status::kOk);
      ASSERT_EQ(EvaluateQ4Point(surface, {0, u, v}, &old), Status::kOk);
      Same(current.position, old.position);
      Same(current.velocity, old.velocity);
      WeightedNodalForces projected;
      Q4NodalForces old_forces;
      const Vec3 force{3, -4, 5};
      ASSERT_EQ(ProjectWeightedSurfaceForce(point, 4, force, &projected), Status::kOk);
      ASSERT_EQ(ProjectQ4PointForce(surface, {0, u, v}, force, &old_forces), Status::kOk);
      double work = 0;
      for (unsigned i = 0; i < 4; ++i) {
        Same(projected.forces[i], old_forces.forces[i]);
        Same(projected.couples[i], {});
        work += Dot(projected.forces[i], f.velocities().at(point.nodes[i]));
      }
      EXPECT_NEAR(work, Dot(force, current.velocity), 1e-13);
    }
  }
  SurfaceTriangle triangle{{2, 0, 1}, 1, 2, 0, .1, SurfaceInterpolation::kLinearTriangle};
  LinearTrianglePoint old_point{0, {.125, .375, .5}};
  LinearTriangleSurfaceView triangle_view{f.positions(), f.velocities(), f.inverse, &triangle, 1};
  WeightedSurfacePoint point;
  ASSERT_EQ(MakeWeightedT3Point(triangle.nodes, 4, old_point.weights, &point), Status::kOk);
  WeightedPointKinematics current;
  LinearPointKinematics old;
  ASSERT_EQ(EvaluateWeightedSurfacePoint(f.positions(), f.velocities(), point, &current), Status::kOk);
  ASSERT_EQ(EvaluateLinearPoint(triangle_view, old_point, &old), Status::kOk);
  Same(current.position, old.position);
  Same(current.velocity, old.velocity);
}
TEST(WeightedSurface, ComposedFacetWeightsAreNotMeanParameterShapes) {
  // Three fixed physical-facet vertices: (+,+), (-,+), (-,-).
  WeightedSurfacePoint point{{0, 1, 2, 3}, {.25, .25, .5, 0}, 4};
  double x[12]{0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  Vec3 result;
  ASSERT_EQ(EvaluateWeightedSurfacePosition({x, 4, 3, 1}, point, &result), Status::kOk);
  EXPECT_DOUBLE_EQ(result.z, 1);
  WeightedSurfacePoint averaged;
  ASSERT_EQ(MakeWeightedQ4Point(point.nodes, 4, -.5, 0, &averaged), Status::kOk);
  Vec3 wrong;
  ASSERT_EQ(EvaluateWeightedSurfacePosition({x, 4, 3, 1}, averaged, &wrong), Status::kOk);
  EXPECT_DOUBLE_EQ(wrong.z, .5);
}
TEST(WeightedSurface, LateFailureAndInvalidTopologyPreserveOutput) {
  Fixture f;
  WeightedSurfacePoint point{{0, 1, 2, 3}, {1, 0, 0, 0}, 4};
  WeightedPointKinematics output{{11, 12, 13}, {14, 15, 16}};
  const auto before = output;
  f.v[11] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(EvaluateWeightedSurfacePoint(f.positions(), f.velocities(), point, &output), Status::kInvalidArgument);
  EXPECT_EQ(std::memcmp(&output, &before, sizeof(output)), 0);
  Vec3 position;
  EXPECT_EQ(EvaluateWeightedSurfacePosition(f.positions(), point, &position), Status::kOk);
  point.nodes[3] = 0;
  EXPECT_EQ(ValidateWeightedSurfacePoint(point, 4), Status::kInvalidArgument);
  point.nodes[3] = 4;
  EXPECT_EQ(ValidateWeightedSurfacePoint(point, 4), Status::kOutOfRange);
  point.nodes[3] = 3;
  point.weights[0] = 1.0001;
  EXPECT_EQ(ValidateWeightedSurfacePoint(point, 4), Status::kInvalidArgument);
  point.weights[0] = .9;
  EXPECT_EQ(ValidateWeightedSurfacePoint(point, 4), Status::kInvalidArgument);
  point.count = 2;
  EXPECT_EQ(ValidateWeightedSurfacePoint(point, 4), Status::kInvalidArgument);
}
TEST(WeightedSurface, SignedZeroSubnormalAndBorrowedViewBounds) {
  const double denorm = std::numeric_limits<double>::denorm_min();
  WeightedSurfacePoint point{{0, 1, 2, 3}, {1, -0., 0, 0}, 4};
  double x[12]{denorm, -0., 0};
  Vec3 result{1, 2, 3};
  ASSERT_EQ(EvaluateWeightedSurfacePosition({x, 4, 3, 1}, point, &result), Status::kOk);
  EXPECT_EQ(result.x, denorm);
  const auto before = result;
  EXPECT_EQ(EvaluateWeightedSurfacePosition({x, 4, UINT64_MAX, 1}, point, &result), Status::kInvalidArgument);
  EXPECT_EQ(std::memcmp(&result, &before, sizeof(result)), 0);
}
} // namespace
