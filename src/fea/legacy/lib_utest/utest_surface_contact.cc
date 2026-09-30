#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <iterator>
#include <limits>
#include <type_traits>

#include "../lib_src/collision/SurfaceContactLaw.h"

namespace {
namespace sc = tlfea::contact;

static_assert(std::is_trivially_copyable<sc::LinearTriangleSurfaceView>::value,
              "Surface views must copy safely to CUDA kernels");
static_assert(std::is_standard_layout<sc::LinearTriangleSurfaceView>::value,
              "Surface views must have a stable field layout");
static_assert(std::is_trivially_copyable<sc::NormalContactTrial>::value,
              "Trial records must not own hidden host state");

sc::Vec3 Cross(sc::Vec3 a, sc::Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

void ExpectVectorNear(sc::Vec3 actual, sc::Vec3 expected, double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

struct TriangleFixture {
  // Deliberately distinct position and velocity layouts, as in existing TL.
  double xyz_soa[9] = {1, 3, 1, 2, 2, 5, -1, -1, -1};
  double velocity_aos[9] = {1, 2, 4, -2, 5, 1, 7, 3, -2};
  double inverse_mass[3] = {0.5, 0.25, 0.125};
  sc::SurfaceTriangle triangle{{0, 1, 2}, 73, 42, 2, 0.0005,
                                sc::SurfaceInterpolation::kLinearTriangle};
  sc::LinearTrianglePoint point{0, {0.2, 0.3, 0.5}};

  sc::LinearTriangleSurfaceView view() const {
    return {{xyz_soa, 3, 1, 3}, {velocity_aos, 3, 3, 1}, inverse_mass, &triangle, 1};
  }
};

TEST(SurfaceContactInterpolation, PreservesForceMomentAndVirtualWork) {
  const TriangleFixture fixture;
  const auto view = fixture.view();
  sc::LinearPointKinematics point;
  ASSERT_EQ(sc::EvaluateLinearPoint(view, fixture.point, &point), sc::Status::kOk);
  ExpectVectorNear(point.position, {1.6, 3.5, -1});
  ExpectVectorNear(point.velocity, {3.1, 3.4, 0.1});

  const sc::Vec3 force{3, -4, 9};
  sc::TriangleNodalForces nodal;
  ASSERT_EQ(sc::ProjectLinearPointForce(view, fixture.point, force, &nodal),
            sc::Status::kOk);
  sc::Vec3 resultant;
  sc::Vec3 moment;
  double nodal_power = 0;
  for (int i = 0; i < 3; ++i) {
    resultant = sc::Add(resultant, nodal.forces[i]);
    moment = sc::Add(moment, Cross(view.positions.at(nodal.nodes[i]), nodal.forces[i]));
    nodal_power += sc::Dot(view.velocities.at(nodal.nodes[i]), nodal.forces[i]);
  }
  ExpectVectorNear(resultant, force);
  ExpectVectorNear(moment, Cross(point.position, force));
  EXPECT_NEAR(nodal_power, sc::Dot(point.velocity, force), 1e-12);

  // Independent finite perturbation of nodal coordinates checks the same
  // force map against point displacement, not merely against interpolation.
  TriangleFixture displaced = fixture;
  const sc::Vec3 dx[3] = {{0.003, -0.001, 0.005}, {0, 0.006, -0.002},
                         {-0.004, 0.002, 0.001}};
  double work = 0;
  for (int i = 0; i < 3; ++i) {
    displaced.xyz_soa[i] += dx[i].x;
    displaced.xyz_soa[i + 3] += dx[i].y;
    displaced.xyz_soa[i + 6] += dx[i].z;
    work += sc::Dot(nodal.forces[i], dx[i]);
  }
  sc::LinearPointKinematics after;
  ASSERT_EQ(sc::EvaluateLinearPoint(displaced.view(), displaced.point, &after),
            sc::Status::kOk);
  EXPECT_NEAR(work, sc::Dot(force, sc::Subtract(after.position, point.position)), 1e-12);
}

TEST(SurfaceContactInterpolation, LocalMassPredictsPointImpulseResponse) {
  TriangleFixture fixture;
  sc::LinearPointKinematics before;
  ASSERT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &before),
            sc::Status::kOk);
  EXPECT_NEAR(before.inverse_effective_mass, 0.07375, 1e-14);
  const sc::Vec3 unit_normal{0.6, 0, 0.8};
  const double impulse = 0.75;
  sc::TriangleNodalForces nodal_impulse;
  ASSERT_EQ(sc::ProjectLinearPointForce(fixture.view(), fixture.point,
                                       sc::Scale(unit_normal, impulse), &nodal_impulse),
            sc::Status::kOk);
  for (int i = 0; i < 3; ++i) {
    const auto dv = sc::Scale(nodal_impulse.forces[i], fixture.inverse_mass[i]);
    fixture.velocity_aos[3 * i] += dv.x;
    fixture.velocity_aos[3 * i + 1] += dv.y;
    fixture.velocity_aos[3 * i + 2] += dv.z;
  }
  sc::LinearPointKinematics after;
  ASSERT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &after),
            sc::Status::kOk);
  EXPECT_NEAR(sc::Dot(sc::Subtract(after.velocity, before.velocity), unit_normal),
              impulse * before.inverse_effective_mass, 1e-13);
}

TEST(SurfaceContactInterpolation, FixedNodesContributeZeroInverseMass) {
  TriangleFixture fixture;
  std::fill(std::begin(fixture.inverse_mass), std::end(fixture.inverse_mass), 0);
  sc::LinearPointKinematics value;
  ASSERT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &value), sc::Status::kOk);
  EXPECT_DOUBLE_EQ(value.inverse_effective_mass, 0);
  // Kinematic wall velocities still interpolate even when inverse mass is zero.
  ExpectVectorNear(value.velocity, {3.1, 3.4, 0.1});
}

TEST(SurfaceContactInterpolation, RejectsUnsupportedAndMalformedSurfaceData) {
  TriangleFixture fixture;
  sc::LinearPointKinematics value;
  fixture.triangle.interpolation = sc::SurfaceInterpolation::kUnspecified;
  EXPECT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &value),
            sc::Status::kUnsupportedInterpolation);
  fixture.triangle.interpolation = sc::SurfaceInterpolation::kLinearTriangle;
  fixture.point.weights[0] = -0.1;
  EXPECT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &value),
            sc::Status::kInvalidArgument);
  fixture.point.weights[0] = 0.21;
  EXPECT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &value),
            sc::Status::kInvalidArgument);
  fixture.point.weights[0] = 0.2;
  fixture.triangle.nodes[2] = 3;
  EXPECT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &value),
            sc::Status::kOutOfRange);
  fixture.triangle.nodes[2] = 1;
  EXPECT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &value),
            sc::Status::kInvalidArgument);
  fixture.triangle.nodes[2] = 2;
  fixture.inverse_mass[1] = -1;
  EXPECT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &value),
            sc::Status::kInvalidArgument);
  fixture.inverse_mass[1] = 0.25;
  auto missing_velocity = fixture.view();
  missing_velocity.velocities.data = nullptr;
  EXPECT_EQ(sc::EvaluateLinearPoint(missing_velocity, fixture.point, &value),
            sc::Status::kInvalidArgument);
  fixture.velocity_aos[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(sc::EvaluateLinearPoint(fixture.view(), fixture.point, &value),
            sc::Status::kInvalidArgument);
}

TEST(SurfaceContactNormalLaw, UsesStiffnessAndDampingConfigurationWithCorrectUnits) {
  const sc::NormalContactInput input{-0.002, -3, 0.5};
  sc::NormalContactResponse undamped, damped, stiffer;
  ASSERT_EQ(sc::EvaluateNormalContact({2000, 0, 0.8}, input, &undamped), sc::Status::kOk);
  ASSERT_EQ(sc::EvaluateNormalContact({2000, 0.25, 0.8}, input, &damped), sc::Status::kOk);
  ASSERT_EQ(sc::EvaluateNormalContact({8000, 0, 0.8}, input, &stiffer), sc::Status::kOk);
  EXPECT_DOUBLE_EQ(undamped.force, 4);
  EXPECT_DOUBLE_EQ(stiffer.force, 16);
  EXPECT_NEAR(damped.damping_coefficient, std::sqrt(1000.0), 1e-12);
  EXPECT_NEAR(damped.force, 4 + 3 * std::sqrt(1000.0), 1e-12);
  EXPECT_NEAR(damped.dissipated_power, 9 * std::sqrt(1000.0), 1e-12);
  EXPECT_LT(damped.stable_timestep, undamped.stable_timestep);
  EXPECT_NEAR(stiffer.stable_timestep, 0.5 * undamped.stable_timestep, 1e-14);
}

TEST(SurfaceContactNormalLaw, NoAttractionAndDampingNeverAddsEnergy) {
  for (double velocity : {-100.0, -0.1, 0.0, 0.1, 100.0}) {
    sc::NormalContactResponse response;
    ASSERT_EQ(sc::EvaluateNormalContact({1000, 0.5, 0.8}, {-0.001, velocity, 1},
                                       &response), sc::Status::kOk);
    EXPECT_GE(response.force, 0);
    EXPECT_GE(response.dissipated_power, 0);
    // d/dt (0.5*k*gap^2) + force*v = -dissipation.
    EXPECT_NEAR(-1000 * 0.001 * velocity + response.force * velocity,
                -response.dissipated_power, 1e-10);
    if (velocity == 100) {
      EXPECT_DOUBLE_EQ(response.force, 0);
    }
  }
  sc::NormalContactResponse separated;
  ASSERT_EQ(sc::EvaluateNormalContact({1000, 0.5, 0.8}, {0.01, -100, 1}, &separated),
            sc::Status::kOk);
  EXPECT_FALSE(separated.active);
  EXPECT_DOUBLE_EQ(separated.force, 0);
  EXPECT_DOUBLE_EQ(separated.dissipated_power, 0);
}

double SymplecticEulerSpectralRadius(double mass, double stiffness,
                                    double damping, double dt) {
  // Derived independently from v'=v-h(k*x+c*v)/m, x'=x+h*v'.
  const double trace = 2 - dt * damping / mass - dt * dt * stiffness / mass;
  const double determinant = 1 - dt * damping / mass;
  const auto discriminant = std::sqrt(std::complex<double>(trace * trace - 4 * determinant));
  return std::max(std::abs(0.5 * (trace + discriminant)),
                  std::abs(0.5 * (trace - discriminant)));
}

TEST(SurfaceContactNormalLaw, TimestepBoundMatchesIndependentAmplificationMatrix) {
  for (double mass : {0.1, 1.0, 7.0}) {
    for (double stiffness : {1e3, 1e6}) {
      for (double damping_ratio : {0.0, 0.1, 1.0, 3.0}) {
        SCOPED_TRACE(::testing::Message() << "m=" << mass << " k=" << stiffness
                     << " zeta=" << damping_ratio);
        sc::NormalContactResponse response;
        ASSERT_EQ(sc::EvaluateNormalContact({stiffness, damping_ratio, 0.8},
                                           {-0.001, 0, 1 / mass}, &response), sc::Status::kOk);
        EXPECT_LE(SymplecticEulerSpectralRadius(mass, stiffness, response.damping_coefficient,
                                               response.stable_timestep), 1 + 1e-12);
        EXPECT_GT(SymplecticEulerSpectralRadius(mass, stiffness, response.damping_coefficient,
                                               response.stable_timestep * 1.01 / 0.8), 1);
      }
    }
  }
}

TEST(SurfaceContactNormalLaw, RejectsInvalidAndOverflowingInputsWithoutUsableForce) {
  sc::NormalContactResponse response;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto parameters : {sc::NormalContactParameters{0, 0, 0.8}, {-1, 0, 0.8},
                                {1000, -1, 0.8}, {1000, nan, 0.8}, {1000, 0, 1},
                                {1000, 0, 0}, {inf, 0, 0.8}}) {
    EXPECT_EQ(sc::EvaluateNormalContact(parameters, {-0.1, -1, 1}, &response),
              sc::Status::kInvalidArgument);
    EXPECT_DOUBLE_EQ(response.force, 0);
    EXPECT_DOUBLE_EQ(response.stable_timestep, 0);
  }
  EXPECT_EQ(sc::EvaluateNormalContact({1000, 0, 0.8}, {-0.1, -1, 0}, &response),
            sc::Status::kNoDynamicDofs);
  for (const auto input : {sc::NormalContactInput{nan, 0, 1}, {-0.1, inf, 1}, {-0.1, 0, -1}})
    EXPECT_EQ(sc::EvaluateNormalContact({1000, 0, 0.8}, input, &response),
              sc::Status::kInvalidArgument);
  EXPECT_EQ(sc::EvaluateNormalContact({1e300, 0, 0.8}, {-1e100, 0, 1}, &response),
            sc::Status::kNonFiniteResult);
  EXPECT_DOUBLE_EQ(response.force, 0);
}

struct ImpactResult {
  double rebound_velocity = 0;
  double impulse = 0;
  double contact_duration = 0;
  double dissipated_energy = 0;
  double max_energy = 0;
  bool finished = false;
};

ImpactResult IntegrateImpact(double mass, double stiffness, double damping_ratio,
                             double dimensionless_dt) {
  const double dt = dimensionless_dt * std::sqrt(mass / stiffness);
  double gap = 0;
  double velocity = -2;
  ImpactResult result;
  for (int step = 0; step < 10000; ++step) {
    sc::NormalContactResponse response;
    if (sc::EvaluateNormalContact({stiffness, damping_ratio, 0.8},
                                 {gap, velocity, 1 / mass}, &response) != sc::Status::kOk)
      return result;
    result.max_energy = std::max(result.max_energy,
                                0.5 * mass * velocity * velocity + response.elastic_energy);
    result.impulse += dt * response.force;
    result.dissipated_energy += dt * response.dissipated_power;
    velocity += dt * response.force / mass;
    gap += dt * velocity;
    result.contact_duration += dt;
    if (gap >= 0 && velocity > 0) {
      result.finished = true;
      result.rebound_velocity = velocity;
      return result;
    }
  }
  return result;
}

TEST(SurfaceContactImpact, UndampedImpactMatchesAnalyticalImpulseAndContactDuration) {
  constexpr double pi = 3.14159265358979323846;
  for (double mass : {0.1, 1.0, 7.0}) {
    for (double stiffness : {1e3, 1e6}) {
      const double dimensionless_dt = 0.01;
      const auto result = IntegrateImpact(mass, stiffness, 0, dimensionless_dt);
      ASSERT_TRUE(result.finished);
      EXPECT_NEAR(result.rebound_velocity, 2, 2e-4);
      EXPECT_NEAR(result.impulse, 4 * mass, 2e-4 * mass);
      EXPECT_NEAR(result.impulse, mass * (result.rebound_velocity + 2), 1e-11);
      EXPECT_NEAR(result.contact_duration, pi * std::sqrt(mass / stiffness),
                  2 * dimensionless_dt * std::sqrt(mass / stiffness));
      EXPECT_LE(result.max_energy, 2 * mass * 1.01);
      EXPECT_DOUBLE_EQ(result.dissipated_energy, 0);
    }
  }
}

TEST(SurfaceContactImpact, DampedImpactEnergyBalanceConvergesWithSmallerStep) {
  const double mass = 2;
  const double initial_energy = 0.5 * mass * 4;
  const auto coarse = IntegrateImpact(mass, 8000, 0.2, 0.02);
  const auto fine = IntegrateImpact(mass, 8000, 0.2, 0.005);
  ASSERT_TRUE(coarse.finished);
  ASSERT_TRUE(fine.finished);
  const double coarse_error = std::abs(0.5 * mass * coarse.rebound_velocity * coarse.rebound_velocity +
                                       coarse.dissipated_energy - initial_energy);
  const double fine_error = std::abs(0.5 * mass * fine.rebound_velocity * fine.rebound_velocity +
                                     fine.dissipated_energy - initial_energy);
  EXPECT_GT(fine.dissipated_energy, 0);
  EXPECT_LT(fine.rebound_velocity, 2);
  EXPECT_LT(fine_error, 0.02 * initial_energy);
  EXPECT_LT(fine_error, coarse_error * 0.4);
  EXPECT_NEAR(fine.impulse, mass * (fine.rebound_velocity + 2), 1e-11);
}

TEST(SurfaceContactTrial, RepeatedEvaluationsDoNotAdvanceCommittedHistory) {
  sc::NormalContactState committed;
  sc::NormalContactResponse response;
  ASSERT_EQ(sc::EvaluateNormalContact({1000, 0.2, 0.8}, {-0.001, -1, 1}, &response),
            sc::Status::kOk);
  sc::NormalContactTrial first, second;
  ASSERT_EQ(sc::MakeNormalContactTrial(committed, response, 0.001, &first), sc::Status::kOk);
  ASSERT_EQ(sc::MakeNormalContactTrial(committed, response, 0.001, &second), sc::Status::kOk);
  EXPECT_EQ(committed.revision, 0u);
  EXPECT_DOUBLE_EQ(committed.accepted_time, 0);
  EXPECT_DOUBLE_EQ(committed.dissipated_energy, 0);
  EXPECT_DOUBLE_EQ(first.candidate.dissipated_energy, second.candidate.dissipated_energy);
  ASSERT_EQ(sc::CommitNormalContactTrial(&committed, &first), sc::Status::kOk);
  EXPECT_EQ(committed.revision, 1u);
  EXPECT_DOUBLE_EQ(committed.accepted_time, 0.001);
  EXPECT_NEAR(committed.dissipated_energy, response.dissipated_power * 0.001, 1e-14);
  EXPECT_EQ(sc::CommitNormalContactTrial(&committed, &first), sc::Status::kNoTrial);
  EXPECT_EQ(sc::CommitNormalContactTrial(&committed, &second), sc::Status::kStaleTrial);
}

TEST(SurfaceContactTrial, RejectedTrialCanRetryWithSmallerStep) {
  sc::NormalContactState committed{4, 0.01, 0.03};
  sc::NormalContactResponse response;
  ASSERT_EQ(sc::EvaluateNormalContact({1000, 0.2, 0.8}, {-0.001, -1, 1}, &response),
            sc::Status::kOk);
  sc::NormalContactTrial rejected;
  ASSERT_EQ(sc::MakeNormalContactTrial(committed, response, 0.001, &rejected), sc::Status::kOk);
  sc::DiscardNormalContactTrial(&rejected);
  EXPECT_EQ(sc::CommitNormalContactTrial(&committed, &rejected), sc::Status::kNoTrial);
  EXPECT_EQ(committed.revision, 4u);
  EXPECT_DOUBLE_EQ(committed.dissipated_energy, 0.03);
  sc::NormalContactTrial retry;
  ASSERT_EQ(sc::MakeNormalContactTrial(committed, response, 0.0005, &retry), sc::Status::kOk);
  ASSERT_EQ(sc::CommitNormalContactTrial(&committed, &retry), sc::Status::kOk);
  EXPECT_EQ(committed.revision, 5u);
  EXPECT_NEAR(committed.accepted_time, 0.0105, 1e-16);
  EXPECT_NEAR(committed.dissipated_energy, 0.03 + response.dissipated_power * 0.0005, 1e-14);
}

TEST(SurfaceContactTrial, InvalidTrialDoesNotBecomeCommittable) {
  sc::NormalContactState committed;
  sc::NormalContactResponse response;
  sc::NormalContactTrial trial;
  EXPECT_EQ(sc::MakeNormalContactTrial(committed, response, -1, &trial),
            sc::Status::kInvalidArgument);
  EXPECT_FALSE(trial.valid);
  EXPECT_EQ(sc::CommitNormalContactTrial(&committed, &trial), sc::Status::kNoTrial);
  response.dissipated_power = std::numeric_limits<double>::infinity();
  EXPECT_EQ(sc::MakeNormalContactTrial(committed, response, 0.001, &trial),
            sc::Status::kInvalidArgument);
  EXPECT_FALSE(trial.valid);
}

}  // namespace
