#include "lib_src/elements/ReissnerShellMass.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
namespace shell = tl::fea::reissner;
using Policy = shell::ShellDrillingInertiaPolicy;
using Status = shell::ShellMassStatus;
constexpr Policy kCoupon = Policy::kEqualPhysicalTangential;
constexpr double kRelative = 3e-13;

struct Fixture {
  shell::ShellReference reference;
  shell::ElasticSection section;
};
Fixture Rectangle(double width = .4, double height = .2, double thickness = .008, double density = 7890) {
  Fixture f;
  constexpr double sign[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
  const double a = 1 / std::sqrt(3.0);
  const double quadrature[4][2] = {{-a, -a}, {a, -a}, {a, a}, {-a, a}};
  for (unsigned n = 0; n < 4; ++n)
    f.reference.initial_position[n] = {.5 * width * sign[n][0], .5 * height * sign[n][1], 0};
  for (unsigned p = 0; p < 4; ++p) {
    auto& point = f.reference.gauss[p];
    point.area_weight = width * height / 4;
    point.natural[0] = quadrature[p][0];
    point.natural[1] = quadrature[p][1];
    for (unsigned n = 0; n < 4; ++n)
      point.shape[n] = .25 * (1 + sign[n][0] * quadrature[p][0]) * (1 + sign[n][1] * quadrature[p][1]);
  }
  f.reference.prepared = true;
  f.section.thickness = thickness;
  f.section.density = density;
  f.section.prepared = true;
  // Other force fields deliberately remain unused: this tests only the
  // mass-bearing setup contract, not a replacement for the Chrono adapter.
  return f;
}
shell::ShellMass Mass(const Fixture& f, Policy policy = kCoupon) {
  shell::ShellMass result;
  EXPECT_EQ(shell::ComputeShellMass(f.reference, f.section, policy, result), Status::kSuccess);
  return result;
}
void Near(double actual, double expected) {
  EXPECT_NEAR(actual, expected, kRelative * std::abs(expected));
}
void Unchanged(const shell::ShellMass& actual, const shell::ShellMass& before) {
  EXPECT_EQ(actual.drilling_policy, before.drilling_policy);
  for (unsigned n = 0; n < 4; ++n) {
    const auto& a = actual.node[n];
    const auto& b = before.node[n];
    EXPECT_EQ(std::memcmp(&a.area, &b.area, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&a.mass, &b.mass, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&a.physical_tangential_inertia, &b.physical_tangential_inertia, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&a.artificial_drilling_inertia, &b.artificial_drilling_inertia, sizeof(double)), 0);
  }
}
void Unchanged(const shell::ShellKineticEnergy& a, const shell::ShellKineticEnergy& b) {
  EXPECT_EQ(std::memcmp(&a.translation, &b.translation, sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&a.physical_rotation, &b.physical_rotation, sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&a.artificial_drilling, &b.artificial_drilling, sizeof(double)), 0);
}

TEST(ReissnerShellMass, MatchesIndependentVolumeAndThicknessMomentIntegrals) {
  const auto f = Rectangle();
  const auto mass = Mass(f);
  // Independent tensor-product Simpson volume integral, including actual z^2.
  // The production operation instead uses supplied surface Gauss weights and
  // the analytic centered-thickness formula.
  const double coordinate[3] = {-1, 0, 1}, weight[3] = {1, 4, 1};
  double volume_mass = 0, thickness_moment = 0;
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      for (unsigned k = 0; k < 3; ++k) {
        const double z = .5 * f.section.thickness * coordinate[k];
        const double dm = f.section.density * .4 * .2 * f.section.thickness / 216 * weight[i] * weight[j] * weight[k];
        volume_mass += dm;
        thickness_moment += dm * z * z;
      }
  for (const auto& node : mass.node) {
    EXPECT_GT(node.area, 0);
    EXPECT_GT(node.mass, 0);
    EXPECT_GT(node.physical_tangential_inertia, 0);
    Near(node.area, .4 * .2 / 4);
    Near(node.mass, volume_mass / 4);
    Near(node.physical_tangential_inertia, thickness_moment / 4);
    EXPECT_DOUBLE_EQ(node.artificial_drilling_inertia, node.physical_tangential_inertia);
  }
  const auto physical_only = Mass(f, Policy::kNone);
  for (unsigned n = 0; n < 4; ++n) {
    EXPECT_DOUBLE_EQ(physical_only.node[n].mass, mass.node[n].mass);
    EXPECT_DOUBLE_EQ(physical_only.node[n].physical_tangential_inertia, mass.node[n].physical_tangential_inertia);
    EXPECT_DOUBLE_EQ(physical_only.node[n].artificial_drilling_inertia, 0);
  }
}

TEST(ReissnerShellMass, HasIndependentAreaDensityAndCubicThicknessScaling) {
  const auto baseline = Mass(Rectangle());
  const auto area = Mass(Rectangle(.8, .6));
  const auto density = Mass(Rectangle(.4, .2, .008, 3 * 7890));
  const auto thickness = Mass(Rectangle(.4, .2, .016));
  for (unsigned n = 0; n < 4; ++n) {
    Near(area.node[n].area, 6 * baseline.node[n].area);
    Near(area.node[n].mass, 6 * baseline.node[n].mass);
    Near(area.node[n].physical_tangential_inertia, 6 * baseline.node[n].physical_tangential_inertia);
    Near(density.node[n].mass, 3 * baseline.node[n].mass);
    Near(density.node[n].physical_tangential_inertia, 3 * baseline.node[n].physical_tangential_inertia);
    Near(thickness.node[n].mass, 2 * baseline.node[n].mass);
    Near(thickness.node[n].physical_tangential_inertia, 8 * baseline.node[n].physical_tangential_inertia);
  }
}

TEST(ReissnerShellMass, PhysicalAndArtificialKineticEnergyRemainSeparate) {
  const auto f = Rectangle();
  shell::Vec3 director[4], velocity[4], omega[4];
  for (unsigned n = 0; n < 4; ++n) {
    director[n] = {0, 0, 1};
    velocity[n] = {2, -3, 4};
    omega[n] = {5, -6, 7};
  }
  const double total_mass = .4 * .2 * .008 * 7890;
  const double total_j = total_mass * .008 * .008 / 12;
  for (const auto policy : {Policy::kNone, kCoupon}) {
    shell::ShellKineticEnergy energy;
    ASSERT_EQ(shell::ComputeShellKineticEnergy(Mass(f, policy), director, velocity, omega, energy), Status::kSuccess);
    Near(energy.translation, .5 * total_mass * (4 + 9 + 16));
    Near(energy.physical_rotation, .5 * total_j * (25 + 36));
    if (policy == Policy::kNone) EXPECT_DOUBLE_EQ(energy.artificial_drilling, 0);
    else Near(energy.artificial_drilling, .5 * total_j * 49);
  }
  for (auto& spin : omega) spin = {0, 0, 7};
  shell::ShellKineticEnergy pure_drilling;
  ASSERT_EQ(shell::ComputeShellKineticEnergy(Mass(f), director, velocity, omega, pure_drilling), Status::kSuccess);
  EXPECT_DOUBLE_EQ(pure_drilling.physical_rotation, 0);
  Near(pure_drilling.artificial_drilling, .5 * total_j * 49);
}

TEST(ReissnerShellMass, WorldTensorAndEnergyCovaryWithReferenceFrame) {
  const auto f = Rectangle();
  auto rotated = f;
  // Explicit proper coordinate permutation: x -> y, y -> z, z -> x.
  const auto rotate = [](shell::Vec3 v) { return shell::Vec3{v.z, v.x, v.y}; };
  const shell::Matrix3 r{{0, 0, 1, 1, 0, 0, 0, 1, 0}};
  for (auto& point : rotated.reference.initial_position) point = rotate(point);
  const auto mass = Mass(f), rotated_mass = Mass(rotated);
  Unchanged(rotated_mass, mass);
  shell::Vec3 d[4], v[4], w[4], rd[4], rv[4], rw[4];
  for (unsigned n = 0; n < 4; ++n) {
    d[n] = {1. / 3, 2. / 3, 2. / 3};
    v[n] = {.1 + n, -.3, .7};
    w[n] = {-.5, 1.3 + n, .2};
    rd[n] = rotate(d[n]); rv[n] = rotate(v[n]); rw[n] = rotate(w[n]);
    shell::ShellInertiaTensor original, transformed;
    ASSERT_EQ(shell::ComputeShellInertiaTensor(mass.node[n], d[n], original), Status::kSuccess);
    ASSERT_EQ(shell::ComputeShellInertiaTensor(mass.node[n], rd[n], transformed), Status::kSuccess);
    const auto expected_physical = shell::detail::Product(shell::detail::Product(r, original.physical), shell::detail::Transpose(r));
    const auto expected_artificial = shell::detail::Product(shell::detail::Product(r, original.artificial), shell::detail::Transpose(r));
    for (unsigned i = 0; i < 9; ++i) {
      EXPECT_NEAR(transformed.physical.v[i], expected_physical.v[i], 1e-15 * mass.node[n].physical_tangential_inertia);
      EXPECT_NEAR(transformed.artificial.v[i], expected_artificial.v[i], 1e-15 * mass.node[n].physical_tangential_inertia);
      EXPECT_NEAR(original.physical.v[i] + original.artificial.v[i],
                  i % 4 == 0 ? mass.node[n].physical_tangential_inertia : 0,
                  1e-15 * mass.node[n].physical_tangential_inertia);
    }
    const auto along_director = shell::detail::Product(original.physical, d[n]);
    EXPECT_NEAR(shell::detail::Dot(d[n], along_director), 0, 1e-15 * mass.node[n].physical_tangential_inertia);
  }
  shell::ShellKineticEnergy energy, rotated_energy;
  ASSERT_EQ(shell::ComputeShellKineticEnergy(mass, d, v, w, energy), Status::kSuccess);
  ASSERT_EQ(shell::ComputeShellKineticEnergy(rotated_mass, rd, rv, rw, rotated_energy), Status::kSuccess);
  Near(rotated_energy.translation, energy.translation);
  Near(rotated_energy.physical_rotation, energy.physical_rotation);
  Near(rotated_energy.artificial_drilling, energy.artificial_drilling);
}

TEST(ReissnerShellMass, TwoElementsShareSixNodesWithoutDoubleCountingMassOrEnergy) {
  const auto left = Mass(Rectangle(.4, .2, .008));
  const auto right = Mass(Rectangle(.6, .2, .012));
  const std::size_t connectivity[2][4] = {{0, 1, 2, 3}, {4, 0, 3, 5}};
  const shell::ShellMass elements[2] = {left, right};
  double assembled_mass[6]{}, assembled_j[6]{};
  for (unsigned e = 0; e < 2; ++e)
    for (unsigned n = 0; n < 4; ++n) {
      assembled_mass[connectivity[e][n]] += elements[e].node[n].mass;
      assembled_j[connectivity[e][n]] += elements[e].node[n].physical_tangential_inertia;
    }
  const double m_left = .4 * .2 * .008 * 7890, m_right = .6 * .2 * .012 * 7890;
  const double j_total = (m_left * .008 * .008 + m_right * .012 * .012) / 12;
  const double expected[6] = {(m_left + m_right) / 4, m_left / 4, m_left / 4,
                             (m_left + m_right) / 4, m_right / 4, m_right / 4};
  double total_mass = 0, global_energy = 0;
  for (unsigned n = 0; n < 6; ++n) {
    Near(assembled_mass[n], expected[n]);
    total_mass += assembled_mass[n];
    // One global physical-node ledger. v=(2,0,0), omega=(3,4,5), d=(0,0,1).
    global_energy += .5 * assembled_mass[n] * 4 + .5 * assembled_j[n] * 50;
  }
  Near(total_mass, m_left + m_right);
  Near(global_energy, .5 * (m_left + m_right) * 4 + .5 * j_total * 50);
}

TEST(ReissnerShellMass, DistinctDirectorContributorsRetainPhysicalTensorAndEnergy) {
  const auto left = Mass(Rectangle(.4, .2, .008));
  const auto right = Mass(Rectangle(.6, .2, .012));
  const double jl = left.node[0].physical_tangential_inertia;
  const double jr = right.node[1].physical_tangential_inertia;
  shell::ShellInertiaTensor along_z, along_x;
  ASSERT_EQ(shell::ComputeShellInertiaTensor(left.node[0], {0, 0, 1}, along_z), Status::kSuccess);
  ASSERT_EQ(shell::ComputeShellInertiaTensor(right.node[1], {1, 0, 0}, along_x), Status::kSuccess);
  const auto physical = shell::detail::Add(along_z.physical, along_x.physical);
  const auto artificial = shell::detail::Add(along_z.artificial, along_x.artificial);
  const double expected_physical[9] = {jl, 0, 0, 0, jl + jr, 0, 0, 0, jr};
  for (unsigned i = 0; i < 9; ++i) {
    EXPECT_NEAR(physical.v[i], expected_physical[i], kRelative * (jl + jr));
    EXPECT_NEAR(physical.v[i] + artificial.v[i], i % 4 == 0 ? jl + jr : 0, kRelative * (jl + jr));
  }
  const shell::Vec3 omega{3, 4, 5};
  const double physical_energy = .5 * shell::detail::Dot(omega, shell::detail::Product(physical, omega));
  Near(physical_energy, .5 * (jl * (9 + 16) + jr * (16 + 25)));
  // Scalar summation about a single z director would lose the right tile's
  // physical tensor, even though numerical regularization makes total J*I.
  EXPECT_GT(std::abs(physical_energy - .5 * (jl + jr) * (9 + 16)), jr);
}

TEST(ReissnerShellMass, MalformedSetupAndUnknownPolicyPreserveOutput) {
  const auto fixture = Rectangle();
  const auto sentinel = Mass(Rectangle(.5, .3));
  auto output = sentinel;
  auto reject = [&](Fixture f, Policy policy, Status expected) {
    EXPECT_EQ(shell::ComputeShellMass(f.reference, f.section, policy, output), expected);
    Unchanged(output, sentinel);
  };
  auto bad = fixture;
  bad.reference.prepared = false; reject(bad, kCoupon, Status::kInvalidReference);
  bad = fixture; bad.section.prepared = false; reject(bad, kCoupon, Status::kInvalidSection);
  for (double value : {0., -1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    bad = fixture; bad.section.thickness = value; reject(bad, kCoupon, Status::kInvalidSection);
    bad = fixture; bad.section.density = value; reject(bad, kCoupon, Status::kInvalidSection);
    bad = fixture; bad.reference.gauss[3].area_weight = value; reject(bad, kCoupon, Status::kInvalidReference);
  }
  bad = fixture; bad.reference.gauss[3].shape[0] = -.1; reject(bad, kCoupon, Status::kInvalidReference);
  bad = fixture; bad.reference.gauss[3].shape[0] = std::numeric_limits<double>::quiet_NaN(); reject(bad, kCoupon, Status::kInvalidReference);
  bad = fixture; bad.reference.gauss[3].shape[0] += .1; reject(bad, kCoupon, Status::kInvalidReference);
  reject(fixture, static_cast<Policy>(99), Status::kUnsupportedPolicy);
  ASSERT_EQ(shell::ComputeShellMass(fixture.reference, fixture.section, kCoupon, output), Status::kSuccess);
}

TEST(ReissnerShellMass, UnrepresentablePositiveMassOrInertiaPreservesOutput) {
  const auto sentinel = Mass(Rectangle());
  auto output = sentinel;
  for (const auto fixture : {Rectangle(20, 20, 1, std::numeric_limits<double>::max()),
                             Rectangle(.4, .2, 1e-300, 1e-300),
                             Rectangle(.4, .2, 1e-250, 1e300)}) {
    EXPECT_EQ(shell::ComputeShellMass(fixture.reference, fixture.section, kCoupon, output), Status::kNonfiniteResult);
    Unchanged(output, sentinel);
  }
}

TEST(ReissnerShellMass, LateKinematicOrEnergyFailurePreservesOutputAndRetry) {
  auto mass = Mass(Rectangle());
  shell::Vec3 d[4], v[4], w[4];
  for (unsigned n = 0; n < 4; ++n) { d[n] = {0, 0, 1}; v[n] = {1, 2, 3}; w[n] = {4, 5, 6}; }
  const shell::ShellKineticEnergy sentinel{17, 19, 23};
  auto output = sentinel;
  d[3] = {0, 0, 2};
  EXPECT_EQ(shell::ComputeShellKineticEnergy(mass, d, v, w, output), Status::kInvalidKinematics);
  Unchanged(output, sentinel);
  d[3] = {0, 0, 1};
  v[3].x = std::numeric_limits<double>::max();
  EXPECT_EQ(shell::ComputeShellKineticEnergy(mass, d, v, w, output), Status::kNonfiniteResult);
  Unchanged(output, sentinel);
  v[3] = {1, 2, 3};
  mass.node[3].artificial_drilling_inertia = 0;
  EXPECT_EQ(shell::ComputeShellKineticEnergy(mass, d, v, w, output), Status::kInvalidMass);
  Unchanged(output, sentinel);
  mass = Mass(Rectangle());
  ASSERT_EQ(shell::ComputeShellKineticEnergy(mass, d, v, w, output), Status::kSuccess);
  EXPECT_GT(output.translation, 0);
  EXPECT_GT(output.physical_rotation, 0);
  EXPECT_GT(output.artificial_drilling, 0);
}

TEST(ReissnerShellMass, InvalidInertiaInputsPreserveBothTensorParts) {
  const auto mass = Mass(Rectangle());
  shell::ShellInertiaTensor output;
  for (unsigned i = 0; i < 9; ++i) { output.physical.v[i] = 10 + i; output.artificial.v[i] = 20 + i; }
  const auto before = output;
  for (const auto director : {shell::Vec3{0, 0, 0}, shell::Vec3{0, 0, 2},
                              shell::Vec3{std::numeric_limits<double>::quiet_NaN(), 0, 1}}) {
    EXPECT_EQ(shell::ComputeShellInertiaTensor(mass.node[0], director, output), Status::kInvalidKinematics);
    EXPECT_EQ(std::memcmp(output.physical.v, before.physical.v, sizeof(output.physical.v)), 0);
    EXPECT_EQ(std::memcmp(output.artificial.v, before.artificial.v, sizeof(output.artificial.v)), 0);
  }
  EXPECT_EQ(shell::ComputeShellInertiaTensor(shell::ShellNodalMass{}, {0, 0, 1}, output), Status::kInvalidMass);
  EXPECT_EQ(std::memcmp(output.physical.v, before.physical.v, sizeof(output.physical.v)), 0);
  EXPECT_EQ(std::memcmp(output.artificial.v, before.artificial.v, sizeof(output.artificial.v)), 0);
}
}  // namespace
