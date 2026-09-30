#pragma once

#include "ShellFixture.h"

namespace shell_spike {

// S2a only: one fresh elastic material-integration increment, not the full S2
// shell gate. No plasticity, hourglass, warped shells, mass, contact or time loop.
enum class ThicknessRule { OriginalMidpoint3, Gauss3 };

struct ElasticInput {
  double half_x = 1;
  double half_y = .75;
  double thickness = .1;
  double dt = .1;
  Vec3 center{};
  std::array<Vec3,3> basis{{{1,0,0},{0,1,0},{0,0,1}}};
  // World-coordinate nodal velocities at corners (-a,-b),(a,-b),(a,b),(-a,b).
  // An instantaneous operator probe: no claim that arbitrary same-instant
  // velocities reproduce the production solver's staggered time trajectory.
  std::array<Vec3,4> velocity{}, angular_velocity{};
};

struct ElasticPoint {
  // sigma_xx, sigma_yy, tau_xy, tau_yz, tau_zx in the current shell frame.
  std::array<double,5> stress{};
  double plastic_strain = 0;
  double plastic_increment = 0;
  double filtered_plastic_rate = 0;
  std::array<double,3> backstress{};
  double temperature = 0;
};

struct ElasticResult {
  ThicknessRule rule = ThicknessRule::OriginalMidpoint3;
  std::array<Vec3,3> frame{};
  double area = 0;
  // xx, yy, engineering xy, yz, zx, kxx, kyy, engineering kxy.
  std::array<double,8> generalized_increment{};
  std::array<ElasticPoint,3> points{};
  // Physical through-thickness resultants: xx, yy, xy, Qy, Qx; Bxx, Byy, Bxy.
  std::array<double,5> forces{};
  std::array<double,3> moments{};
  std::array<double,5> normalized_forces{};
  std::array<double,3> normalized_moments{};
  Tensor membrane_world{}, bending_world{};
  Vec3 shear_world{};
  double thickness = 0;
  double reference_thickness = 0;
  double off = 0;
  double mean_yield = 0;
  double element_rate = 0;
  std::array<double,2> energy{};
  std::size_t device_bytes = 0;
};

// Fixed synthetic material, elastic predictor only; these are not imported
// MAT024 constants. A conservative prelaunch bound and zero-plasticity output
// checks keep accepted fixture inputs inside this declared scope.
constexpr double kElasticYoung = 210e9;
constexpr double kElasticNu = .3;
constexpr double kElasticYield = 1e12;
constexpr double kElasticTemperature = 293.15;
constexpr double kElasticShearFactor = 5.0/6.0;

// Always starts from zero strain/stress/history with the selected rule's own
// material points. Midpoint histories cannot be resumed as Gauss histories.
// `committed` is only a complete one-shot result, replaced on Ok; it is never
// input to a subsequent constitutive step. Input and old result survive failure.
Attempt EvaluateElastic(const ElasticInput&, ThicknessRule, ElasticResult*,
                        const Options& = {});

namespace detail {
Attempt EvaluateElasticOriginal(const ElasticInput&,ElasticResult*,const Options&);
Attempt EvaluateElasticGauss(const ElasticInput&,ElasticResult*,const Options&);
}  // namespace detail

}  // namespace shell_spike
