#pragma once

#include "lib_src/elements/ReissnerFrame.h"

#include <array>
#include <cstdint>

namespace tl::qualification::qeph {

// Reuse TL's binary64 vector/matrix records, not Reissner mechanics or rest data.
using Vec3 = tl::fea::reissner::Vec3;
using Matrix3 = tl::fea::reissner::Matrix3;

enum class Status {
  kSuccess,
  kInvalidInput,
  kInvalidReference,
  kNativeFailure,
  kNonfiniteResult,
};

struct ReferenceInput {
  std::array<Vec3, 4> position{};  // Original world vertices, cyclic native order.
  std::array<std::uint32_t, 4> node_ids{{0, 1, 2, 3}};
  double density = 7890;          // kg/m^3
  double young_modulus = 200e9;   // Pa
  double poisson_ratio = 0.3;     // Fixed initial material domain: 0 <= nu < .5.
  double thickness = 0.001648;    // m
};

struct ReferenceData {
  ReferenceInput input;
  Matrix3 frame;                         // Row-major; columns are world axes.
  double area = 0;                       // Native startup area, m^2.
  std::array<double, 4> derivative_x{};   // CDERII unnormalized coefficients, m.
  std::array<double, 4> derivative_y{};
  std::array<Vec3, 4> local_position{};   // CDERII coordinates relative to node0.
  std::array<double, 4> nodal_mass{};     // rho*t*A/4, kg.
  std::array<double, 4> physical_inertia{};  // m*t^2/12, kg*m^2 per axis.
  std::array<double, 4> added_inertia{};     // m*A/12, separately accounted.
  std::array<double, 4> isotropic_inertia{}; // Source m*(A/12+t*t*(1/12+0)).
};

struct PrescribedInterval {
  std::array<Vec3, 4> position_endpoint{};
  std::array<Vec3, 4> velocity_midpoint{};
  std::array<Vec3, 4> omega_midpoint{};  // World rad/s; no hidden q state.
  double base_time = 0;
  double dt = 0;
  std::uint64_t sample_index = 0;       // Caller-owned prescribed sequence.
};

struct Kinematics {
  Matrix3 frame;                       // Native current frame, same convention.
  double area = 0;
  double reciprocal_area = 0;
  double characteristic_length = 0;    // CZCORC1 LL before material/damping.
  std::array<double, 2> nodal_factors{};// Source FACN, applied later by CUPDTN3.
  double raw_warpage_abs = 0;          // abs(raw source Z1), before planar switch.
  double effective_warpage = 0;        // Source Z1 after native projection.
  bool planar = false;
  std::array<Vec3, 4> local_position{}; // Centered source COREL, alternating Z1.
  std::array<Vec3, 4> local_normals{};  // Native VQN; flat branch reports +Z.
  std::array<double, 6> projection_inverse{}; // DI; zero for flat branch.
  std::array<Vec3, 4> projection_columns{};   // DB; zero for flat branch.
  std::array<double, 8> projected_omega{};    // Two local components per node.
  // Native VDEF: XX YY XY XZ YZ KXX KYY KXY (not GSTR's YZ/XZ order).
  // First5 units 1/s; last3 units 1/(m*s).
  std::array<double, 8> regular_rate{};
  // Native VHG: first2 and last2 m/s; components2/3 are 1/s.
  std::array<double, 6> hourglass_rate{};
  double base_time = 0;
  double dt = 0;
  std::uint64_t sample_index = 0;
};

class Reference {
 public:
  bool prepared() const noexcept { return prepared_; }
  const ReferenceData& data() const noexcept { return data_; }

 private:
  ReferenceData data_{};
  bool prepared_ = false;
  friend Status Initialize(const ReferenceInput&, Reference&) noexcept;
};

}  // namespace tl::qualification::qeph
