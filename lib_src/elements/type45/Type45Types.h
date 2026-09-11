// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/math/Fixed3Operations.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_TYPE45_HD __host__ __device__
#else
#define TL_TYPE45_HD
#endif

namespace tl::fea::type45 {
using Vec3 = tl::math::Vec3;
using Matrix3 = tl::math::Matrix3;
enum class Kind : std::uint8_t { Spherical=1, Revolute=2, Cylindrical=3 };
enum class WorkingUnits : std::uint8_t { SI, MillimetreTonneSecond };
enum class EndpointRole : std::uint8_t { Structural, RigidMember, RigidMain };
enum class Status : std::uint8_t {
  Success, InvalidProperty, InvalidGeometry, InvalidContext, UnsupportedRole,
  InvalidHistory, StaleInterval, ReferenceMismatch, NonfiniteResult
};

struct DofValues {
  Vec3 translation{}, rotation{};
};
// Explicit supplied native scalar property. Source-card blank/default/export
// resolution belongs to the caller; these values do not authenticate a deck.
struct Property {
  Kind kind = Kind::Spherical;
  WorkingUnits working_units = WorkingUnits::SI;
  double automatic_stiffness_scale = 1;
  double critical_damping_ratio = .05;
  DofValues free_stiffness; // N/m and N m/rad
  DofValues free_viscosity; // N s/m and N m s/rad
};

struct GeometryInput {
  std::uint64_t source_joint_id = 0;
  std::uint64_t source_node_id[3]{}; // N1/N2; optional axis-only N3
  Vec3 position_m[3]{};
};

// RINI45_RB damping uses body mass and mean principal inertia. It does not
// consume the main-node inertia used by JOINT_BLOCK_STIFFNESS below.
struct DampingEndpoint {
  double mass_kg = 0;
  double mean_principal_inertia_kg_m2 = 0;
};
struct MainNodeContext {
  EndpointRole role = EndpointRole::Structural;
  std::uint64_t source_body_id = 0;
  Vec3 position_m{};
  double mass_kg = 0;
  double inertia_kg_m2 = 0;
  double translational_stiffness_n_m = 0;
  double rotational_stiffness_nm = 0;
};
// Supplied from the common TT0 stiffness snapshot and selected time step.
// This is a value context, not an owner token or a second timestep selector.
struct AutomaticStiffnessContext {
  double target_dt_s = 0;
  MainNodeContext main[2];
};

struct AutomaticStiffness {
  double blocked_translation_n_m = 0;
  double blocked_rotation_nm = 0;
  double unconstrained_translation_limit_n_m = 0;
  double unconstrained_rotation_limit_nm = 0;
  bool raised_to_structural_stiffness = false;
};

// Literal RINI45 observations before automatic stiffness is assigned. These
// are not an assembled TT0 force cache or a completed interval.
struct StartupValues {
  DofValues stiffness;
  double maximum_translation_n_m = 0;
  double maximum_rotation_nm = 0;
  double maximum_viscosity_n_s_m = 0;
  double maximum_rotational_viscosity_nm_s = 0;
};

struct Interval {
  Vec3 position_m[2]{};
  Vec3 angular_velocity_rad_s[2]{};
  double base_time_s = 0;
  double dt_s = 0;
  std::uint64_t sample_index = 0;
};
struct Stamp {
  double time_s = 0;
  std::uint64_t sample_index = 0;
};
struct HistoryValues {
  Matrix3 frame; // Columns are native mean-frame axes in world coordinates.
  Vec3 local_displacement_m{}, relative_rotation_rad{};
  Vec3 local_force_n{}, local_couple_nm{};
  double internal_work_j = 0;
};
struct EndpointResult {
  Vec3 force_n{}, couple_nm{};
  double translational_stiffness_n_m = 0;
  double rotational_stiffness_nm = 0;
};
struct Diagnostics {
  Vec3 local_separation_m{}, local_velocity_m_s{}, relative_rate_rad_s{};
  double harmonic_mass_kg = 0;
  double harmonic_inertia_kg_m2 = 0;
  double maximum_stiffness_n_m = 0;
  double maximum_rotational_stiffness_nm = 0;
  double damping_n_s_m = 0;
  double rotational_damping_nm_s = 0;
  double internal_work_increment_j = 0;
};
} // namespace tl::fea::type45
