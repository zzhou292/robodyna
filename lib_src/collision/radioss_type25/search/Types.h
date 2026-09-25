// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Units.h"
#include "../../SurfaceContactTypes.h"
#include "lib_src/math/Fixed3Operations.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::search {
using Vector = tl::math::Vec3;
inline constexpr double ExtentIdentity = native_constant::ep20 * native_constant::ep10;
inline constexpr double MotionFactor = 1. + 1. / 100.; // Native ONEP01.
enum class Status { Ok, InvalidInput, UnsupportedProfile, UnsupportedLifecycle,
  ResourceLimit, NotInitialized, AlreadyInitialized, NoReference, StaleReference,
  NonfiniteResult, DeviceFailure, Unusable };
enum class GapMode { Fixed, CurrentMainGaps };
enum class InputUnits { Native, Si };
enum class VelocityStatus { Normal, Warning, Error };
struct SourceStamp {
  std::uint64_t source = 0, topology = 0, activity = 0;
};
struct QueryStamp { SourceStamp source; std::uint64_t epoch = 0, attempt = 0; };
// Host-only startup descriptors. Maps are copied before Initialize returns.
// UINT32_MAX is an inactive main/main1D entry; secondary entries are local nodes.
struct Source {
  SourceStamp stamp;
  UnitScale units; // Resolved native working units expressed in SI.
  InputUnits input_units = InputUnits::Native;
  std::size_t physical_nodes = 0;
  const std::uint32_t* secondary_nodes = nullptr; std::size_t secondaries = 0;
  const std::uint32_t* main_nodes = nullptr; std::size_t mains = 0;
  const std::uint32_t* main_1d_nodes = nullptr; std::size_t main_1d = 0;
  std::size_t main_segments = 0;
  double margin = 0; // Resolved initialization output in native units; not a tuning factor.
  GapMode gap_mode = GapMode::Fixed;
  int processors = 1, edge_mode = 0, converged = 1;
};
// Borrowed DEVICE arrays on the Initialize stream; every operation drains its
// use before returning, including errors. Native/SI kinematics and gaps follow
// Source.input_units. Secondary stiffness must be finite and nonnegative; its
// zero/nonzero mask supplies activity. Native negative-STFN normalization is a
// future staged caller phase, not permission to pre-clamp this query input.
// Arrays may alias each other when read-only, never outputs or the private arena.
struct Current {
  QueryStamp stamp;
  VectorView positions, velocities;
  const double* secondary_stiffness = nullptr; std::size_t secondary_count = 0;
  const double* main_gaps = nullptr; std::size_t main_gap_count = 0;
};
struct MotionExtrema {
  Vector maximum{-ExtentIdentity, -ExtentIdentity, -ExtentIdentity};
  Vector minimum{ExtentIdentity, ExtentIdentity, ExtentIdentity};
};
struct Extrema {
  MotionExtrema secondary_displacement, main_displacement;
  MotionExtrema secondary_velocity, main_velocity;
  double maximum_gap_change = -ExtentIdentity;
  std::uint64_t secondary_uses = 0, main_uses = 0;
};
struct Budget {
  double displacement = 0, relative_speed = 0;
  double raw_motion = 0, stored_motion = 0;
  double raw_distance = 0, distance = 0;
  VelocityStatus velocity = VelocityStatus::Normal;
  bool requires_sort = true;
};
struct FailureInfo {
  Status status = Status::Ok;
  QueryStamp stamp;
  std::size_t input_row = SIZE_MAX;
  bool query_available = false, row_available = false;
};
struct Report {
  QueryStamp stamp;
  std::uint64_t reference_generation = 0, reference_epoch = 0;
  Extrema extrema;
  Budget budget;
  // Diagnostic cache maintenance only. Never a candidate/physical receipt.
};
struct Limits {
  std::size_t max_nodes = 524288, max_role_entries = 1114112;
  std::size_t max_main_segments = 1048576;
  std::size_t max_device_bytes = 128u << 20, max_host_bytes = 128u << 20;
};
struct Forecast {
  std::size_t device_bytes = 0, startup_host_bytes = 0;
  std::size_t reference_positions = 0, reference_gaps = 0, role_entries = 0;
  bool compact_reference = false;
};
} // namespace tlfea::contact::radioss_type25::search
