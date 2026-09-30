// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type13Types.h"

namespace tl::fea::type13 {
// These values remain in the Property's explicit working units. Positions are
// at the interval endpoint; velocities and spin are its actual midpoint values.
struct NativeEndpointKinematics {
  Vec3 position{};
  Vec3 velocity{};
  Vec3 angular_velocity{};
};

struct NativeChannelHistory {
  // Channels 0..2: native length; 3..5: radians. REDEF3 temporarily divides
  // BOTH by original native length, then restores dimensional history.
  double deformation = 0;
  double accumulated_plastic_deformation = 0;
  double elastic_plastic_force = 0; // FXEP: native force/moment, unmasked.
  double force = 0; // FX: actual cached force/moment, masked by previous OFF.
  double signed_work = 0; // Native force*length; signed, not dissipation.
  unsigned curve_position = 0; // VINTER2 interval, including knot-side history.
};

struct NativeHistory {
  Vec3 transverse_axis{};
  NativeChannelHistory channels[ChannelCount]{};
  double failure_criterion = 0; // Native CRIT_NEW, capped at one.
  bool active = true;
};

struct NativeFrame {
  Matrix3 axes{};
  Matrix3 midpoint_axes{};
  double length = 0;
  double midpoint_length = 0;
};

struct EndpointWrenchSI {
  Vec3 force_N{};
  Vec3 couple_Nm{};
};

struct Stability {
  double critical_dt_s = 0;
  double translation_stiffness_N_per_m = 0;
  double rotation_stiffness_Nm_per_rad = 0;
};

struct Evaluation {
  NativeHistory native_history{};
  NativeFrame native_frame{};
  EndpointWrenchSI endpoints[2]{};
  Vec3 local_force_N{};
  Vec3 local_couple_Nm{};
  double signed_work_J[ChannelCount]{};
  double total_signed_work_J = 0;
  Stability stability{};
  bool newly_failed = false; // Caller associates this with its own interval.
};
} // namespace tl::fea::type13
