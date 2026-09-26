// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../selection/Types.h"
namespace tlfea::contact::radioss_type25::initial_state {
enum class Status { Ok, InvalidInput, UnsupportedProfile, UndefinedNativeInput, NonfiniteResult, CapacityExceeded };
struct Profile {
  int gap_mode=1, initial_penetration=5, damping_flag=1, sharp=1, arithmetic_precision=0;
  int partitions=1;
};
// Native working units and genuine Starter normals/gaps. This value API is not
// source/candidate completeness authority and does not admit runtime histories.
// NativePairInput's current response coefficients/ICONT are not consumed here.
struct PairInput {
  Profile profile;
  selection::NativePairInput geometry;
  int expanded_main_count=0;
};
struct PairResult {
  GeometryRowKey key;
  int local_main=0, sector=0, far=0;
  double distance_squared=0, penetration_offset=0;
  bool considered=false;
};
struct Winner {
  NativeContactRow row;
  double distance_squared=1.e20; // Ephemeral Starter TIME_S nearest-distance workspace.
};
} // namespace tlfea::contact::radioss_type25::initial_state
