// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
namespace rear_force_test {
struct NativeState {
  std::array<double,160> point{};
  std::array<double,12> global{};
  std::array<double,21> saved{};
  std::array<int,8> cursor{};
  std::array<int,8> source_slot{}, local_nodes{};
  double initial_center_volume = 0;
};
struct NativeResult {
  NativeState next;
  std::array<double,304> observation{};
  std::array<double,24> force{};
  std::array<double,1111> geometry{};
  std::array<double,8> diagnostics{};
  int status = -1;
};
NativeState NativeInitial(const s::ReferenceInput&);
NativeResult Native(const law::Material&, const NativeState&, const law::PrescribedInterval&);
bool Agree(const law::ForceTrial&, const NativeResult&);
NativeResult Values(const law::ForceTrial&);
}
