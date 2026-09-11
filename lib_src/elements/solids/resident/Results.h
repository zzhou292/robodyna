// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <type_traits>
#include "../ForceStiffness.h"
#include "../../solid18/law44/ForceTypes.h"
#include "../../solid18/total_strain/ForceTypes.h"

namespace tl::fea::solids {
// Retained observations omit temporary current derivatives and reference
// geometry. All actual point histories and native physical HG history remain.
struct Cache18 {
  solid18::Vec3 rhs_force_n[8]{};
  solid18::ForceDiagnostics diagnostics;
  NodalStiffness stiffness;
};
struct Cache24 {
  solid24::Vec3 rhs_force_n[8]{};
  solid24::ForceDiagnostics diagnostics;
  NodalStiffness stiffness;
};
struct Cache6z {
  solid6z::Vec3 rhs_force_n[6]{};
  tl::material::law42::CallerResult material;
  solid6z::HourglassObservation stabilization;
  double total_internal_work_increment_j = 0;
  NodalStiffness stiffness;
};
struct Result18 {
  solid18::HistoryValues history;
  solid18::HistoryStamp stamp;
  Cache18 cache;
};
struct Result24 {
  solid24::HistoryValues history;
  solid24::HistoryStamp stamp;
  Cache24 cache;
};
struct Result6z {
  solid6z::HistoryValues history;
  solid6z::HistoryStamp stamp;
  Cache6z cache;
};
struct Cache18Law44 {
  solid18::Vec3 rhs_force_n[8]{};
  solid18::law44::ForceDiagnostics diagnostics;
  NodalStiffness stiffness;
};
struct Result18Law44 {
  solid18::law44::HistoryValues history;
  solid18::law44::HistoryStamp stamp;
  Cache18Law44 cache;
};
static_assert(std::is_trivially_copyable_v<Result18Law44>);
struct Cache18Law90 {
  solid18::Vec3 rhs_force_n[8]{};
  solid18::total_strain::ForceDiagnostics diagnostics;
  NodalStiffness stiffness;
};
struct Result18Law90 {
  solid18::total_strain::HistoryValues history;
  solid18::HistoryStamp stamp;
  Cache18Law90 cache;
};
static_assert(std::is_trivially_copyable_v<Result18Law90>);
static_assert(std::is_trivially_copyable_v<Result18>);
static_assert(std::is_trivially_copyable_v<Result24>);
static_assert(std::is_trivially_copyable_v<Result6z>);
// Exact complete family counts are required. An absent family uses {nullptr,0}.
struct ResultBuffers {
  Result18* solid18 = nullptr;
  std::size_t count18 = 0;
  Result24* solid24 = nullptr;
  std::size_t count24 = 0;
  Result6z* solid6z = nullptr;
  std::size_t count6z = 0;
  Result18Law44* solid18_law44 = nullptr;
  std::size_t count18_law44 = 0;
  Result18Law90* solid18_law90 = nullptr;
  std::size_t count18_law90 = 0;
};
} // namespace tl::fea::solids
