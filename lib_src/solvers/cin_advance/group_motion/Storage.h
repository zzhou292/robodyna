// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Groups.h"

namespace tl::fea::cin_advance::group_motion {
inline constexpr unsigned Threads = 64;
// Shared storage is plain scalar data. Construct the existing math values only
// in thread-private scope; no placement casts or shared nontrivial objects.
struct Vector { double x, y, z; };
struct Frame { double axes[9]; Vector inertia; };
struct InputValues {
  Frame previous_frame;
  Vector center, velocity, omega;
  double mass;
  Vector force, couple;
  double previous_drift_dt, kick_dt, drift_dt;
};
struct PrimaryValues {
  Frame force_frame;
  Vector saved_body_omega, acceleration, angular_acceleration;
  Vector center, velocity, omega;
};
struct State {
  InputValues input;
  PrimaryValues primary;
  groups::Report report;
  std::uint32_t offset, count, prefix;
  bool dependent_coefficients;
};
struct WrenchSlot {
  Vector force, couple;
  NodalStatus status;
  std::uint32_t node;
};
struct Tile {
  State state;
  WrenchSlot wrench[Threads];
  NodalStatus status[Threads];
};
static_assert(std::is_trivial<Tile>::value);
static_assert(sizeof(InputValues) == 248);
static_assert(sizeof(PrimaryValues) == 240);
static_assert(sizeof(State) == 528);
static_assert(sizeof(Tile) == 4368);
TL_SURFACE_HD inline Vector Store(rigid::Vec3 v) { return {v.x,v.y,v.z}; }
TL_SURFACE_HD inline rigid::Vec3 Read(Vector v) { return {v.x,v.y,v.z}; }
TL_SURFACE_HD inline Frame Store(const rigid::PrincipalFrame& v) {
  Frame out{};
  for (unsigned i=0;i<9;++i) out.axes[i]=v.axes.v[i];
  out.inertia=Store(v.inertia);return out;
}
TL_SURFACE_HD inline rigid::PrincipalFrame Read(const Frame& v) {
  rigid::PrincipalFrame out;
  for (unsigned i=0;i<9;++i) out.axes.v[i]=v.axes[i];
  out.inertia=Read(v.inertia);return out;
}
TL_SURFACE_HD inline InputValues Store(const rigid::PrimaryStepInput& v) {
  return {Store(v.previous_frame),Store(v.center),Store(v.velocity),Store(v.omega),v.mass,
    Store(v.applied.force),Store(v.applied.couple),v.durations.previous_drift_dt,
    v.durations.kick_dt,v.durations.drift_dt};
}
TL_SURFACE_HD inline rigid::PrimaryStepInput Read(const InputValues& v) {
  return {Read(v.previous_frame),Read(v.center),Read(v.velocity),Read(v.omega),v.mass,
    {Read(v.force),Read(v.couple)},{v.previous_drift_dt,v.kick_dt,v.drift_dt}};
}
TL_SURFACE_HD inline PrimaryValues Store(const rigid::PrimaryStepTrial& v) {
  return {Store(v.force_frame),Store(v.saved_body_omega),Store(v.acceleration),
    Store(v.angular_acceleration),Store(v.center),Store(v.velocity),Store(v.omega)};
}
TL_SURFACE_HD inline rigid::PrimaryStepTrial Read(const PrimaryValues& v) {
  return {Read(v.force_frame),Read(v.saved_body_omega),Read(v.acceleration),
    Read(v.angular_acceleration),Read(v.center),Read(v.velocity),Read(v.omega)};
}
TL_SURFACE_HD inline NodalStatus Status(rigid::StepStatus value) {
  return value==rigid::StepStatus::Success ? NodalStatus::Ok :
      value==rigid::StepStatus::RotationLimit ? NodalStatus::StepTooLarge : NodalStatus::InvalidOutput;
}
TL_SURFACE_HD inline bool Failed(const State& state) { return state.report.status!=NodalStatus::Ok; }
TL_SURFACE_HD inline void Fail(State& state,NodalStatus status,std::uint32_t node) {
  state.report.status=status;state.report.last_node=node;
}
} // namespace tl::fea::cin_advance::group_motion
