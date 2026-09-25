// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Values.h"
namespace tlfea::contact::radioss_type25::search::detail {
inline constexpr unsigned Threads = 128, MaximumBlocks = 256;
struct Partial { Extrema extrema; Status status = Status::Ok; std::size_t invalid = SIZE_MAX; };
struct DeviceControl { Partial partial; Budget budget; };
struct Device {
  std::uint32_t* roles = nullptr;
  Vector* reference[2]{};
  std::uint8_t* masks[2]{};
  double* gaps[2]{};
  Partial* partials = nullptr;
  DeviceControl* control = nullptr;
  std::size_t nodes = 0, secondary = 0, main = 0, one_d = 0, segments = 0;
  std::size_t reference_count = 0;
  bool compact = false, gap_changes = false, si = false;
  double length = 1, velocity = 1;
};
TL_MATH_HOST_DEVICE inline std::size_t RoleCount(const Device& d) noexcept {
  return d.secondary+d.main+d.one_d;
}
TL_MATH_HOST_DEVICE inline Vector Position(const Device& d, VectorView view, std::uint32_t node) noexcept {
  const auto v=view.at(node);
  return d.si ? Vector{v.x/d.length,v.y/d.length,v.z/d.length} : Vector{v.x,v.y,v.z};
}
TL_MATH_HOST_DEVICE inline Vector Velocity(const Device& d, VectorView view, std::uint32_t node) noexcept {
  const auto v=view.at(node);
  return d.si ? Vector{v.x/d.velocity,v.y/d.velocity,v.z/d.velocity} : Vector{v.x,v.y,v.z};
}
TL_MATH_HOST_DEVICE inline void Fail(Partial& p, Status status, std::size_t row) noexcept {
  if (row<p.invalid) { p.status=status;p.invalid=row; }
}
TL_MATH_HOST_DEVICE inline void Merge(Partial& a,const Partial& b) noexcept {
  Merge(a.extrema,b.extrema);if (b.invalid<a.invalid) {a.status=b.status;a.invalid=b.invalid;}
}
// Same source masks as I25BUCE_CRIT. Reference capture validates only consumed
// active positions; inactive data can remain unused, as in the native routine.
TL_MATH_HOST_DEVICE inline void ObserveRole(const Device& d, const Current& input,
    unsigned slab, std::size_t role, bool capture, bool has_reference, Partial& output) noexcept {
  const auto node=d.roles[role];
  const bool secondary=role<d.secondary;
  const bool one_d=role>=d.secondary+d.main;
  if (secondary) {
    const double stiffness=input.secondary_stiffness[role];
    if (!tl::math::Finite(stiffness)) {Fail(output,Status::NonfiniteResult,role);return;}
    // Native observes the nonzero row before its NSPMD=1 negative clamp.
    // Supporting that transition requires a staged caller phase, not pre-clamping.
    if (stiffness < 0) {Fail(output,Status::UnsupportedLifecycle,role);return;}
    const bool active=stiffness!=0;
    if ((!capture || has_reference) && bool(d.masks[capture ? 1-slab : slab][role])!=active) {
      Fail(output,Status::UnsupportedLifecycle,role);return;
    }
    if (capture) d.masks[slab][role]=active;
    if (!active) return;
  } else if (node==UINT32_MAX) return;
  const auto current=Position(d,input.positions,node);
  const auto reference=capture ? current : d.reference[slab][d.compact ? role : node];
  const auto velocity=capture ? Vector{} : Velocity(d,input.velocities,node);
  const auto delta=tl::math::fixed3::Subtract(current,reference);
  if (!tl::math::fixed3::Finite(current) || !tl::math::fixed3::Finite(reference) ||
      !tl::math::fixed3::Finite(delta) || !tl::math::fixed3::Finite(velocity)) {
    Fail(output,Status::NonfiniteResult,role);return;
  }
  if (secondary || one_d) {
    Observe(output.extrema.secondary_displacement,delta);Observe(output.extrema.secondary_velocity,velocity);
    ++output.extrema.secondary_uses;
  }
  if (!secondary) {
    Observe(output.extrema.main_displacement,delta);Observe(output.extrema.main_velocity,velocity);
    ++output.extrema.main_uses;
  }
}
TL_MATH_HOST_DEVICE inline void ObserveGap(const Device& d,const Current& input,
    unsigned slab,std::size_t row,bool capture,Partial& output) noexcept {
  const double current=d.si ? input.main_gaps[row]/d.length : input.main_gaps[row];
  const double reference=capture ? current : d.gaps[slab][row];
  const double delta=current-reference;
  if (!normal_detail::Nonnegative(current) || !normal_detail::Nonnegative(reference) ||
      !tl::math::Finite(delta)) {Fail(output,Status::NonfiniteResult,RoleCount(d)+row);return;}
  if (capture) d.gaps[slab][row]=current;
  output.extrema.maximum_gap_change=Maximum(output.extrema.maximum_gap_change,delta);
}
} // namespace tlfea::contact::radioss_type25::search::detail
