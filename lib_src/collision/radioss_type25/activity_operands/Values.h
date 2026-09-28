// SPDX-License-Identifier: AGPL-3.0-or-later
// Native CHECK_SURFACE_STATE/CHECK_NODAL_STATE branches, OpenRadioss a62b27e6.
#pragma once
#include "Types.h"
#include <climits>
#include <cstring>
namespace tlfea::contact::radioss_type25::activity_operands::detail {
TL_MATH_HOST_DEVICE inline bool Same(double a, double b) {
#if defined(__CUDA_ARCH__)
  return __double_as_longlong(a) == __double_as_longlong(b);
#else
  return std::memcmp(&a, &b, sizeof(a)) == 0;
#endif
}
TL_MATH_HOST_DEVICE inline bool Scale(double native, double factor, double& si) {
  const auto value = native * factor;
  if (!tl::math::Finite(native) || !tl::math::Finite(value) || (native != 0 && value == 0)) return false;
  si = value; return true;
}
struct MainResult {
  double coefficient = 0;
  std::int32_t connected = 0;
  bool removed = false, exposure = false, valid = false;
};
TL_MATH_HOST_DEVICE inline MainResult Main(double coefficient, std::int32_t connected,
    std::uint32_t events, bool supported, activity_source::Controls controls) {
  MainResult out{coefficient, connected, false, false, false};
  if (!tl::math::Finite(coefficient)) return out;
  if (!events || controls.deletion == activity_source::Deletion::Disabled) { out.valid = true; return out; }
  const bool erosion = controls.solid_erosion == startup::SolidErosion::Enabled;
  if (erosion) {
    const auto next = std::int64_t(connected) - events;
    if (next < INT32_MIN) return out;
    out.connected = static_cast<std::int32_t>(next);
  }
  out.removed = !supported;
  if (out.removed) out.coefficient = 0;
  // Native evaluates this after EACH event. A 2->1->0 chain exposes at its
  // intermediate count when current support survives; checking only the final
  // count would miss that unsupported topology transition.
  else if (erosion && coefficient < 0 && connected > 1 &&
           std::int64_t(connected) - events <= 1) out.exposure = true;
  out.valid = true; return out;
}
// Native CHKMSR3NB tests global ITAG independently of IDELKEEP. The source
// main geometry defines MSR membership; nonmembers are neutral and cannot
// cause a source-generation change when unrelated elements are removed.
TL_MATH_HOST_DEVICE inline std::uint8_t MainNodeActivity(bool member, bool supported,
    activity_source::Controls controls) {
  return controls.deletion == activity_source::Deletion::Disabled || !member || supported ? 1 : 0;
}
TL_MATH_HOST_DEVICE inline double MarkSecondary(double coefficient, bool supported,
    activity_source::Controls controls) {
  const bool registered = controls.deletion != activity_source::Deletion::Disabled &&
      !controls.keep_disconnected_nodes;
  return registered && !supported && coefficient > 0 ? -coefficient : coefficient;
}
TL_MATH_HOST_DEVICE inline double NormalizeSecondary(double marked) { return marked < 0 ? 0 : marked; }
} // namespace tlfea::contact::radioss_type25::activity_operands::detail
