// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../source_shells/Values.h"
namespace tlfea::contact::radioss_type25::source_gaps::detail {
// GNU native MAX/MIN choose the later equal finite operand. Keep this local;
// the legacy coefficient/shell builder's helper has its own frozen contract.
inline double Max(double a, double b) noexcept { return a>b?a:b; }
inline double Min(double a, double b) noexcept { return a<b?a:b; }
inline unsigned Slots(ShellLayout layout) noexcept { return source_shells::detail::Slots(layout); }
inline double HalfShell(const PhysicalShell& shell, int mode) noexcept {
  return source_shells::detail::HalfGap(shell,mode);
}
inline bool Supported(const Profile& p) noexcept {
  return p.property_type==1 && (p.input_thickness_mode==0 || p.input_thickness_mode==1) &&
    p.level==1 && p.gap_mode==1 && p.free_edge_gap==0 && p.contact_thickness_update==0;
}
} // namespace tlfea::contact::radioss_type25::source_gaps::detail
