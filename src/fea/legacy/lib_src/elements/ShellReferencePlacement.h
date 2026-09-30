// SPDX-License-Identifier: AGPL-3.0-or-later
// Native TYPE1 IPOS/GEO199 placement; source-card resolution belongs to the app.
#pragma once
#if defined(__CUDACC__)
#define TL_PLACEMENT_HD __host__ __device__
#else
#define TL_PLACEMENT_HD
#endif
namespace tl::fea {
enum class ShellReferencePlacement : unsigned char {
  Centered,
  TopReferencePlane,
  BottomReferencePlane
};
TL_PLACEMENT_HD inline bool ValidShellReferencePlacement(ShellReferencePlacement p) noexcept {
  return p==ShellReferencePlacement::Centered ||
      p==ShellReferencePlacement::TopReferencePlane ||
      p==ShellReferencePlacement::BottomReferencePlane;
}
// Callers validate the enum before evaluating this native GEO199 value.
TL_PLACEMENT_HD inline double NativeShellShift(ShellReferencePlacement p) noexcept {
  return p==ShellReferencePlacement::TopReferencePlane ? -.5 :
      p==ShellReferencePlacement::BottomReferencePlane ? .5 : 0.;
}
} // namespace tl::fea
#undef TL_PLACEMENT_HD
