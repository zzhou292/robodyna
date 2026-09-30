// Generated from authenticated source cards by verify_curves.py.
// Original stresses are MPa; multiplication by 1e6 converts to Pa.
#pragma once
#include <array>
#include <cstdint>
namespace continuation_test {
struct OriginalCurve { std::uint64_t id; const double* x; const double* y; std::uint32_t count; };
inline constexpr double x2100180[]{0.0, 0.025, 0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2, 0.225, 0.25, 0.275, 0.3, 0.325, 0.35, 0.4, 0.5};
inline constexpr double y2100180[]{180000000.0, 219000000.0, 247000000.0, 271000000.0, 290000000.0, 307000000.0, 321000000.0, 334000000.0, 345000000.0, 355000000.0, 365000000.0, 374000000.0, 381000000.0, 388000000.0, 394000000.0, 401000000.0, 410000000.0};
inline constexpr double x2100220[]{0.0, 0.025, 0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2, 0.225, 0.25, 0.275, 0.3, 0.35, 0.4};
inline constexpr double y2100220[]{220000000.0, 250000000.0, 270000000.0, 288000000.0, 304000000.0, 317000000.0, 330000000.0, 340000000.0, 350000000.0, 359000000.0, 367000000.0, 374000000.0, 380000000.0, 390000000.0, 398000000.0};
inline constexpr double x2100271[]{0.0, 0.025, 0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2, 0.25, 0.3};
inline constexpr double y2100271[]{271000000.0, 302000000.0, 322000000.0, 336000000.0, 345000000.0, 354000000.0, 362000000.0, 370000000.0, 377000000.0, 385000000.0, 390000000.0};
inline constexpr std::array<OriginalCurve,3> Curves{{
  {2100180, x2100180, y2100180, 17},
  {2100220, x2100220, y2100220, 15},
  {2100271, x2100271, y2100271, 11},
}};
} // namespace continuation_test
