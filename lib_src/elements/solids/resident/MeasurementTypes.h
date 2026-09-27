// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <type_traits>

namespace tl::fea::solids::batch_detail {
// Fresh operands, never a cache, history, or per-parent floating-point sum.
template<unsigned Slots> struct MeasurementOperands {
  double work = 0;
  double hourglass_work = 0;
  double distortion_work = 0;
  double plastic_work = 0;
  double native_dt = 0;
  double kick[Slots]{};
  double drift[Slots]{};
};
static_assert(sizeof(MeasurementOperands<8>) == 168);
static_assert(sizeof(MeasurementOperands<6>) == 136);
static_assert(std::is_trivially_copyable_v<MeasurementOperands<8>>);
static_assert(std::is_trivially_copyable_v<MeasurementOperands<6>>);
} // namespace tl::fea::solids::batch_detail
