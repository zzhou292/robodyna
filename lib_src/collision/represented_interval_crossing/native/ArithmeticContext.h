// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/math/HostDevice.h"

namespace tlfea::contact::represented_interval_crossing::native {
template <unsigned, class> struct CellKernel;
// One lexical pair execution owns this state. Arithmetic may only latch failure;
// BeginPair is called by the pair driver, never by a predicate or a cell reset.
class ArithmeticContext {
 public:
  TL_MATH_HOST_DEVICE ArithmeticContext() noexcept = default;
  ArithmeticContext(const ArithmeticContext&) = delete;
  ArithmeticContext& operator=(const ArithmeticContext&) = delete;
  TL_MATH_HOST_DEVICE void Observe(bool valid) noexcept { failed_ = failed_ || !valid; }
  TL_MATH_HOST_DEVICE bool valid() const noexcept { return !failed_; }
 private:
  template <unsigned, class> friend struct CellKernel;
  TL_MATH_HOST_DEVICE void BeginPair() noexcept { failed_ = false; }
  bool failed_ = false;
};
}  // namespace tlfea::contact::represented_interval_crossing::native
