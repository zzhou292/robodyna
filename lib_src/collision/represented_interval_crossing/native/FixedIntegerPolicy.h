// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ArithmeticContext.h"
#include "lib_src/math/FixedInteger.h"

namespace tlfea::contact::represented_interval_crossing::native {
// Thin adapter to the existing shared fixed integer core. All temporary errors
// latch before a boolean/sign can erase them. No retry, exception emulation,
// global state, source-key encoding or caller eligibility flag is involved.
template <unsigned Bits>
class FixedIntegerPolicy {
 public:
  static constexpr bool TracksErrors = true;
  static_assert(Bits % 64 == 0 && Bits >= 64);
  using Core = tl::math::fixed_integer::Arithmetic<Bits / 64>;
  using Integer = typename Core::Integer;
  TL_MATH_HOST_DEVICE explicit FixedIntegerPolicy(ArithmeticContext& context) noexcept : context_(context) {}
  TL_MATH_HOST_DEVICE void Assign(Integer& output, std::uint64_t value) noexcept {
    output = Checked(Core::FromU64(value));
  }
  TL_MATH_HOST_DEVICE bool IsZero(const Integer& value) noexcept { return Sign(value) == 0; }
  TL_MATH_HOST_DEVICE bool IsNegative(const Integer& value) noexcept { return Sign(value) < 0; }
  TL_MATH_HOST_DEVICE Integer Add(const Integer& a, const Integer& b) noexcept { return Checked(Core::Add(a, b)); }
  TL_MATH_HOST_DEVICE Integer Negate(const Integer& value) noexcept { return Checked(Core::Negate(value)); }
  TL_MATH_HOST_DEVICE Integer Multiply(const Integer& a, const Integer& b) noexcept { return Checked(Core::Multiply(a, b)); }
  TL_MATH_HOST_DEVICE void Shift(Integer& value, unsigned amount) noexcept { value = Checked(Core::ShiftLeft(value, amount)); }
  TL_MATH_HOST_DEVICE void Scale(Integer& value, std::uint64_t factor) noexcept {
    value = Checked(Core::Multiply(value, Core::FromU64(factor)));
  }
  TL_MATH_HOST_DEVICE int Sign(const Integer& value) noexcept {
    const auto result = Core::Result(value);
    context_.Observe(result.valid);
    return result.value;
  }
  template <class Body, class Failure>
  TL_MATH_HOST_DEVICE auto Protect(Body&& body, Failure&& failure) noexcept -> decltype(body()) {
    const auto result = body();
    return context_.valid() ? result : failure();
  }
 private:
  TL_MATH_HOST_DEVICE Integer Checked(Integer value) noexcept {
    context_.Observe(!value.overflow);
    return value;
  }
  ArithmeticContext& context_;
};
}  // namespace tlfea::contact::represented_interval_crossing::native
