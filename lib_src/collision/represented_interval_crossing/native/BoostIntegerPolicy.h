// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ArithmeticContext.h"
#include <boost/multiprecision/cpp_int.hpp>
#include <limits>

namespace tlfea::contact::represented_interval_crossing::native {
// Original checked Boost operators and catch boundary. No geometry, admission
// decision, allocation strategy or new arithmetic lives in this policy.
template <unsigned Bits>
class BoostIntegerPolicy {
 public:
  static constexpr bool TracksErrors = false;
  using Backend = boost::multiprecision::cpp_int_backend<Bits, Bits,
      boost::multiprecision::signed_magnitude, boost::multiprecision::checked, void>;
  using Integer = boost::multiprecision::number<Backend, boost::multiprecision::et_off>;
  static_assert(std::numeric_limits<Integer>::digits >= Bits);
  explicit BoostIntegerPolicy(ArithmeticContext& context) noexcept : context_(context) {}
  void Assign(Integer& output, std::uint64_t value) { output = value; }
  bool IsZero(const Integer& value) const noexcept { return value == 0; }
  bool IsNegative(const Integer& value) const noexcept { return value < 0; }
  Integer Add(const Integer& a, const Integer& b) { return a + b; }
  Integer Negate(const Integer& value) { return -value; }
  Integer Multiply(const Integer& a, const Integer& b) { return a * b; }
  void Shift(Integer& value, unsigned amount) { value <<= amount; }
  void Scale(Integer& value, std::uint64_t factor) { value *= factor; }
  int Sign(const Integer& value) const noexcept { return value < 0 ? -1 : (value > 0 ? 1 : 0); }
  template <class Body, class Failure>
  auto Protect(Body&& body, Failure&& failure) noexcept -> decltype(body()) {
    try { return body(); }
    catch (...) { context_.Observe(false); return failure(); }
  }
 private:
  ArithmeticContext& context_;
};
}  // namespace tlfea::contact::represented_interval_crossing::native
