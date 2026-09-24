// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <algorithm>
#include <cstdint>

namespace tl::math::fixed_integer {
// Private fixed-capacity signed-magnitude arithmetic shared by exact predicates.
// Inputs are canonical values produced by this arithmetic, never external limb
// descriptors. Overflow remains sticky and must be checked before using a sign.
struct CheckedSign { int value = 0; bool valid = false; };
template<unsigned LimbCount, class Sign = CheckedSign>
struct Arithmetic {
  static constexpr unsigned kLimbs = LimbCount;
  struct Integer {
    std::uint64_t limbs[kLimbs]{};
    unsigned used = 0;
    bool negative = false;
    bool overflow = false;
  };

  static void Normalize(Integer* value) noexcept {
    while (value->used && !value->limbs[value->used - 1])
      --value->used;
    if (!value->used)
      value->negative = false;
  }

  static int CompareMagnitude(const Integer& a, const Integer& b) noexcept {
    if (a.used != b.used)
      return a.used < b.used ? -1 : 1;
    for (unsigned i = a.used; i; --i)
      if (a.limbs[i - 1] != b.limbs[i - 1])
        return a.limbs[i - 1] < b.limbs[i - 1] ? -1 : 1;
    return 0;
  }

  static Integer AddMagnitude(const Integer& a, const Integer& b) noexcept {
    Integer result;
    result.overflow = a.overflow || b.overflow;
    const unsigned count = std::max(a.used, b.used);
    unsigned __int128 carry = 0;
    for (unsigned i = 0; i < count; ++i) {
      const unsigned __int128 value =
          static_cast<unsigned __int128>(i < a.used ? a.limbs[i] : 0) +
          (i < b.used ? b.limbs[i] : 0) + carry;
      result.limbs[i] = static_cast<std::uint64_t>(value);
      carry = value >> 64;
    }
    result.used = count;
    if (carry) {
      if (count == kLimbs)
        result.overflow = true;
      else {
        result.limbs[count] = static_cast<std::uint64_t>(carry);
        result.used = count + 1;
      }
    }
    return result;
  }

  static Integer SubtractMagnitude(const Integer& larger,
                            const Integer& smaller) noexcept {
    Integer result;
    result.overflow = larger.overflow || smaller.overflow;
    std::uint64_t borrow = 0;
    for (unsigned i = 0; i < larger.used; ++i) {
      const std::uint64_t right =
          i < smaller.used ? smaller.limbs[i] : 0;
      const std::uint64_t first =
          larger.limbs[i] - right;
      const std::uint64_t borrow_first = larger.limbs[i] < right;
      const std::uint64_t second = first - borrow;
      const std::uint64_t borrow_second = first < borrow;
      result.limbs[i] = second;
      borrow = borrow_first | borrow_second;
    }
    result.used = larger.used;
    result.overflow = result.overflow || borrow;
    Normalize(&result);
    return result;
  }

  static Integer Add(const Integer& a, const Integer& b) noexcept {
    if (a.negative == b.negative) {
      auto result = AddMagnitude(a, b);
      result.negative = a.negative && result.used;
      return result;
    }
    const int order = CompareMagnitude(a, b);
    if (!order) {
      Integer result;
      result.overflow = a.overflow || b.overflow;
      return result;
    }
    const bool a_larger = order > 0;
    auto result =
        SubtractMagnitude(a_larger ? a : b, a_larger ? b : a);
    result.negative = (a_larger ? a.negative : b.negative) && result.used;
    return result;
  }

  static Integer Negate(Integer value) noexcept {
    if (value.used)
      value.negative = !value.negative;
    return value;
  }

  static Integer Subtract(const Integer& a, const Integer& b) noexcept {
    return Add(a, Negate(b));
  }

  static Integer Multiply(const Integer& a, const Integer& b) noexcept {
    Integer result;
    result.overflow = a.overflow || b.overflow;
    if (!a.used || !b.used)
      return result;
    for (unsigned i = 0; i < a.used; ++i) {
      unsigned __int128 carry = 0;
      for (unsigned j = 0; j < b.used; ++j) {
        if (i + j >= kLimbs) {
          result.overflow = true;
          continue;
        }
        const unsigned __int128 value =
            static_cast<unsigned __int128>(a.limbs[i]) * b.limbs[j] +
            result.limbs[i + j] + carry;
        result.limbs[i + j] = static_cast<std::uint64_t>(value);
        carry = value >> 64;
      }
      unsigned at = i + b.used;
      while (carry) {
        if (at >= kLimbs) {
          result.overflow = true;
          break;
        }
        const unsigned __int128 value =
            static_cast<unsigned __int128>(result.limbs[at]) + carry;
        result.limbs[at] = static_cast<std::uint64_t>(value);
        carry = value >> 64;
        ++at;
      }
    }
    result.used = std::min<unsigned>(kLimbs, a.used + b.used);
    result.negative = a.negative != b.negative;
    Normalize(&result);
    return result;
  }

  static Sign Result(const Integer& value) noexcept {
    if (value.overflow)
      return {};
    return {value.used ? (value.negative ? -1 : 1) : 0, true};
  }

  struct Integer3 {
    Integer x;
    Integer y;
    Integer z;
  };

  static Integer3 Subtract(const Integer3& a, const Integer3& b) noexcept {
    return {Subtract(a.x, b.x), Subtract(a.y, b.y),
            Subtract(a.z, b.z)};
  }

  static Integer Dot(const Integer3& a, const Integer3& b) noexcept {
    return Add(Add(Multiply(a.x, b.x), Multiply(a.y, b.y)),
               Multiply(a.z, b.z));
  }
};
}  // namespace tl::math::fixed_integer
