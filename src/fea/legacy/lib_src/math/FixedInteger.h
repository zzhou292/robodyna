// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "HostDevice.h"
#include <cstdint>

namespace tl::math::fixed_integer {
namespace detail {
TL_MATH_HOST_DEVICE inline unsigned Maximum(unsigned a, unsigned b) noexcept {
  return a < b ? b : a;
}
TL_MATH_HOST_DEVICE inline unsigned Minimum(unsigned a, unsigned b) noexcept {
  return b < a ? b : a;
}
}  // namespace detail
// Private fixed-capacity signed-magnitude arithmetic shared by exact predicates.
// Inputs are canonical values produced by this arithmetic, never external limb
// descriptors. Overflow remains sticky and must be checked before using a sign.
struct CheckedSign { int value = 0; bool valid = false; };
template<unsigned LimbCount, class Sign = CheckedSign>
struct Arithmetic {
  static_assert(LimbCount > 0);
  static constexpr unsigned kLimbs = LimbCount;
  struct Integer {
    std::uint64_t limbs[kLimbs]{};
    unsigned used = 0;
    bool negative = false;
    bool overflow = false;
  };

  TL_MATH_HOST_DEVICE static void Normalize(Integer* value) noexcept {
    while (value->used && !value->limbs[value->used - 1])
      --value->used;
    if (!value->used)
      value->negative = false;
  }

  TL_MATH_HOST_DEVICE static int CompareMagnitude(const Integer& a, const Integer& b) noexcept {
    if (a.used != b.used)
      return a.used < b.used ? -1 : 1;
    for (unsigned i = a.used; i; --i)
      if (a.limbs[i - 1] != b.limbs[i - 1])
        return a.limbs[i - 1] < b.limbs[i - 1] ? -1 : 1;
    return 0;
  }

  TL_MATH_HOST_DEVICE static Integer AddMagnitude(const Integer& a, const Integer& b) noexcept {
    Integer result;
    result.overflow = a.overflow || b.overflow;
    const unsigned count = detail::Maximum(a.used, b.used);
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

  TL_MATH_HOST_DEVICE static Integer SubtractMagnitude(const Integer& larger,
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

  TL_MATH_HOST_DEVICE static Integer Add(const Integer& a, const Integer& b) noexcept {
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

  TL_MATH_HOST_DEVICE static Integer Negate(Integer value) noexcept {
    if (value.used)
      value.negative = !value.negative;
    return value;
  }

  TL_MATH_HOST_DEVICE static Integer Subtract(const Integer& a, const Integer& b) noexcept {
    return Add(a, Negate(b));
  }

  TL_MATH_HOST_DEVICE static Integer Multiply(const Integer& a, const Integer& b) noexcept {
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
    result.used = detail::Minimum(kLimbs, a.used + b.used);
    result.negative = a.negative != b.negative;
    Normalize(&result);
    return result;
  }

  TL_MATH_HOST_DEVICE static Sign Result(const Integer& value) noexcept {
    if (value.overflow)
      return {};
    return {value.used ? (value.negative ? -1 : 1) : 0, true};
  }

  struct Integer3 {
    Integer x;
    Integer y;
    Integer z;
  };

  TL_MATH_HOST_DEVICE static Integer3 Subtract(const Integer3& a, const Integer3& b) noexcept {
    return {Subtract(a.x, b.x), Subtract(a.y, b.y),
            Subtract(a.z, b.z)};
  }

  TL_MATH_HOST_DEVICE static Integer Dot(const Integer3& a, const Integer3& b) noexcept {
    return Add(Add(Multiply(a.x, b.x), Multiply(a.y, b.y)),
               Multiply(a.z, b.z));
  }
  // Scalar construction preserves complete zero initialization and normalized
  // positive zero. The full unsigned input is retained without signed casts.
  TL_MATH_HOST_DEVICE static Integer FromU64(std::uint64_t value) noexcept {
    Integer result;
    result.limbs[0] = value;
    result.used = value ? 1 : 0;
    return result;
  }

  // Shift the magnitude, retaining the sign and sticky overflow. Amount zero
  // and word-boundary shifts never evaluate a shift by 64. Lost high nonzero
  // bits are explicit overflow; the truncated low limbs are diagnostic only.
  TL_MATH_HOST_DEVICE static Integer ShiftLeft(const Integer& value,
                                               unsigned amount) noexcept {
    Integer result;
    result.overflow = value.overflow;
    if (!value.used) return result;
    const unsigned words = amount / 64, bits = amount % 64;
    if (words >= kLimbs) {
      result.overflow = true;
      return result;
    }
    for (unsigned i = 0; i < value.used; ++i) {
      const unsigned at = i + words;
      if (at >= kLimbs) {
        result.overflow = result.overflow || value.limbs[i] != 0;
        continue;
      }
      result.limbs[at] |= value.limbs[i] << bits;
      if (bits) {
        const auto high = value.limbs[i] >> (64 - bits);
        if (at + 1 < kLimbs) result.limbs[at + 1] |= high;
        else result.overflow = result.overflow || high != 0;
      }
    }
    result.used = detail::Minimum(kLimbs, value.used + words + (bits != 0));
    result.negative = value.negative;
    Normalize(&result);
    return result;
  }

  // A validity-bearing comparison, never an overflow-to-zero conversion.
  // Native dyadic alignment/first-error policy remains owned by its caller.
  TL_MATH_HOST_DEVICE static Sign Compare(const Integer& a,
                                           const Integer& b) noexcept {
    if (a.overflow || b.overflow) return {};
    if (a.negative != b.negative) return {a.negative ? -1 : 1, true};
    const int magnitude = CompareMagnitude(a, b);
    return {a.negative ? -magnitude : magnitude, true};
  }

};
}  // namespace tl::math::fixed_integer
