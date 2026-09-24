// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "ExactPredicates.h"
#include <algorithm>
#include <cstdint>
#include <cstring>

namespace tlfea::contact::fixed_triangle_features::exact::detail {

inline constexpr unsigned WideLimbs = 144;
inline constexpr unsigned SmallLimbs = 8;
inline constexpr unsigned SmallCoordinateBits = 125;
static_assert(4 * SmallCoordinateBits + 9 <= 64 * SmallLimbs);
static_assert(4 * 2098 + 9 <= 64 * WideLimbs);

// Actual consumed coordinates only, with the same zero-ignoring common scale
// as the original MinimumExponent. If |coordinate integer| < 2^B, every
// degree-four closest-region intermediate needs at most 4B+9 bits. Smaller
// predicates fit the same conservative bound. This record is private data,
// computed afresh in each entry; no caller supplies fast-path eligibility.
struct CoordinateDomain {
  int exponent = 0;
  unsigned coordinate_bits = 0;
  bool finite = true;
  bool narrow() const noexcept {
    return finite && coordinate_bits <= SmallCoordinateBits;
  }
};

// The established fixed integer implementation, parameterized only by storage
// extent. Preserve complete zero initialization, significant-limb loops,
// signed-magnitude rules, carry propagation and overflow behavior at both sizes.
template<unsigned LimbCount> struct Arithmetic {
  static constexpr unsigned kLimbs = LimbCount;
  struct Integer {
    std::uint64_t limbs[kLimbs]{};
    unsigned used = 0;
    bool negative = false;
    bool overflow = false;
  };

  struct Decoded {
    std::uint64_t significand = 0;
    int exponent = 0;
    bool negative = false;
  };

  static Decoded Decode(double value) noexcept {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    const unsigned raw_exponent =
        static_cast<unsigned>((bits >> 52) & 0x7ffu);
    const std::uint64_t fraction = bits & ((std::uint64_t{1} << 52) - 1);
    Decoded result;
    result.negative = (bits >> 63) != 0;
    if (raw_exponent == 0) {
      result.significand = fraction;
      result.exponent = -1074;
    } else {
      result.significand = (std::uint64_t{1} << 52) | fraction;
      result.exponent = static_cast<int>(raw_exponent) - 1023 - 52;
    }
    if (!result.significand)
      result.negative = false;
    return result;
  }

  static void Normalize(Integer* value) noexcept {
    while (value->used && !value->limbs[value->used - 1])
      --value->used;
    if (!value->used)
      value->negative = false;
  }

  static Integer Aligned(double value, int common_exponent) noexcept {
    const auto decoded = Decode(value);
    Integer result;
    if (!decoded.significand)
      return result;
    const unsigned shift =
        static_cast<unsigned>(decoded.exponent - common_exponent);
    const unsigned word = shift / 64;
    const unsigned bit = shift % 64;
    if (word >= kLimbs) {
      result.overflow = true;
      return result;
    }
    result.limbs[word] = decoded.significand << bit;
    result.used = word + 1;
    if (bit && word + 1 < kLimbs) {
      result.limbs[word + 1] = decoded.significand >> (64 - bit);
      if (result.limbs[word + 1])
        result.used = word + 2;
    } else if (bit && decoded.significand >> (64 - bit)) {
      result.overflow = true;
    }
    result.negative = decoded.negative;
    return result;
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

  static CoordinateDomain AnalyzeCoordinates(const double* values, unsigned count) noexcept {
    CoordinateDomain result;
    int maximum = 0;
    bool have = false;
    for (unsigned i = 0; i < count; ++i) {
      const auto decoded = Decode(values[i]);
      // Decode maps only NaN/infinity's exponent field to 972. Still compute the
      // old scale for those inputs so fallback keeps the old behavior unchanged.
      if (decoded.exponent > 971) result.finite = false;
      if (decoded.significand) {
        if (!have || decoded.exponent < result.exponent)
          result.exponent = decoded.exponent;
        if (!have || decoded.exponent > maximum)
          maximum = decoded.exponent;
        have = true;
      }
    }
    result.coordinate_bits = have
        ? 53u + static_cast<unsigned>(maximum - result.exponent) : 0u;
    return result;
  }

  static double Component(Vec3 value, int axis) noexcept {
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
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

inline CoordinateDomain AnalyzeCoordinates(const double* values, unsigned count) noexcept {
  return Arithmetic<WideLimbs>::AnalyzeCoordinates(values, count);
}

}  // namespace tlfea::contact::fixed_triangle_features::exact::detail
