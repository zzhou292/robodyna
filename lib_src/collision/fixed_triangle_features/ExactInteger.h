// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "ExactPredicates.h"
#include "../../math/FixedInteger.h"
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
template<unsigned LimbCount> struct Arithmetic
    : tl::math::fixed_integer::Arithmetic<LimbCount, Sign> {
  using Base = tl::math::fixed_integer::Arithmetic<LimbCount, Sign>;
  using typename Base::Integer;
  using typename Base::Integer3;
  using Base::kLimbs;
  using Base::Normalize;
  using Base::CompareMagnitude;
  using Base::AddMagnitude;
  using Base::SubtractMagnitude;
  using Base::Add;
  using Base::Negate;
  using Base::Subtract;
  using Base::Multiply;
  using Base::Result;
  using Base::Dot;

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

};

inline CoordinateDomain AnalyzeCoordinates(const double* values, unsigned count) noexcept {
  return Arithmetic<WideLimbs>::AnalyzeCoordinates(values, count);
}

}  // namespace tlfea::contact::fixed_triangle_features::exact::detail
