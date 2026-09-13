// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ExactPredicates.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace tlfea::contact::fixed_triangle_features::exact {
namespace {

// A finite binary64 coordinate needs at most 2098 aligned bits.  A 3D
// orientation determinant needs fewer than 6304 bits.  The closest-region
// tests multiply two exact dot products and need fewer than 8410 bits,
// including subtraction carries.  Keep an explicit fixed margin.
constexpr unsigned kLimbs = 144;

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

Decoded Decode(double value) noexcept {
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

void Normalize(Integer* value) noexcept {
  while (value->used && !value->limbs[value->used - 1])
    --value->used;
  if (!value->used)
    value->negative = false;
}

Integer Aligned(double value, int common_exponent) noexcept {
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

int CompareMagnitude(const Integer& a, const Integer& b) noexcept {
  if (a.used != b.used)
    return a.used < b.used ? -1 : 1;
  for (unsigned i = a.used; i; --i)
    if (a.limbs[i - 1] != b.limbs[i - 1])
      return a.limbs[i - 1] < b.limbs[i - 1] ? -1 : 1;
  return 0;
}

Integer AddMagnitude(const Integer& a, const Integer& b) noexcept {
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

Integer SubtractMagnitude(const Integer& larger,
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

Integer Add(Integer a, Integer b) noexcept {
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

Integer Negate(Integer value) noexcept {
  if (value.used)
    value.negative = !value.negative;
  return value;
}

Integer Subtract(Integer a, Integer b) noexcept {
  return Add(a, Negate(b));
}

Integer Multiply(const Integer& a, const Integer& b) noexcept {
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

Sign Result(const Integer& value) noexcept {
  if (value.overflow)
    return {};
  return {value.used ? (value.negative ? -1 : 1) : 0, true};
}

int MinimumExponent(const double* values, unsigned count) noexcept {
  int result = 0;
  bool have = false;
  for (unsigned i = 0; i < count; ++i) {
    const auto decoded = Decode(values[i]);
    if (decoded.significand && (!have || decoded.exponent < result)) {
      result = decoded.exponent;
      have = true;
    }
  }
  return result;
}

double Component(Vec3 value, int axis) noexcept {
  return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

struct Integer3 {
  Integer x;
  Integer y;
  Integer z;
};

Integer3 Subtract(Integer3 a, Integer3 b) noexcept {
  return {Subtract(a.x, b.x), Subtract(a.y, b.y),
          Subtract(a.z, b.z)};
}

Integer Dot(Integer3 a, Integer3 b) noexcept {
  return Add(Add(Multiply(a.x, b.x), Multiply(a.y, b.y)),
             Multiply(a.z, b.z));
}

}  // namespace

Sign Orient2D(Vec3 a, Vec3 b, Vec3 c, int dropped_axis) noexcept {
  const int first_axis = dropped_axis == 0 ? 1 : 0;
  const int second_axis = dropped_axis == 2 ? 1 : 2;
  const double values[6]{
      Component(a, first_axis), Component(a, second_axis),
      Component(b, first_axis), Component(b, second_axis),
      Component(c, first_axis), Component(c, second_axis)};
  const int exponent = MinimumExponent(values, 6);
  const Integer ax = Aligned(values[0], exponent);
  const Integer ay = Aligned(values[1], exponent);
  const Integer bx = Aligned(values[2], exponent);
  const Integer by = Aligned(values[3], exponent);
  const Integer cx = Aligned(values[4], exponent);
  const Integer cy = Aligned(values[5], exponent);
  const Integer left =
      Multiply(Subtract(bx, ax), Subtract(cy, ay));
  const Integer right =
      Multiply(Subtract(by, ay), Subtract(cx, ax));
  return Result(Subtract(left, right));
}

Sign Orient3D(Vec3 a, Vec3 b, Vec3 c, Vec3 d) noexcept {
  const double values[12]{a.x, a.y, a.z, b.x, b.y, b.z,
                          c.x, c.y, c.z, d.x, d.y, d.z};
  const int exponent = MinimumExponent(values, 12);
  const Integer ax = Aligned(a.x, exponent);
  const Integer ay = Aligned(a.y, exponent);
  const Integer az = Aligned(a.z, exponent);
  const Integer bx = Aligned(b.x, exponent);
  const Integer by = Aligned(b.y, exponent);
  const Integer bz = Aligned(b.z, exponent);
  const Integer cx = Aligned(c.x, exponent);
  const Integer cy = Aligned(c.y, exponent);
  const Integer cz = Aligned(c.z, exponent);
  const Integer dx = Aligned(d.x, exponent);
  const Integer dy = Aligned(d.y, exponent);
  const Integer dz = Aligned(d.z, exponent);
  const Integer adx = Subtract(ax, dx);
  const Integer ady = Subtract(ay, dy);
  const Integer adz = Subtract(az, dz);
  const Integer bdx = Subtract(bx, dx);
  const Integer bdy = Subtract(by, dy);
  const Integer bdz = Subtract(bz, dz);
  const Integer cdx = Subtract(cx, dx);
  const Integer cdy = Subtract(cy, dy);
  const Integer cdz = Subtract(cz, dz);
  const Integer first = Multiply(
      adx, Subtract(Multiply(bdy, cdz), Multiply(bdz, cdy)));
  const Integer second = Multiply(
      ady, Subtract(Multiply(bdx, cdz), Multiply(bdz, cdx)));
  const Integer third = Multiply(
      adz, Subtract(Multiply(bdx, cdy), Multiply(bdy, cdx)));
  return Result(Add(Subtract(first, second), third));
}

Sign DirectedTriangle(Vec3 a, Vec3 b, Vec3 c,
                      Vec3 direction) noexcept {
  const double values[12]{
      a.x, a.y, a.z, b.x, b.y, b.z,
      c.x, c.y, c.z, direction.x, direction.y, direction.z};
  const int exponent = MinimumExponent(values, 12);
  const Integer3 ai{Aligned(a.x, exponent), Aligned(a.y, exponent),
                    Aligned(a.z, exponent)};
  const Integer3 bi{Aligned(b.x, exponent), Aligned(b.y, exponent),
                    Aligned(b.z, exponent)};
  const Integer3 ci{Aligned(c.x, exponent), Aligned(c.y, exponent),
                    Aligned(c.z, exponent)};
  const Integer3 di{Aligned(direction.x, exponent),
                    Aligned(direction.y, exponent),
                    Aligned(direction.z, exponent)};
  const Integer3 ab = Subtract(bi, ai);
  const Integer3 ac = Subtract(ci, ai);
  const Integer first = Multiply(
      di.x, Subtract(Multiply(ab.y, ac.z), Multiply(ab.z, ac.y)));
  const Integer second = Multiply(
      di.y, Subtract(Multiply(ab.z, ac.x), Multiply(ab.x, ac.z)));
  const Integer third = Multiply(
      di.z, Subtract(Multiply(ab.x, ac.y), Multiply(ab.y, ac.x)));
  return Result(Add(Add(first, second), third));
}

bool ClosestStratum(Vec3 point, const Vec3 (&triangle)[3],
                    ClosestTriangleStratum* output) noexcept {
  if (!output)
    return false;
  const double values[12]{
      point.x, point.y, point.z,
      triangle[0].x, triangle[0].y, triangle[0].z,
      triangle[1].x, triangle[1].y, triangle[1].z,
      triangle[2].x, triangle[2].y, triangle[2].z};
  const int exponent = MinimumExponent(values, 12);
  const Integer3 p{Aligned(point.x, exponent),
                   Aligned(point.y, exponent),
                   Aligned(point.z, exponent)};
  const Integer3 a{Aligned(triangle[0].x, exponent),
                   Aligned(triangle[0].y, exponent),
                   Aligned(triangle[0].z, exponent)};
  const Integer3 b{Aligned(triangle[1].x, exponent),
                   Aligned(triangle[1].y, exponent),
                   Aligned(triangle[1].z, exponent)};
  const Integer3 c{Aligned(triangle[2].x, exponent),
                   Aligned(triangle[2].y, exponent),
                   Aligned(triangle[2].z, exponent)};
  const Integer3 ab = Subtract(b, a);
  const Integer3 ac = Subtract(c, a);
  const Integer d1 = Dot(ab, Subtract(p, a));
  const Integer d2 = Dot(ac, Subtract(p, a));
  const Integer d3 = Dot(ab, Subtract(p, b));
  const Integer d4 = Dot(ac, Subtract(p, b));
  const Integer d5 = Dot(ab, Subtract(p, c));
  const Integer d6 = Dot(ac, Subtract(p, c));
  const Sign s1 = Result(d1);
  const Sign s2 = Result(d2);
  const Sign s3 = Result(d3);
  const Sign s4 = Result(d4);
  const Sign s5 = Result(d5);
  const Sign s6 = Result(d6);
  if (!s1.valid || !s2.valid || !s3.valid || !s4.valid ||
      !s5.valid || !s6.valid)
    return false;
  const Sign at_b = Result(Subtract(d4, d3));
  const Sign at_c = Result(Subtract(d5, d6));
  if (!at_b.valid || !at_c.valid)
    return false;

  if (s1.value <= 0 && s2.value <= 0) {
    *output = {ClosestStratumKind::Vertex, 0};
    return true;
  }
  if (s3.value >= 0 && at_b.value <= 0) {
    *output = {ClosestStratumKind::Vertex, 1};
    return true;
  }
  const Integer vc =
      Subtract(Multiply(d1, d4), Multiply(d3, d2));
  const Sign svc = Result(vc);
  if (!svc.valid)
    return false;
  if (svc.value <= 0 && s1.value >= 0 && s3.value <= 0) {
    *output = {ClosestStratumKind::Edge, 0};
    return true;
  }
  if (s6.value >= 0 && at_c.value <= 0) {
    *output = {ClosestStratumKind::Vertex, 2};
    return true;
  }
  const Integer vb =
      Subtract(Multiply(d5, d2), Multiply(d1, d6));
  const Sign svb = Result(vb);
  if (!svb.valid)
    return false;
  if (svb.value <= 0 && s2.value >= 0 && s6.value <= 0) {
    *output = {ClosestStratumKind::Edge, 2};
    return true;
  }
  const Integer va =
      Subtract(Multiply(d3, d6), Multiply(d5, d4));
  const Sign sva = Result(va);
  if (!sva.valid)
    return false;
  if (sva.value <= 0 && at_b.value >= 0 && at_c.value >= 0) {
    *output = {ClosestStratumKind::Edge, 1};
    return true;
  }
  *output = {ClosestStratumKind::Face, 0};
  return true;
}

}  // namespace tlfea::contact::fixed_triangle_features::exact
