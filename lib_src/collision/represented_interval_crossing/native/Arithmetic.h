// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Identity.h"
#include <boost/multiprecision/cpp_int.hpp>
#include <limits>
namespace tlfea::contact::represented_interval_crossing::native {
template <unsigned Bits>
struct Arithmetic {
  using ExactBackend = boost::multiprecision::cpp_int_backend<
      Bits, Bits, boost::multiprecision::signed_magnitude,
      boost::multiprecision::checked, void>;
  using ExactInteger =
      boost::multiprecision::number<ExactBackend,
                                    boost::multiprecision::et_off>;
  static_assert(Bits == 512 || Bits == 16384);
  static_assert(std::numeric_limits<ExactInteger>::digits >= Bits);

  struct Dyadic {
    ExactInteger numerator = 0;
    int exponent = 0;
  };

  static Dyadic Exact(double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "binary64 representation");
    std::memcpy(&bits, &value, sizeof(bits));
    const bool negative = (bits >> 63) != 0;
    const unsigned encoded_exponent =
        static_cast<unsigned>((bits >> 52) & 0x7ffu);
    const std::uint64_t fraction = bits & ((std::uint64_t{1} << 52) - 1);
    Dyadic result;
    if (encoded_exponent == 0) {
      result.numerator = fraction;
      result.exponent = -1074;
    } else {
      result.numerator = (std::uint64_t{1} << 52) | fraction;
      result.exponent = static_cast<int>(encoded_exponent) - 1023 - 52;
    }
    if (negative)
      result.numerator = -result.numerator;
    return result;
  }

  static Dyadic Add(Dyadic a, Dyadic b) {
    if (a.numerator == 0)
      return b;
    if (b.numerator == 0)
      return a;
    const int exponent = std::min(a.exponent, b.exponent);
    const auto shift = [](ExactInteger* value, unsigned amount) {
      const bool negative = *value < 0;
      if (negative)
        *value = -*value;
      *value <<= amount;
      if (negative)
        *value = -*value;
    };
    shift(&a.numerator, static_cast<unsigned>(a.exponent - exponent));
    shift(&b.numerator, static_cast<unsigned>(b.exponent - exponent));
    return {a.numerator + b.numerator, exponent};
  }

  static Dyadic Negate(Dyadic value) {
    value.numerator = -value.numerator;
    return value;
  }

  static Dyadic Subtract(Dyadic a, Dyadic b) { return Add(a, Negate(b)); }

  static Dyadic Multiply(const Dyadic& a, const Dyadic& b) {
    return {a.numerator * b.numerator, a.exponent + b.exponent};
  }

  static Dyadic Scale(Dyadic value, std::uint64_t factor) {
    value.numerator *= factor;
    return value;
  }

  static int Sign(const Dyadic& value) noexcept {
    return value.numerator < 0 ? -1 : (value.numerator > 0 ? 1 : 0);
  }

  static int Compare(const Dyadic& a, const Dyadic& b) {
    return Sign(Subtract(a, b));
  }

  struct ExactVec3 {
    Dyadic x, y, z;
  };

  static ExactVec3 Add(const ExactVec3& a, const ExactVec3& b) {
    return {Add(a.x, b.x), Add(a.y, b.y), Add(a.z, b.z)};
  }

  static ExactVec3 Subtract(const ExactVec3& a, const ExactVec3& b) {
    return {Subtract(a.x, b.x), Subtract(a.y, b.y),
            Subtract(a.z, b.z)};
  }

  static ExactVec3 Cross(const ExactVec3& a, const ExactVec3& b) {
    return {Subtract(Multiply(a.y, b.z), Multiply(a.z, b.y)),
            Subtract(Multiply(a.z, b.x), Multiply(a.x, b.z)),
            Subtract(Multiply(a.x, b.y), Multiply(a.y, b.x))};
  }

  static Dyadic Dot(const ExactVec3& a, const ExactVec3& b) {
    return Add(Add(Multiply(a.x, b.x), Multiply(a.y, b.y)),
               Multiply(a.z, b.z));
  }

  static bool Zero(const ExactVec3& value) noexcept {
    return Sign(value.x) == 0 && Sign(value.y) == 0 && Sign(value.z) == 0;
  }

  static Dyadic Component(const ExactVec3& value, unsigned component) {
    return component == 0 ? value.x : (component == 1 ? value.y : value.z);
  }

  static double Component(Vec3 value, unsigned component) noexcept {
    return component == 0 ? value.x : (component == 1 ? value.y : value.z);
  }

  static ExactVec3 At(const RepresentedVertexPath& path, DyadicTime time) {
    const std::uint64_t denominator = std::uint64_t{1} << time.depth;
    ExactVec3 result;
    Dyadic* target[3] = {&result.x, &result.y, &result.z};
    for (unsigned component = 0; component < 3; ++component) {
      const Dyadic a = Scale(Exact(Component(path.endpoint[0], component)),
                             denominator - time.numerator);
      const Dyadic b =
          Scale(Exact(Component(path.endpoint[1], component)),
                time.numerator);
      *target[component] = Add(a, b);
      target[component]->exponent -= static_cast<int>(time.depth);
    }
    return result;
  }

  struct ExactTriangle {
    ExactVec3 vertex[3];
  };

  static ExactTriangle At(const RepresentedTrianglePath& path, DyadicTime time) {
    ExactTriangle result;
    for (unsigned i = 0; i < 3; ++i)
      result.vertex[i] = At(path.vertices[i], time);
    return result;
  }

  static bool CommonTranslation(
      const RepresentedTrianglePath& a,
      const RepresentedTrianglePath& b) {
    Dyadic reference[3];
    for (unsigned component = 0; component < 3; ++component) {
      reference[component] = Subtract(
          Exact(Component(a.vertices[0].endpoint[1], component)),
          Exact(Component(a.vertices[0].endpoint[0], component)));
    }
    const RepresentedTrianglePath* paths[2]{&a, &b};
    for (const auto* path : paths)
      for (const auto& vertex : path->vertices)
        for (unsigned component = 0; component < 3; ++component) {
          const auto displacement = Subtract(
              Exact(Component(vertex.endpoint[1], component)),
              Exact(Component(vertex.endpoint[0], component)));
          if (Compare(displacement, reference[component]) != 0)
            return false;
        }
    return true;
  }

};

}  // namespace tlfea::contact::represented_interval_crossing::native
