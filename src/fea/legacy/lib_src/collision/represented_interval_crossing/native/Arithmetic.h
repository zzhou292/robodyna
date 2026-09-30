// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Identity.h"
#include "BoostIntegerPolicy.h"
namespace tlfea::contact::represented_interval_crossing::native {
template <unsigned Bits, class IntegerPolicy = BoostIntegerPolicy<Bits>>
struct Arithmetic {
  using ExactInteger = typename IntegerPolicy::Integer;
  static_assert(Bits == 512 || Bits == 16384);
  TL_MATH_HOST_DEVICE explicit Arithmetic(ArithmeticContext& context) noexcept : context_(context), integers_(context) {}
  Arithmetic(const Arithmetic&) = delete;
  Arithmetic& operator=(const Arithmetic&) = delete;
  TL_MATH_HOST_DEVICE bool Healthy() const noexcept {
    if constexpr (IntegerPolicy::TracksErrors) return context_.valid();
    else return true;
  }


  struct Dyadic {
    ExactInteger numerator{};
    int exponent = 0;
  };

  TL_MATH_HOST_DEVICE Dyadic Exact(double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "binary64 representation");
    portable::memcpy(&bits, &value, sizeof(bits));
    const bool negative = (bits >> 63) != 0;
    const unsigned encoded_exponent =
        static_cast<unsigned>((bits >> 52) & 0x7ffu);
    const std::uint64_t fraction = bits & ((std::uint64_t{1} << 52) - 1);
    Dyadic result;
    if (encoded_exponent == 0) {
      integers_.Assign(result.numerator, fraction);
      result.exponent = -1074;
    } else {
      integers_.Assign(result.numerator, (std::uint64_t{1} << 52) | fraction);
      result.exponent = static_cast<int>(encoded_exponent) - 1023 - 52;
    }
    if (negative)
      result.numerator = integers_.Negate(result.numerator);
    return result;
  }

  TL_MATH_HOST_DEVICE Dyadic Add(Dyadic a, Dyadic b) {
    if (integers_.IsZero(a.numerator))
      return b;
    if (integers_.IsZero(b.numerator))
      return a;
    const int exponent = portable::min(a.exponent, b.exponent);
    const auto shift = [this](ExactInteger* value, unsigned amount) {
      const bool negative = integers_.IsNegative(*value);
      if (negative)
        *value = integers_.Negate(*value);
      integers_.Shift(*value, amount);
      if (negative)
        *value = integers_.Negate(*value);
    };
    shift(&a.numerator, static_cast<unsigned>(a.exponent - exponent));
    shift(&b.numerator, static_cast<unsigned>(b.exponent - exponent));
    return {integers_.Add(a.numerator, b.numerator), exponent};
  }

  TL_MATH_HOST_DEVICE Dyadic Negate(Dyadic value) {
    value.numerator = integers_.Negate(value.numerator);
    return value;
  }

  TL_MATH_HOST_DEVICE Dyadic Subtract(Dyadic a, Dyadic b) { return Add(a, Negate(b)); }

  TL_MATH_HOST_DEVICE Dyadic Multiply(const Dyadic& a, const Dyadic& b) {
    return {integers_.Multiply(a.numerator, b.numerator), a.exponent + b.exponent};
  }

  TL_MATH_HOST_DEVICE Dyadic Scale(Dyadic value, std::uint64_t factor) {
    integers_.Scale(value.numerator, factor);
    return value;
  }

  TL_MATH_HOST_DEVICE int Sign(const Dyadic& value) noexcept {
    return integers_.Sign(value.numerator);
  }

  TL_MATH_HOST_DEVICE int Compare(const Dyadic& a, const Dyadic& b) {
    return Sign(Subtract(a, b));
  }

  struct ExactVec3 {
    Dyadic x, y, z;
  };

  TL_MATH_HOST_DEVICE ExactVec3 Add(const ExactVec3& a, const ExactVec3& b) {
    return {Add(a.x, b.x), Add(a.y, b.y), Add(a.z, b.z)};
  }

  TL_MATH_HOST_DEVICE ExactVec3 Subtract(const ExactVec3& a, const ExactVec3& b) {
    return {Subtract(a.x, b.x), Subtract(a.y, b.y),
            Subtract(a.z, b.z)};
  }

  TL_MATH_HOST_DEVICE ExactVec3 Cross(const ExactVec3& a, const ExactVec3& b) {
    return {Subtract(Multiply(a.y, b.z), Multiply(a.z, b.y)),
            Subtract(Multiply(a.z, b.x), Multiply(a.x, b.z)),
            Subtract(Multiply(a.x, b.y), Multiply(a.y, b.x))};
  }

  TL_MATH_HOST_DEVICE Dyadic Dot(const ExactVec3& a, const ExactVec3& b) {
    return Add(Add(Multiply(a.x, b.x), Multiply(a.y, b.y)),
               Multiply(a.z, b.z));
  }

  TL_MATH_HOST_DEVICE bool Zero(const ExactVec3& value) noexcept {
    return Sign(value.x) == 0 && Sign(value.y) == 0 && Sign(value.z) == 0;
  }

  TL_MATH_HOST_DEVICE Dyadic Component(const ExactVec3& value, unsigned component) {
    return component == 0 ? value.x : (component == 1 ? value.y : value.z);
  }

  TL_MATH_HOST_DEVICE double Component(Vec3 value, unsigned component) noexcept {
    return component == 0 ? value.x : (component == 1 ? value.y : value.z);
  }

  TL_MATH_HOST_DEVICE ExactVec3 At(const RepresentedVertexPath& path, DyadicTime time) {
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

  TL_MATH_HOST_DEVICE ExactTriangle At(const RepresentedTrianglePath& path, DyadicTime time) {
    ExactTriangle result;
    for (unsigned i = 0; i < 3; ++i)
      result.vertex[i] = At(path.vertices[i], time);
    return result;
  }

  TL_MATH_HOST_DEVICE bool CommonTranslation(
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

 protected:
  ArithmeticContext& context_;
  IntegerPolicy integers_;
};

}  // namespace tlfea::contact::represented_interval_crossing::native
