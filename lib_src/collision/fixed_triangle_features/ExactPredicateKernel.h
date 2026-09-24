// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ExactInteger.h"

namespace tlfea::contact::fixed_triangle_features::exact::detail {

template<unsigned Limbs> struct PredicateKernel : Arithmetic<Limbs> {
  using Base = Arithmetic<Limbs>;
  using typename Base::Integer;
  using typename Base::Integer3;
  using Base::Aligned;
  using Base::Add;
  using Base::Subtract;
  using Base::Multiply;
  using Base::Dot;
  using Base::Result;
  static Sign Orient2D(const double (&values)[6], int exponent) noexcept {
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

  static Sign Orient3D(Vec3 a, Vec3 b, Vec3 c, Vec3 d, int exponent) noexcept {
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

  static Sign DirectedTriangle(Vec3 a, Vec3 b, Vec3 c,
                                 Vec3 direction, int exponent) noexcept {
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

  static bool ClosestStratum(Vec3 point, const Vec3 (&triangle)[3],
                              ClosestTriangleStratum* output, int exponent) noexcept {
    if (!output)
      return false;
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
};

// Only qualification's separate wide binary selects Wide. Production wrappers
// instantiate Adaptive directly; there is no runtime mode or global override.
enum class Storage { Adaptive, Wide };

template<Storage storage> Sign EvaluateOrient2D(Vec3 a, Vec3 b, Vec3 c,
                                               int dropped_axis) noexcept {
  const int first_axis = dropped_axis == 0 ? 1 : 0;
  const int second_axis = dropped_axis == 2 ? 1 : 2;
  using A = Arithmetic<WideLimbs>;
  const double values[6]{A::Component(a, first_axis), A::Component(a, second_axis),
      A::Component(b, first_axis), A::Component(b, second_axis),
      A::Component(c, first_axis), A::Component(c, second_axis)};
  const auto domain = AnalyzeCoordinates(values, 6);
  if constexpr (storage == Storage::Adaptive)
    if (domain.narrow()) return PredicateKernel<SmallLimbs>::Orient2D(values, domain.exponent);
  return PredicateKernel<WideLimbs>::Orient2D(values, domain.exponent);
}

template<Storage storage> Sign EvaluateOrient3D(Vec3 a, Vec3 b, Vec3 c, Vec3 d) noexcept {
  const double values[12]{a.x, a.y, a.z, b.x, b.y, b.z,
                         c.x, c.y, c.z, d.x, d.y, d.z};
  const auto domain = AnalyzeCoordinates(values, 12);
  if constexpr (storage == Storage::Adaptive)
    if (domain.narrow()) return PredicateKernel<SmallLimbs>::Orient3D(a, b, c, d, domain.exponent);
  return PredicateKernel<WideLimbs>::Orient3D(a, b, c, d, domain.exponent);
}

template<Storage storage> Sign EvaluateDirectedTriangle(Vec3 a, Vec3 b, Vec3 c,
                                                       Vec3 direction) noexcept {
  const double values[12]{a.x, a.y, a.z, b.x, b.y, b.z,
                         c.x, c.y, c.z, direction.x, direction.y, direction.z};
  const auto domain = AnalyzeCoordinates(values, 12);
  if constexpr (storage == Storage::Adaptive)
    if (domain.narrow()) return PredicateKernel<SmallLimbs>::DirectedTriangle(a, b, c, direction, domain.exponent);
  return PredicateKernel<WideLimbs>::DirectedTriangle(a, b, c, direction, domain.exponent);
}

template<Storage storage> bool EvaluateClosestStratum(Vec3 point, const Vec3 (&triangle)[3],
                                                     ClosestTriangleStratum* output) noexcept {
  // Retain the original early return: a null output never reads triangle data.
  if (!output) return false;
  const double values[12]{point.x, point.y, point.z,
      triangle[0].x, triangle[0].y, triangle[0].z,
      triangle[1].x, triangle[1].y, triangle[1].z,
      triangle[2].x, triangle[2].y, triangle[2].z};
  const auto domain = AnalyzeCoordinates(values, 12);
  if constexpr (storage == Storage::Adaptive)
    if (domain.narrow()) return PredicateKernel<SmallLimbs>::ClosestStratum(point, triangle, output, domain.exponent);
  return PredicateKernel<WideLimbs>::ClosestStratum(point, triangle, output, domain.exponent);
}

}  // namespace tlfea::contact::fixed_triangle_features::exact::detail
