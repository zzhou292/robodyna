// SPDX-License-Identifier: MIT
#include "Oracle.h"

#include <boost/multiprecision/cpp_int.hpp>
#include <cstring>

namespace represented_interval_test {
namespace {

using Integer = boost::multiprecision::cpp_int;
using Rational = boost::multiprecision::cpp_rational;

Rational Exact(double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  const bool negative = (bits >> 63) != 0;
  const unsigned encoded = static_cast<unsigned>((bits >> 52) & 0x7ffu);
  const std::uint64_t fraction = bits & ((std::uint64_t{1} << 52) - 1);
  Integer significand =
      encoded ? Integer((std::uint64_t{1} << 52) | fraction)
              : Integer(fraction);
  int exponent = encoded ? static_cast<int>(encoded) - 1023 - 52 : -1074;
  Rational result(significand);
  if (exponent >= 0)
    result *= Integer(1) << exponent;
  else
    result /= Integer(1) << -exponent;
  return negative ? -result : result;
}

struct V {
  Rational x, y, z;
};

V Add(const V& a, const V& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

V Subtract(const V& a, const V& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

V Scale(const V& value, const Rational& scale) {
  return {value.x * scale, value.y * scale, value.z * scale};
}

V Cross(const V& a, const V& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

Rational Dot(const V& a, const V& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

bool Zero(const V& value) {
  return value.x == 0 && value.y == 0 && value.z == 0;
}

V Value(ct::Vec3 value) {
  return {Exact(value.x), Exact(value.y), Exact(value.z)};
}

V At(const ct::RepresentedVertexPath& path, const Rational& time) {
  return Add(Scale(Value(path.endpoint[0]), Rational(1) - time),
             Scale(Value(path.endpoint[1]), time));
}

struct T {
  V vertex[3];
};

T At(const ct::RepresentedTrianglePath& path, const Rational& time) {
  T result;
  for (unsigned i = 0; i < 3; ++i)
    result.vertex[i] = At(path.vertices[i], time);
  return result;
}

V Edge(const T& triangle, unsigned edge) {
  return Subtract(triangle.vertex[(edge + 1) % 3], triangle.vertex[edge]);
}

V Normal(const T& triangle) {
  return Cross(Edge(triangle, 0),
               Subtract(triangle.vertex[2], triangle.vertex[0]));
}

bool Separated(const T& a, const T& b, const V& axis) {
  if (Zero(axis))
    return false;
  Rational minimum_a = Dot(a.vertex[0], axis);
  Rational maximum_a = minimum_a;
  Rational minimum_b = Dot(b.vertex[0], axis);
  Rational maximum_b = minimum_b;
  for (unsigned i = 1; i < 3; ++i) {
    const Rational pa = Dot(a.vertex[i], axis);
    const Rational pb = Dot(b.vertex[i], axis);
    minimum_a = std::min(minimum_a, pa);
    maximum_a = std::max(maximum_a, pa);
    minimum_b = std::min(minimum_b, pb);
    maximum_b = std::max(maximum_b, pb);
  }
  return maximum_a < minimum_b || maximum_b < minimum_a;
}

bool PointInTriangle(const V& point, const T& triangle) {
  const V normal = Normal(triangle);
  if (Dot(Subtract(point, triangle.vertex[0]), normal) != 0)
    return false;
  int sign = 0;
  for (unsigned edge = 0; edge < 3; ++edge) {
    const Rational side =
        Dot(Cross(Edge(triangle, edge),
                  Subtract(point, triangle.vertex[edge])),
            normal);
    const int next = side < 0 ? -1 : (side > 0 ? 1 : 0);
    if (next && sign && next != sign)
      return false;
    if (next)
      sign = next;
  }
  return true;
}

bool SegmentIntersection(const V& a0, const V& a1, const V& b0,
                         const V& b1) {
  const V a = Subtract(a1, a0);
  const V b = Subtract(b1, b0);
  const V delta = Subtract(b0, a0);
  const V normal = Cross(a, b);
  if (!Zero(normal)) {
    if (Dot(delta, normal) != 0)
      return false;
    const Rational denominator = Dot(normal, normal);
    const Rational pa = Dot(Cross(delta, b), normal);
    const Rational pb = Dot(Cross(delta, a), normal);
    return pa >= 0 && pa <= denominator && pb >= 0 && pb <= denominator;
  }
  if (!Zero(Cross(a, delta)))
    return false;
  const Rational* av[3] = {&a.x, &a.y, &a.z};
  const Rational* a0v[3] = {&a0.x, &a0.y, &a0.z};
  const Rational* a1v[3] = {&a1.x, &a1.y, &a1.z};
  const Rational* b0v[3] = {&b0.x, &b0.y, &b0.z};
  const Rational* b1v[3] = {&b1.x, &b1.y, &b1.z};
  unsigned axis = 0;
  while (axis < 2 && *av[axis] == 0)
    ++axis;
  const Rational amin = std::min(*a0v[axis], *a1v[axis]);
  const Rational amax = std::max(*a0v[axis], *a1v[axis]);
  const Rational bmin = std::min(*b0v[axis], *b1v[axis]);
  const Rational bmax = std::max(*b0v[axis], *b1v[axis]);
  return amax >= bmin && bmax >= amin;
}

}  // namespace

ExactOracleResult ExactOracleAt(
    const ct::RepresentedTrianglePath& path_a,
    const ct::RepresentedTrianglePath& path_b, std::uint64_t numerator,
    std::uint64_t denominator) {
  ExactOracleResult result;
  if (!denominator || numerator > denominator)
    return result;
  const Rational time = Rational(numerator) / denominator;
  const T a = At(path_a, time);
  const T b = At(path_b, time);
  const V normal_a = Normal(a);
  const V normal_b = Normal(b);
  if (Zero(normal_a) || Zero(normal_b))
    return result;
  result.valid = true;
  result.coplanar = true;
  for (unsigned i = 0; i < 3; ++i)
    result.coplanar =
        result.coplanar &&
        Dot(Subtract(b.vertex[i], a.vertex[0]), normal_a) == 0 &&
        Dot(Subtract(a.vertex[i], b.vertex[0]), normal_b) == 0;
  if (Separated(a, b, normal_a) || Separated(a, b, normal_b))
    return result;
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      if (Separated(a, b, Cross(Edge(a, i), Edge(b, j))))
        return result;
  if (result.coplanar)
    for (unsigned i = 0; i < 3; ++i)
      if (Separated(a, b, Cross(normal_a, Edge(a, i))) ||
          Separated(a, b, Cross(normal_a, Edge(b, i))))
        return result;
  result.intersects = true;
  for (unsigned i = 0; i < 3; ++i) {
    result.vertex_face =
        result.vertex_face || PointInTriangle(a.vertex[i], b) ||
        PointInTriangle(b.vertex[i], a);
    for (unsigned j = 0; j < 3; ++j)
      result.edge_edge =
          result.edge_edge ||
          SegmentIntersection(a.vertex[i], a.vertex[(i + 1) % 3],
                              b.vertex[j], b.vertex[(j + 1) % 3]);
  }
  return result;
}

bool ExactRootIntervalCertificate(const ct::Vec3 (&vertices)[2][2][3], ct::Vec3 axis) {
  const auto direction=Value(axis);
  if (Zero(direction)) return false;
  T endpoint[2][2];
  for (unsigned side=0; side<2; ++side) {
    T middle;
    for (unsigned vertex=0; vertex<3; ++vertex) {
      for (unsigned time=0; time<2; ++time)
        endpoint[side][time].vertex[vertex]=Value(vertices[side][time][vertex]);
      middle.vertex[vertex]=Scale(Add(endpoint[side][0].vertex[vertex],
                                     endpoint[side][1].vertex[vertex]),Rational(1)/2);
    }
    const V first=Normal(endpoint[side][0]),last=Normal(endpoint[side][1]);
    // Independent midpoint interpolation, rather than the production mixed
    // endpoint-edge formula, constructs twice the middle Bernstein control.
    const V control=Subtract(Subtract(Scale(Normal(middle),4),first),last);
    const Rational* a[]{&first.x,&first.y,&first.z};
    const Rational* b[]{&control.x,&control.y,&control.z};
    const Rational* c[]{&last.x,&last.y,&last.z};
    bool regular=false;
    for (unsigned component=0; component<3; ++component)
      regular=regular || (*a[component]>0 && *b[component]>0 && *c[component]>0) ||
          (*a[component]<0 && *b[component]<0 && *c[component]<0);
    if (!regular) return false;
  }
  Rational lower[2],upper[2];
  for (unsigned side=0; side<2; ++side)
    for (unsigned time=0; time<2; ++time)
      for (unsigned vertex=0; vertex<3; ++vertex) {
        const Rational value=Dot(Subtract(endpoint[side][time].vertex[vertex],
                                          endpoint[0][time].vertex[0]),direction);
        if (!time && !vertex) lower[side]=upper[side]=value;
        else { lower[side]=std::min(lower[side],value); upper[side]=std::max(upper[side],value); }
      }
  return upper[0]<lower[1] || upper[1]<lower[0];
}

}  // namespace represented_interval_test
