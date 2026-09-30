// SPDX-License-Identifier: MIT
#pragma once

#include "lib_src/collision/SurfaceContactTypes.h"

#include <array>
#include <boost/multiprecision/cpp_dec_float.hpp>

namespace current_regularity_test::oracle {

using Decimal =
    boost::multiprecision::number<
        boost::multiprecision::cpp_dec_float<100>>;

struct Vec {
  Decimal x, y, z;
};

inline Vec Value(tlfea::contact::Vec3 value) {
  return {Decimal(value.x), Decimal(value.y), Decimal(value.z)};
}
inline Vec Add(Vec a, Vec b) {
  return {a.x+b.x, a.y+b.y, a.z+b.z};
}
inline Vec Subtract(Vec a, Vec b) {
  return {a.x-b.x, a.y-b.y, a.z-b.z};
}
inline Vec Scale(Vec a, Decimal b) {
  return {a.x*b, a.y*b, a.z*b};
}
inline Vec Cross(Vec a, Vec b) {
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z,
          a.x*b.y-a.y*b.x};
}
inline Decimal Dot(Vec a, Vec b) {
  return a.x*b.x+a.y*b.y+a.z*b.z;
}

inline Vec Q4Cross(
    const std::array<tlfea::contact::Vec3, 4>& points,
    Decimal u, Decimal v) {
  const auto p0 = Value(points[0]);
  const auto p1 = Value(points[1]);
  const auto p2 = Value(points[2]);
  const auto p3 = Value(points[3]);
  const Decimal fourth = Decimal(1)/4;
  const Vec xu = Scale(Add(
      Add(Scale(p0,1+v),Scale(p1,-(1+v))),
      Add(Scale(p2,-(1-v)),Scale(p3,1-v))), fourth);
  const Vec xv = Scale(Add(
      Add(Scale(p0,1+u),Scale(p1,1-u)),
      Add(Scale(p2,-(1-u)),Scale(p3,-(1+u)))), fourth);
  return Cross(xu,xv);
}

inline bool Q4ChartPositive(
    const std::array<tlfea::contact::Vec3, 4>& points,
    tlfea::contact::Vec3 direction) {
  const auto chart = Value(direction);
  for (int i = 0; i <= 16; ++i)
    for (int j = 0; j <= 16; ++j) {
      const Decimal u = Decimal(-1)+Decimal(i)/8;
      const Decimal v = Decimal(-1)+Decimal(j)/8;
      if (!(Dot(Q4Cross(points,u,v),chart) > 0)) return false;
    }
  return true;
}

inline Decimal DirectedTriangle(
    tlfea::contact::Vec3 a, tlfea::contact::Vec3 b,
    tlfea::contact::Vec3 c, tlfea::contact::Vec3 direction) {
  return Dot(Cross(Subtract(Value(b),Value(a)),
                   Subtract(Value(c),Value(a))),
             Value(direction));
}

}  // namespace current_regularity_test::oracle
