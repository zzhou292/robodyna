// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/SurfaceContactTypes.h"
#include <boost/multiprecision/cpp_int.hpp>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>

// Independent unbounded rational geometry, shared only by qualification tests.
// Source binary64 coordinate differences are formed exactly, never rounded first.
namespace cone_direction_test {
using Integer=boost::multiprecision::cpp_int;
using Rational=boost::multiprecision::cpp_rational;
using V=std::array<Rational,3>;
using Rays=std::array<V,8>;
inline Rational Exact(double value) {
  std::uint64_t bits=0;std::memcpy(&bits,&value,sizeof(bits));
  const unsigned raw=static_cast<unsigned>((bits>>52)&0x7ffu);
  if(raw==0x7ffu) throw std::invalid_argument("Nonfinite cone oracle coordinate");
  const auto fraction=bits&((std::uint64_t{1}<<52)-1);
  Rational result(raw?((std::uint64_t{1}<<52)|fraction):fraction);
  const int exponent=raw?static_cast<int>(raw)-1023-52:-1074;
  if(exponent>=0)result*=Integer(1)<<exponent;else result/=Integer(1)<<-exponent;
  return bits>>63?-result:result;
}
inline V Exact(tlfea::contact::Vec3 value) {return {Exact(value.x),Exact(value.y),Exact(value.z)};}
inline V Subtract(const V& a,const V& b) {return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline V Cross(const V& a,const V& b) {return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline Rational Dot(const V& a,const V& b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline Rational ArmDot(tlfea::contact::Vec3 point,tlfea::contact::Vec3 origin,tlfea::contact::Vec3 axis) {
  return Dot(Subtract(Exact(point),Exact(origin)),Exact(axis));
}
template <std::size_t Count>
inline bool Strict(const std::array<V, Count>& rays,const V& axis) {
  bool positive=true,negative=true;
  for(const auto& ray:rays){const auto dot=Dot(axis,ray);positive=positive&&dot>0;negative=negative&&dot<0;}
  return positive||negative;
}
template <std::size_t Count>
inline bool Feasible(const std::array<V, Count>& rays) {
  // Independent nested subset enumeration of exact closest-face directions.
  // No production iterator, rounded construction or tolerance is involved.
  for(unsigned i=0;i<Count;++i)if(Strict(rays,rays[i]))return true;
  for(unsigned i=0;i<Count;++i)for(unsigned j=i+1;j<Count;++j) {
    const auto e=Subtract(rays[j],rays[i]);
    if(Strict(rays,Cross(e,Cross(rays[i],e))))return true;
  }
  for(unsigned i=0;i<Count;++i)for(unsigned j=i+1;j<Count;++j)for(unsigned k=j+1;k<Count;++k)
    if(Strict(rays,Cross(Subtract(rays[j],rays[i]),Subtract(rays[k],rays[i]))))return true;
  return false;
}
} // namespace cone_direction_test
