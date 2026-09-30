#pragma once
#include "lib_src/collision/RigidNormalResponse.h"
#include <gtest/gtest.h>
#include <cstring>

namespace rigid_contact_test {
namespace sc=tlfea::contact;
namespace rigid=tl::fea::rigid;
using Vec3=tl::math::Vec3;
inline sc::RigidContactBody Body() {
  return {2,{{{0,0,1,1,0,0,0,1,0}},{3,5,7}},{.25,-.5,1}};
}
inline long double Oracle(const sc::RigidContactBody& b,Vec3 point,Vec3 n) {
  const long double r[]{static_cast<long double>(point.x)-b.center.x,
      static_cast<long double>(point.y)-b.center.y,static_cast<long double>(point.z)-b.center.z};
  const long double normal[]{n.x,n.y,n.z},j[]{b.current_frame.inertia.x,b.current_frame.inertia.y,b.current_frame.inertia.z};
  long double moment[]{r[1]*normal[2]-r[2]*normal[1],r[2]*normal[0]-r[0]*normal[2],
      r[0]*normal[1]-r[1]*normal[0]},sum=0;
  for(unsigned axis=0;axis<3;++axis) {
    long double local=0;
    for(unsigned i=0;i<3;++i)local+=b.current_frame.axes.v[3*i+axis]*moment[i];
    sum+=normal[axis]*normal[axis]/b.mass+local*local/j[axis];
  }
  return sum;
}
} // namespace rigid_contact_test
