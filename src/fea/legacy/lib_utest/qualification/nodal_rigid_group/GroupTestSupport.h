#pragma once
#include "lib_src/constraints/NodalRigidGroupModel.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>

namespace rigid_test {
namespace fe=tl::fea;
namespace rigid=tl::fea::rigid;
using Vec3=tl::math::Vec3;
inline std::array<fe::NodalRigidGroupMember,4> Members(std::uint64_t first=101,std::size_t offset=0) {
  const Vec3 positions[]{{-.06,.01,.025},{.09,-.02,.05},{.03,.07,-.04},{-.02,-.05,.08}};
  const double masses[]{2,3,5,7},inertias[]{.001,.003,.006,.004};
  std::array<fe::NodalRigidGroupMember,4> result;
  for(unsigned i=0;i<4;++i) result[i]={first+i,offset+i,positions[i],masses[i],inertias[i],.6*inertias[i],.4*inertias[i]};
  return result;
}
inline fe::NodalRigidGroupModelInput Input(const fe::NodalRigidGroupInput* groups,std::size_t count,std::size_t nodes=128) {
  fe::NodalRigidGroupModelInput in;
  in.source_instance_id=0x5249474944475250; in.groups=groups; in.group_count=count;
  in.global_node_count=nodes; in.source_units={1000,.001}; return in;
}
inline double Get(Vec3 v,unsigned i) { return i==0?v.x:i==1?v.y:v.z; }
inline void Near(Vec3 a,Vec3 b,double tolerance=1e-12) {
  for(unsigned i=0;i<3;++i) EXPECT_NEAR(Get(a,i),Get(b,i),tolerance);
}
inline rigid::PrincipalFrame Frame() {
  // Exact signed permutation: world axes are cyclic principal axes.
  return {{{0,0,1,1,0,0,0,1,0}},{2,3,5}};
}
} // namespace rigid_test
