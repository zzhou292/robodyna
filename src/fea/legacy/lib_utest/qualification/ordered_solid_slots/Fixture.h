// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/solvers/NodalRepeatedForceAssembly.h"
#include "lib_src/solvers/NodalRepeatedStiffness.h"
#include <array>
#include <cstring>
#include <limits>
#include <gtest/gtest.h>
namespace slot_test {
using Status = tl::fea::NodalForceAssemblyStatus;
struct Packet {
  std::size_t nodes[8]{0,1,2,3,4,4,5,5};
  tl::math::Vec3 force[8]{};
  double destination[6][9]{};
  double stiffness[9]{}, increments[8]{};
  Status status = Status::InvalidView;
  bool scalar = false, null_rotations = false;
  int sign = 1;
};
#if defined(__CUDACC__)
__host__ __device__
#endif
inline tl::fea::DeviceNodalForceView View(Packet& p) {
  return {p.destination[0],p.destination[1],p.destination[2],
    p.null_rotations ? nullptr : p.destination[3],
    p.null_rotations ? nullptr : p.destination[4],
    p.null_rotations ? nullptr : p.destination[5],9,17};
}
#if defined(__CUDACC__)
__host__ __device__
#endif
inline void Execute(Packet& p) {
  p.status = p.scalar ? tl::fea::AccumulateRepeatedNodalStiffness<8>(
      p.nodes,p.increments,p.stiffness,9) :
    tl::fea::AccumulateRepeatedNodalTranslationalForces<8>(p.nodes,p.force,View(p),p.sign);
}
template<class T> inline auto Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> result;
  std::memcpy(result.data(),&value,sizeof(T)); return result;
}
inline Packet Initial() {
  Packet p;
  for (unsigned c=0;c<6;++c) for (unsigned n=0;n<9;++n)
    p.destination[c][n] = c<3 ? 0 : -0.0;
  for (unsigned c=0;c<3;++c) {
    p.destination[c][4]=1e16;
    p.destination[c][5]=1e16;
    p.destination[c][8]=std::numeric_limits<double>::quiet_NaN(); // Unused.
  }
  p.force[4]=p.force[6]={-1e16,-1e16,-1e16};
  p.force[5]=p.force[7]={1,1,1};
  for (double& x:p.increments) x=1;
  p.stiffness[4]=0x1p53; p.stiffness[5]=16;
  p.stiffness[8]=std::numeric_limits<double>::quiet_NaN();
  return p;
}
template<class Run> inline void OrderAndUntouchedRotation(Run run) {
  for (bool absent:{false,true}) {
    auto p=Initial();p.null_rotations=absent;
    auto prior=Bytes(p.destination);
    EXPECT_EQ(tl::fea::AccumulateNodalTranslationalForces<8>(p.nodes,p.force,View(p)),
              Status::InvalidConnectivity);
    EXPECT_EQ(Bytes(p.destination),prior);
    run(p);ASSERT_EQ(p.status,Status::Success);
    for(unsigned c=0;c<3;++c) {
      EXPECT_DOUBLE_EQ(p.destination[c][4],1);
      EXPECT_DOUBLE_EQ(p.destination[c][5],1);
    }
    auto expected=Initial();
    for(unsigned c=0;c<3;++c) expected.destination[c][4]=expected.destination[c][5]=1;
    EXPECT_EQ(Bytes(p.destination),Bytes(expected.destination));
  }
}
template<class Run> inline void StiffnessOrderAndRollback(Run run) {
  auto p=Initial();p.scalar=true;run(p);ASSERT_EQ(p.status,Status::Success);
  // Both rounded additions lose one at 2^53. Combining two slots first would
  // produce2^53+2; overwriting the second slot loses the increment at node5.
  EXPECT_DOUBLE_EQ(p.stiffness[4],0x1p53);
  EXPECT_DOUBLE_EQ(p.stiffness[5],18);
  EXPECT_NE(p.stiffness[4],0x1p53+(1.+1.));
  for(unsigned fault=0;fault<5;++fault) {
    p=Initial();p.scalar=true;
    if(fault==0)p.nodes[7]=9;
    if(fault==1)p.increments[7]=-1;
    if(fault==2)p.increments[7]=0;
    if(fault==3)p.increments[7]=std::numeric_limits<double>::quiet_NaN();
    if(fault==4) {p.stiffness[5]=std::numeric_limits<double>::max();
      p.increments[7]=std::numeric_limits<double>::max();}
    const auto before=Bytes(p.stiffness);run(p);
    EXPECT_NE(p.status,Status::Success);EXPECT_EQ(Bytes(p.stiffness),before);
  }
  p=Initial();p.scalar=true;run(p);EXPECT_EQ(p.status,Status::Success);
}
template<class Run> inline void ForceFailureAndRetry(Run run) {
  for(unsigned fault=0;fault<4;++fault) {
    auto p=Initial();
    if(fault==0)p.nodes[7]=9;
    if(fault==1)p.force[7].z=std::numeric_limits<double>::quiet_NaN();
    if(fault==2) {p.destination[2][5]=std::numeric_limits<double>::max();
      p.force[6].z=0;p.force[7].z=std::numeric_limits<double>::max();}
    if(fault==3)p.sign=0;
    const auto before=Bytes(p.destination);run(p);
    EXPECT_NE(p.status,Status::Success);EXPECT_EQ(Bytes(p.destination),before);
  }
  auto p=Initial();run(p);EXPECT_EQ(p.status,Status::Success);
  EXPECT_DOUBLE_EQ(p.destination[0][5],1);
}
} // namespace slot_test
