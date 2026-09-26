#pragma once
#include "lib_src/solvers/NodalUniformMotionObserver.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <gtest/gtest.h>
#include <vector>
namespace motion_observer_test {
namespace fe = tl::fea;
using Code = fe::NodalStatus;
struct Snapshot {
  std::size_t count;
  std::vector<double> values;
  explicit Snapshot(std::size_t n): count(n), values(19*n) {}
  fe::NodalSnapshotBuffer Buffer() {
    return {values.data(), values.data()+3*count, count, values.data()+9*count,
        values.data()+6*count, values.data()+13*count, values.data()+16*count};
  }
};
// Independent host implementation of the previous component scan. No GPU
// production reduction/value helper is included or called here.
inline fe::NodalUniformMotionSummary Oracle(const double* initial, const Snapshot& x,
    tl::math::Vec3 velocity, double time, bool rotations=true) {
  fe::NodalUniformMotionSummary result;result.nodes=x.count;
  const double speed[]{velocity.x,velocity.y,velocity.z};
  const auto include=[](double actual,double expected,double& maximum) {
    if(!std::isfinite(actual)||!std::isfinite(expected))throw std::runtime_error("nonfinite field");
    const auto difference=std::abs(actual-expected);
    if(!std::isfinite(difference))throw std::runtime_error("difference overflow");
    maximum=std::max(maximum,difference);
  };
  for(std::size_t i=0;i<x.count;++i) {
    for(unsigned a=0;a<3;++a) {
      const double expected=speed[a]==0?initial[3*i+a]:initial[3*i+a]+speed[a]*time;
      include(x.values[3*i+a],expected,result.maximum_position_error);
      include(x.values[3*x.count+3*i+a],speed[a],result.maximum_velocity_error);
      if(rotations)include(x.values[6*x.count+3*i+a],0,result.maximum_spin);
    }
    if(rotations)for(unsigned a=0;a<4;++a)
      include(x.values[9*x.count+4*i+a],a==0?1:0,result.maximum_orientation_error);
  }
  return result;
}
inline void Same(const fe::NodalUniformMotionSummary& a,const fe::NodalUniformMotionSummary& b) {
  EXPECT_EQ(a.nodes,b.nodes);
  const double x[]{a.maximum_position_error,a.maximum_velocity_error,a.maximum_orientation_error,a.maximum_spin};
  const double y[]{b.maximum_position_error,b.maximum_velocity_error,b.maximum_orientation_error,b.maximum_spin};
  EXPECT_EQ(std::memcmp(x,y,sizeof(x)),0);
}
inline fe::NodalUniformMotionObservation Sentinel() {
  fe::NodalUniformMotionObservation out;
  out.prepared.owner_id=71;out.prepared.attempt=72;out.prepared.proposed_time=-1;
  out.motion={999,11,12,13,14};return out;
}
inline void SameOutput(const fe::NodalUniformMotionObservation& a,const fe::NodalUniformMotionObservation& b) {
  EXPECT_TRUE(fe::trial_identity::SamePrepared(a.prepared,b.prepared));Same(a.motion,b.motion);
}
class Cuda:public testing::Test {
  void SetUp() override {int count=0;ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0);}
};
}
