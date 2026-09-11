// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/ShellNodalStiffness.h"
#include "lib_src/solvers/NodalForceAssembly.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>

extern "C" void qeph_q2_scatter(const double*,const double*,const int*,double*);
extern "C" void t3_r3_scatter(const int*,const double*,const double*,const double*,double*,double*,double*,double*);
namespace qt_mapped_test {
inline void Near(double a,double b) {
  ASSERT_TRUE(std::isfinite(a));
  ASSERT_TRUE(std::isfinite(b));
  EXPECT_NEAR(a,b,2e-9+2e-10*std::max(std::abs(a),std::abs(b)));
}
template<std::size_t N> void Scatter(const tl::math::Vec3 (&force)[N],
    const tl::math::Vec3 (&couple)[N],const tl::fea::shell_nodal_stiffness::Packet<N>& stiffness,
    const double* native_force,const double* native_couple,const double* native_coefficients) {
  const std::size_t nodes[N]{3,0,2};
  // Q4 uses all four slots; the T3 deliberately leaves global node 1 unused.
  std::size_t mapping[N];
  int connectivity[N];
  for (unsigned slot=0;slot<N;++slot) {
    mapping[slot]=slot==3?1:nodes[slot];
    connectivity[slot]=static_cast<int>(mapping[slot]+1);
  }
  std::array<double,32> native{};
  if constexpr(N==4) {
    std::array<double,24> internal;
    std::copy_n(native_force,12,internal.begin());
    std::copy_n(native_couple,12,internal.begin()+12);
    qeph_q2_scatter(internal.data(),native_coefficients,connectivity,native.data());
  } else {
    t3_r3_scatter(connectivity,native_force,native_couple,native_coefficients,
        native.data(),native.data()+12,native.data()+24,native.data()+28);
  }
  double fx[4]{},fy[4]{},fz[4]{},mx[4]{},my[4]{},mz[4]{},st[4]{},sr[4]{};
  tl::fea::DeviceNodalForceView view;
  view.node_count=4;
  view.force_x=fx; view.force_y=fy; view.force_z=fz;
  view.couple_x=mx; view.couple_y=my; view.couple_z=mz;
  ASSERT_EQ(tl::fea::AccumulateNodalForces<N>(mapping,force,couple,view,-1),
      tl::fea::NodalForceAssemblyStatus::Success);
  ASSERT_TRUE(tl::fea::shell_nodal_stiffness::Add(mapping,stiffness,st,sr,4));
  for (unsigned node=0;node<4;++node) {
    const double actual[]{fx[node],fy[node],fz[node],mx[node],my[node],mz[node],st[node],sr[node]};
    const double expected[]{native[3*node],native[3*node+1],native[3*node+2],
        native[12+3*node],native[13+3*node],native[14+3*node],native[24+node],native[28+node]};
    for (unsigned value=0;value<8;++value) Near(actual[value],expected[value]);
  }
}
} // namespace qt_mapped_test
