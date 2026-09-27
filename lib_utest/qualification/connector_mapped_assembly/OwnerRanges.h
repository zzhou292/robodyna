// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/solvers/NodalCinRuntime.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <gtest/gtest.h>
namespace connector_owner_test {
inline void Distinct(const tl::fea::NodalAssemblyView& view,const tl::fea::NodalCinAssemblyView& cin) {
  ASSERT_EQ(view.forces.node_count,cin.node_count);ASSERT_GT(cin.node_count,1u);
  const double* arrays[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,
      view.forces.couple_x,view.forces.couple_y,view.forces.couple_z,
      cin.translational_stiffness,cin.rotational_stiffness};
  for(unsigned a=0;a<8;++a)for(unsigned b=0;b<a;++b)
    EXPECT_TRUE(tl::fea::trial_identity::Disjoint(arrays[a],cin.node_count*sizeof(double),
        arrays[b],cin.node_count*sizeof(double)))<<a<<":"<<b;
}
inline tl::fea::NodalAssemblyView Forge(tl::fea::NodalAssemblyView view,
    const tl::fea::NodalCinAssemblyView& cin,unsigned fault) {
  // Genuine owner storage but deliberately forged capability. No fabricated
  // allocation or device access is needed to exercise host authentication.
  if(fault==0)view.forces.force_x=cin.translational_stiffness;
  if(fault==1)view.forces.couple_z=cin.rotational_stiffness;
  if(fault==2)view.forces.force_y=view.forces.force_x+1;
  if(fault==3)view.forces.couple_x=view.forces.force_z+view.forces.node_count-1;
  return view;
}
} // namespace connector_owner_test
