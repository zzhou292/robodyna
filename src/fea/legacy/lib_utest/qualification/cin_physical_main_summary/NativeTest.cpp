// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/solvers/cin_physical_mains/Values.h"
#include "../cin_physical_timestep/Fixture.h"
#include <gtest/gtest.h>
extern "C" void cin_native_rigid_terms(int,const double*,const double*,const double*,const double*,double*);
TEST(CinPhysicalMainsNative, BothPhysicalMainScalarSumsMatchIndependentNativeMemberTerms) {
  namespace main=tl::fea::cin_physical_mains;
  cin_step_test::Fixture f;
  for(unsigned group=0;group<2;++group) {
    const auto view=f.View();
    main::Values result;
    std::uint32_t bad=UINT32_MAX;
    ASSERT_TRUE(main::Reduce(view.rigid,group,view.accepted,view.translation,view.rotation,view.nodes,result,bad));
    const auto range=f.groups[group];
    double positions[6],kn[2],kr[2];
    for(unsigned i=0;i<2;++i) {
      const auto node=f.members[range.offset+i].node;
      std::copy_n(f.accepted.data()+3*node,3,positions+3*i);
      kn[i]=f.translation[node];kr[i]=f.rotation[node];
    }
    const double center[]{result.center.x,result.center.y,result.center.z};
    double native[4];
    cin_native_rigid_terms(2,positions,center,kn,kr,native);
    EXPECT_EQ(result.translation,native[0]+native[1]);
    EXPECT_EQ(result.rotation,native[2]+native[3]);
  }
}
