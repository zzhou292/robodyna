#include "lib_src/solvers/NodalStateLayout.h"
#include <gtest/gtest.h>
#include <limits>

namespace {
namespace fe=tl::fea;
using fe::nodal_detail::StateLayout;
TEST(NodalVehicleLayout, ExactSeparateAllocationBudgetIncludesMasksWithoutInventedPadding) {
  for(auto n:{std::size_t(1),std::size_t(1030),fe::MaxNodalStateNodes,std::size_t(393165),fe::MaxActiveNodalStateNodes})
    for(bool rotations:{false,true}) {
      StateLayout layout;
      constexpr std::size_t control=137; // Deliberately odd, separate allocation.
      const auto expected=(rotations?411:193)*n+control;
      ASSERT_TRUE(layout.Initialize(n,rotations,0,0,0,control,expected));
      EXPECT_EQ(layout.bytes,expected); EXPECT_EQ(layout.accepted.count,(rotations?19:6)*n);
      EXPECT_EQ(layout.fixed.bytes,(rotations?3:1)*n);
      StateLayout rejected; EXPECT_FALSE(rejected.Initialize(n,rotations,0,0,0,control,expected-1));
      EXPECT_EQ(rejected.bytes,0u);
    }
}
TEST(NodalVehicleLayout, GroupTailAndOptionalCaptureUseOneOwnerBudget) {
  const auto n=fe::MaxActiveNodalStateNodes;
  constexpr std::size_t groups=64,members=64*256,control=137;
  const auto immutable=40*groups+24*members+n;
  const auto expected=411*n+288*groups+immutable+48*(n+groups)+control;
  StateLayout layout;
  ASSERT_TRUE(layout.Initialize(n,true,18*groups,immutable,6*(n+groups),control,expected));
  EXPECT_EQ(layout.bytes,expected); EXPECT_LT(expected,fe::MaxActiveNodalStateDeviceBytes);
  EXPECT_EQ(layout.accepted.count,19*n+18*groups);
  EXPECT_EQ(layout.scratch.count,11*n+6*(n+groups));
}
TEST(NodalVehicleLayout, InvalidCountsAndOverflowPreserveCompletePriorLayout) {
  StateLayout initial; ASSERT_TRUE(initial.Initialize(3,true,0,0,0,137,4096));
  const auto huge=std::numeric_limits<std::size_t>::max();
  const auto reject=[&](std::size_t n,bool rotation,std::size_t groups,std::size_t rigid,
                       std::size_t capture,std::size_t control,std::size_t cap) {
    auto out=initial;
    EXPECT_FALSE(out.Initialize(n,rotation,groups,rigid,capture,control,cap));
    EXPECT_EQ(out.bytes,initial.bytes);
    const tl::util::ArenaRegion a[]{out.accepted,out.trial,out.scratch,out.inverse,out.fixed,out.control,out.rigid};
    const tl::util::ArenaRegion b[]{initial.accepted,initial.trial,initial.scratch,initial.inverse,initial.fixed,initial.control,initial.rigid};
    for(unsigned i=0;i<7;++i) {
      EXPECT_EQ(a[i].offset,b[i].offset); EXPECT_EQ(a[i].count,b[i].count); EXPECT_EQ(a[i].bytes,b[i].bytes);
    }
  };
  reject(0,true,0,0,0,137,4096);
  reject(fe::MaxActiveNodalStateNodes+1,true,0,0,0,137,4096);
  reject(huge,true,0,0,0,137,4096);
  reject(3,true,huge,0,0,137,4096);
  reject(3,true,0,huge,0,137,4096);
  reject(3,true,0,0,huge,137,4096);
  reject(3,true,0,0,0,huge,4096);
  reject(3,true,0,0,0,137,fe::MaxActiveNodalStateDeviceBytes+1);
  reject(3,false,18,0,0,137,4096);
  reject(3,false,0,1,0,137,4096);
  reject(3,false,0,0,1,137,4096);
}
} // namespace
