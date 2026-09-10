#include "StorageExpectations.h"
#include <limits>
namespace active_shell_test {
TEST(ActiveArena, AlignmentOverflowAndFailedAppendAreAtomic) {
  tl::util::BoundedArenaLayout layout(64); tl::util::ArenaRegion first,second;
  ASSERT_TRUE(layout.Append<unsigned char>(3,first)); ASSERT_TRUE(layout.Append<double>(3,second));
  EXPECT_EQ(first.offset,0u); EXPECT_EQ(second.offset%alignof(double),0u);
  EXPECT_GE(second.offset,first.offset+first.bytes);
  const auto bytes=layout.bytes(); const auto held=second;
  EXPECT_FALSE(layout.Append<double>(std::numeric_limits<std::size_t>::max(),second));
  EXPECT_EQ(layout.bytes(),bytes); EXPECT_EQ(second.offset,held.offset); EXPECT_EQ(second.bytes,held.bytes);
  tl::util::HostArena host; ASSERT_TRUE(host.Initialize(bytes));
  auto* values=host.Construct<double>(second); ASSERT_NE(values,nullptr);
  for(unsigned i=0;i<3;++i) EXPECT_EQ(values[i],0);
  auto bad=second; ++bad.offset; EXPECT_EQ(host.Construct<double>(bad),nullptr);
  EXPECT_FALSE(host.Initialize(bytes));
}
TEST(ActiveArena, FamilyGrowthUsesActiveRecordsAndHostRebasedHeaders) {
  using Layout=fe::qeph::batch_detail::Layout;
  Layout one,large;
  ASSERT_TRUE(one.Initialize(1,4,fe::MaxShellResidentDeviceBytes));
  ASSERT_TRUE(large.Initialize(804,1030,fe::MaxShellResidentDeviceBytes));
  EXPECT_EQ(large.bytes-one.bytes,
    803*(sizeof(fe::qeph::QephBatchElement)+2*sizeof(fe::qeph::ForceTrial)+sizeof(fe::qeph::Status))+
    1026*(sizeof(tl::math::Vec3)+4*sizeof(double)));
  tl::util::HostArena host,other;
  ASSERT_TRUE(host.Initialize(large.bytes)); ASSERT_TRUE(other.Initialize(large.bytes));
  auto* h=large.Construct(host); ASSERT_NE(h,nullptr);
  const auto rebased=large.Rebase(*h,other.data());
  EXPECT_EQ(rebased.model.element,tl::util::ArenaPointer<fe::qeph::QephBatchElement>(other.data(),large.element));
  EXPECT_EQ(rebased.slab[1].element,tl::util::ArenaPointer<fe::qeph::ForceTrial>(other.data(),large.slab[1]));
  EXPECT_NE(rebased.model.element,h->model.element);
  const auto held=large.bytes;
  EXPECT_FALSE(large.Initialize(std::numeric_limits<std::size_t>::max(),1030,fe::MaxShellResidentDeviceBytes));
  EXPECT_EQ(large.bytes,held);
}
TEST(ActiveArena, FullHardBoundsFitDeclaredDeviceCapsAndRequireOptIn) {
  EXPECT_LT(QBytes(1024,2048,1024),fe::MaxShellResidentDeviceBytes);
  EXPECT_LT(TBytes(1024,2048,1024),fe::MaxShellResidentDeviceBytes);
  EXPECT_LT(PublicationBytes(2048),128*1024u);
  fe::ShellResidentLimits limits;
  EXPECT_FALSE(fe::ValidShellResidentLimits(limits,804,1030,4*1024*1024));
  limits.max_parents=1024; limits.max_nodes=2048;
  EXPECT_TRUE(fe::ValidShellResidentLimits(limits,804,1030,4*1024*1024));
  EXPECT_FALSE(fe::ValidShellResidentLimits(limits,1025,1030,4*1024*1024));
  EXPECT_FALSE(fe::ValidShellResidentLimits(limits,804,2049,4*1024*1024));
  EXPECT_FALSE(fe::ValidShellResidentLimits(limits,804,1030,fe::MaxShellResidentDeviceBytes+1));
}
}
