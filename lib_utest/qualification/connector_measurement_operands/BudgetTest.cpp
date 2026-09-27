// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "FrozenType13Arena.h"
#include "FrozenType25Arena.h"
namespace connector_operand_test {
TEST(ConnectorOperandBudget, Type13PacketRegionAndExactDeviceCapRejectWithoutMutatingOutput) {
  namespace d=a::batch_detail; namespace prior=d::baseline_layout;
  for(std::size_t count : {1u,2u,17u,257u,4442u}) {
    a::BatchLimits cap; d::ArenaLayout current; prior::ArenaLayout old;
    ASSERT_TRUE(d::MakeLayout(1,count,cap,current));
    ASSERT_TRUE(prior::MakeLayout(1,count,cap,old));
    EXPECT_EQ(current.measurement.bytes,144*count);
    EXPECT_EQ(current.measurement.offset%alignof(d::Measurement),0u);
    EXPECT_GE(current.bytes-old.bytes,144*count+sizeof(d::Storage)-sizeof(prior::Storage));
    cap.max_device_bytes=current.bytes;
    d::ArenaLayout exact;
    EXPECT_TRUE(d::MakeLayout(1,count,cap,exact)); EXPECT_EQ(exact.bytes,current.bytes);
    --cap.max_device_bytes; exact.bytes=71;
    EXPECT_FALSE(d::MakeLayout(1,count,cap,exact)); EXPECT_EQ(exact.bytes,71u);
    tl::util::HostArena arena; ASSERT_TRUE(arena.Initialize(current.bytes));
    const auto header=d::RebasedHeader(arena.data(),current);
    EXPECT_EQ(header.measurement,tl::util::ArenaPointer<d::Measurement>(arena.data(),current.measurement));
  }
}
TEST(ConnectorOperandBudget, Type25BothCapacityProfilesChargeEveryPacketAndRejectOneShort) {
  namespace d=b::batch_detail; namespace prior=d::baseline_layout;
  for(auto profile : {b::CapacityProfile::Legacy,b::CapacityProfile::Vehicle}) {
    for(std::size_t count : {1u,2u,17u,257u}) {
      d::ArenaLayout current; prior::ArenaLayout old;
      const auto cap=b::Bounds(profile).batch_device_bytes;
      ASSERT_TRUE(d::MakeLayout(1,count,4,cap,current,profile));
      ASSERT_TRUE(prior::MakeLayout(1,count,4,cap,old,profile));
      EXPECT_EQ(current.measurement.bytes,112*count);
      EXPECT_EQ(current.measurement.offset%alignof(d::Measurement),0u);
      EXPECT_GE(current.bytes-old.bytes,112*count+sizeof(d::Storage)-sizeof(prior::Storage));
      d::ArenaLayout exact;
      EXPECT_TRUE(d::MakeLayout(1,count,4,current.bytes,exact,profile));
      EXPECT_EQ(exact.bytes,current.bytes);
      exact.bytes=83;
      EXPECT_FALSE(d::MakeLayout(1,count,4,current.bytes-1,exact,profile)); EXPECT_EQ(exact.bytes,83u);
      tl::util::HostArena arena; ASSERT_TRUE(arena.Initialize(current.bytes));
      const auto header=d::RebasedHeader(arena.data(),current);
      EXPECT_EQ(header.measurement,tl::util::ArenaPointer<d::Measurement>(arena.data(),current.measurement));
    }
  }
}
} // namespace connector_operand_test
