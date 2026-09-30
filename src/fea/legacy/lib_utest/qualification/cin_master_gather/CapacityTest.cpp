// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/solvers/NodalCinStorage.h"
#include "lib_src/solvers/NodalForceStageCaptureLayout.h"
#include "lib_src/solvers/nodal_seal/RowLayout.h"
namespace tl::fea::cin_gather_test {
namespace {
struct Layouts {
  nodal_detail::CinLayout cin;
  nodal_detail::StateLayout owner;
};
Layouts Required(std::size_t n,std::size_t r,std::size_t w=1,std::size_t groups=0,
    std::size_t capture_values=0) {
  Layouts result;
  const NodalCinLimits limits;
  EXPECT_TRUE(result.cin.Initialize(n,r,w,limits,sizeof(nodal_detail::CinStorage),groups));
  EXPECT_TRUE(result.owner.Initialize(n,true,rigid::GroupStateValues*groups,0,capture_values,
      nodal_seal::ControlBytes(sizeof(nodal_detail::Control),n),MaxActiveNodalStateDeviceBytes,
      result.cin.state_values,result.cin.device_bytes,true));
  return result;
}
bool Select(Layouts& value,NodalCinLimits limits={},std::size_t cap=MaxActiveNodalStateDeviceBytes) {
  return nodal_detail::SelectCinGatherLayout(value.cin,value.owner,limits,cap,0,sizeof(nodal_detail::Control));
}
}
TEST(CinMasterGatherCapacity, ExactFitAndOneByteBelowSelectGatherOrRequiredSerialMode) {
  const auto base=Required(37,3);
  auto full=base;ASSERT_TRUE(Select(full));
  const auto delta=full.owner.bytes-base.owner.bytes;
  EXPECT_EQ(delta,full.cin.device_bytes-base.cin.device_bytes);
  NodalCinLimits exact;
  exact.max_device_bytes=full.cin.optional_device_bytes;
  ASSERT_TRUE(nodal_detail::CinOwnerHostFits(full.cin.host_bytes,0,full.owner.accepted.count,
      full.owner.fixed.count,sizeof(nodal_detail::Control),exact.max_host_bytes,&exact.max_host_bytes));
  auto selected=base;ASSERT_TRUE(Select(selected,exact,full.owner.bytes));
  EXPECT_EQ(selected.owner.bytes,full.owner.bytes);
  for (unsigned limit=0;limit<3;++limit) {
    auto short_limits=exact;auto cap=full.owner.bytes;
    if (limit==0) --short_limits.max_device_bytes;
    if (limit==1) --short_limits.max_host_bytes;
    if (limit==2) --cap;
    auto fallback=base;
    EXPECT_FALSE(Select(fallback,short_limits,cap));
    EXPECT_EQ(fallback.owner.bytes,base.owner.bytes);
    EXPECT_EQ(fallback.cin.device_bytes,base.cin.device_bytes);
    EXPECT_EQ(fallback.cin.host_bytes,base.cin.host_bytes);
    EXPECT_EQ(fallback.cin.gather.device_bytes,0u);
  }
  auto tight=base;EXPECT_FALSE(Select(tight,{},base.owner.bytes));
  EXPECT_EQ(tight.owner.bytes,base.owner.bytes);
}
TEST(CinMasterGatherCapacity, V5AllocationIsCountedWithoutPopulationAndDisabledTailStaysEmpty) {
  auto value=Required(376930,11165,13173,779);
  const auto prior=value.cin.device_bytes;
  ASSERT_TRUE(Select(value));
  EXPECT_EQ(value.cin.gather.capacity,44660u);
  EXPECT_EQ(value.cin.device_bytes-prior,2679632u);
  EXPECT_EQ(value.cin.gather.host_bytes,535924u);
  EXPECT_EQ(value.cin.gather.temporary_bytes,1507724u);
  EXPECT_EQ(sizeof(gather::Master),48u);EXPECT_EQ(sizeof(gather::Summary),24u);
  gather::Layout missing;
  EXPECT_FALSE(missing.Initialize(0,0,1,128u<<20,128u<<20));
  EXPECT_FALSE(missing.Initialize(0,10,0,128u<<20,128u<<20));
  EXPECT_FALSE(missing.Initialize(0,10,UINT32_MAX,128u<<20,128u<<20));
  EXPECT_EQ(missing.device_bytes,0u);
  EXPECT_EQ(sizeof(nodal_detail::CinStorage),1000u);
  EXPECT_EQ(sizeof(nodal_detail::CinLayout),648u);
  EXPECT_EQ(sizeof(gather::View),64u);
  RecordProperty("cin_storage_bytes",std::to_string(sizeof(nodal_detail::CinStorage)));
  RecordProperty("cin_layout_bytes",std::to_string(sizeof(nodal_detail::CinLayout)));
  RecordProperty("private_launch_view_bytes",std::to_string(sizeof(gather::View)));
}
TEST(CinMasterGatherCapacity, MaximumBuilderAndV5OptionalCompositionStayWithinFixedCaps) {
  gather::Layout maximum;
  ASSERT_TRUE(maximum.Initialize(0,MaxActiveNodalStateNodes,65536,128u<<20,128u<<20));
  EXPECT_EQ(maximum.capacity,262144u);
  EXPECT_LE(maximum.device_bytes,128u<<20);
  EXPECT_LE(maximum.host_bytes+maximum.temporary_bytes,128u<<20);
  gather::Layout overflow;
  EXPECT_FALSE(overflow.Initialize(0,UINT32_MAX,65536,128u<<20,128u<<20));
  EXPECT_EQ(overflow.device_bytes,0u);

  constexpr std::size_t nodes=376930,rows=11165,witnesses=13173,groups=779;
  const nodal_detail::ForceStageCaptureLayout capture{nodes,groups};
  auto value=Required(nodes,rows,witnesses,groups,capture.values());
  const auto prior=value.cin.device_bytes;
  ASSERT_TRUE(Select(value));
  EXPECT_EQ(value.cin.device_bytes-prior,2679632u);
  EXPECT_EQ(value.cin.gather.host_bytes,535924u);
  EXPECT_EQ(value.cin.gather.temporary_bytes,1507724u);

  nodal_detail::CinLayout combined_ceiling;
  const NodalCinLimits limits;
  EXPECT_FALSE(combined_ceiling.Initialize(MaxActiveNodalStateNodes,65536,262144,
      limits,sizeof(nodal_detail::CinStorage),0));
  EXPECT_EQ(combined_ceiling.device_bytes,0u);
}
} // namespace tl::fea::cin_gather_test

namespace tl::fea::cin_gather_test {
TEST(CinMasterGatherReuse, ExplicitEmptyCinKeepsRequiredOwnerWithoutGatherAllocation) {
  Layouts empty;const NodalCinLimits limits;
  ASSERT_TRUE(empty.cin.Initialize(37,0,0,limits,sizeof(nodal_detail::CinStorage),0,true));
  ASSERT_TRUE(empty.owner.Initialize(37,true,0,0,0,nodal_seal::ControlBytes(sizeof(nodal_detail::Control),37),
      MaxActiveNodalStateDeviceBytes,empty.cin.state_values,empty.cin.device_bytes,true));
  const auto bytes=empty.owner.bytes,host=empty.cin.host_bytes;
  EXPECT_FALSE(Select(empty));EXPECT_EQ(empty.owner.bytes,bytes);EXPECT_EQ(empty.cin.host_bytes,host);
  EXPECT_EQ(empty.cin.gather.device_bytes,0u);EXPECT_EQ(empty.cin.gather.host_bytes,0u);
}
}
