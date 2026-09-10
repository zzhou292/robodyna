#include "../surface_contact/NodalWallCapacityFixture.h"
#include "lib_src/collision/NodalWallContactIncidence.h"
#include "lib_src/collision/NodalWallContactResultIO.h"
namespace vehicle_wall_device_test {
using namespace nodal_wall_capacity_test;
TEST(VehicleWallLayout, ExactCompleteArenasProfilesAndOverflowRemainBounded) {
  detail::ArenaLayout layout;
  const auto vehicle=sc::NodalWallDeviceProfile::Vehicle;const auto cap=sc::MaxVehicleNodalWallDeviceBytes;
  ASSERT_TRUE(detail::BuildArenaLayout(349645,359785,359785,cap,layout,vehicle));
  EXPECT_EQ(layout.bytes,1078530400u);EXPECT_EQ(detail::HostPreparationBytes(layout),1080493924u);
  EXPECT_EQ(layout.incident_offsets.count,359786u);EXPECT_EQ(layout.incident_slots.count,4*349645u);
  ASSERT_TRUE(detail::BuildArenaLayout(524288,524288,524288,cap,layout,vehicle));
  EXPECT_EQ(layout.bytes,1602831752u);EXPECT_LT(detail::HostPreparationBytes(layout),sc::MaxVehicleNodalWallHostBytes);
  const auto held=nodal_wall_owner_test::Bytes(layout);
  for(unsigned fault=0;fault<7;++fault) {
    const auto p=fault==0?0:fault==1?524289:fault==2?SIZE_MAX:524288;
    const auto n=fault==3?524289:524288;
    const auto budget=fault==4?layout.bytes-1:fault==5?cap+1:cap;
    EXPECT_FALSE(detail::BuildArenaLayout(p,n,n,budget,layout,fault==6?sc::NodalWallDeviceProfile::Legacy:vehicle));
    EXPECT_EQ(nodal_wall_owner_test::Bytes(layout),held);
  }
  EXPECT_FALSE(detail::BuildArenaLayout(1025,2048,2048,sc::MaxActiveNodalWallDeviceBytes,layout));
  EXPECT_FALSE(detail::BuildArenaLayout(1024,2049,2049,sc::MaxActiveNodalWallDeviceBytes,layout));
  ASSERT_TRUE(detail::BuildArenaLayout(1024,2048,2048,sc::MaxActiveNodalWallDeviceBytes,layout));
}
TEST(VehicleWallLayout, ActiveResultMetadataRequiresExplicitProfileAndExactDisjointExtents) {
  sc::NodalWallDiagnostics expected;
  sc::NodalWallDeviceResultView v{reinterpret_cast<sc::NodalWallDiagnostics*>(UINT64_C(0x100000000)),
    reinterpret_cast<sc::NodalWallParentResult*>(UINT64_C(0x200000000)),
    reinterpret_cast<sc::NodalWallPointResult*>(UINT64_C(0x300000000)),
    reinterpret_cast<std::uint64_t*>(UINT64_C(0x400000000)),349645,359785};
  const auto vehicle=sc::NodalWallDeviceProfile::Vehicle;
  EXPECT_FALSE(detail::ValidResultView(v,349645,359785,&expected));
  EXPECT_TRUE(detail::ValidResultView(v,349645,359785,&expected,vehicle));
  auto bad=v;--bad.node_capacity;bad.nodes=reinterpret_cast<sc::NodalWallPointResult*>(1);
  EXPECT_FALSE(detail::ValidResultView(bad,349645,359785,&expected,vehicle));
  bad=v;bad.parents=reinterpret_cast<sc::NodalWallParentResult*>(bad.nodes);
  EXPECT_FALSE(detail::ValidResultView(bad,349645,359785,&expected,vehicle));
  bad=v;bad.diagnostics=&expected;
  EXPECT_FALSE(detail::ValidResultView(bad,349645,359785,&expected,vehicle));
}
} // namespace vehicle_wall_device_test
