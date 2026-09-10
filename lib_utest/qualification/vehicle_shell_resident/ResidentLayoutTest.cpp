#include "../active_shell_collection/StorageExpectations.h"
#include "../vehicle_plasticity_catalog/VehicleCatalogFixture.h"
#include "lib_src/elements/ShellResidentHostAccounting.h"
#include "lib_src/elements/ShellResidentStartupIndex.h"
namespace vehicle_resident_test {
namespace fe=tl::fea;namespace sd=fe::shell_batch_detail;
TEST(VehicleResidentLayout, SourceCountsAndHardMaximumUseExactExistingNativeArrays) {
  fe::qeph::batch_detail::Layout q;fe::t3::batch_detail::Layout t;
  fe::shell_batch_plasticity_detail::Layout qp,tp;
  const auto cap=fe::MaxVehicleShellResidentDeviceBytes;
  ASSERT_TRUE(q.Initialize(328344,359785,cap));ASSERT_TRUE(t.Initialize(21301,359785,cap));
  ASSERT_TRUE(qp.Initialize(328344,1024,cap));ASSERT_TRUE(tp.Initialize(21301,1024,cap));
  EXPECT_EQ(q.bytes,1132577992u);EXPECT_EQ(t.bytes,72889780u);
  EXPECT_EQ(qp.bytes,210156584u);EXPECT_EQ(tp.bytes,13649064u);
  EXPECT_EQ(q.bytes+t.bytes+qp.bytes+tp.bytes,1429273420u);
  ASSERT_TRUE(q.Initialize(524288,524288,cap));ASSERT_TRUE(qp.Initialize(524288,1024,cap));
  EXPECT_EQ(q.bytes+qp.bytes,2141209176u);EXPECT_LT(q.bytes+qp.bytes,cap);
  const auto held=q.bytes;EXPECT_FALSE(q.Initialize(524288,524288,held-1));EXPECT_EQ(q.bytes,held);
  EXPECT_FALSE(q.Initialize(SIZE_MAX,524288,cap));EXPECT_EQ(q.bytes,held);
  EXPECT_FALSE(qp.Initialize(524289,1024,cap));
  fe::shell_publication_detail::Layout p;
  ASSERT_TRUE(p.Initialize(359785,fe::ShellPublicationLimits::Vehicle().max_device_bytes));
  EXPECT_EQ(p.bytes,11513280u);
  ASSERT_TRUE(p.Initialize(524288,fe::ShellPublicationLimits::Vehicle().max_device_bytes,true));
  EXPECT_EQ(p.bytes,25165984u);
}
TEST(VehicleResidentLayout, DefaultCountAndByteCeilingsStayClosedWhileVehicleIsExplicit) {
  fe::ShellResidentLimits legacy;
  EXPECT_FALSE(fe::ValidShellResidentLimits(legacy,129,128,1024*1024));
  EXPECT_FALSE(fe::ValidShellResidentLimits(legacy,128,129,1024*1024));
  legacy.max_parents=1024;legacy.max_nodes=2048;
  EXPECT_TRUE(fe::ValidShellResidentLimits(legacy,1024,2048,fe::MaxShellResidentDeviceBytes));
  auto oversized=legacy;oversized.max_parents=1025;
  EXPECT_FALSE(fe::ValidShellResidentLimits(oversized,1,4,1024));
  oversized=legacy;oversized.max_nodes=2049;
  EXPECT_FALSE(fe::ValidShellResidentLimits(oversized,1,4,1024));
  EXPECT_FALSE(fe::ValidShellResidentLimits(legacy,1024,2048,fe::MaxShellResidentDeviceBytes+1));
  legacy.max_host_bytes=fe::MaxShellResidentHostBytes+1;
  EXPECT_FALSE(fe::ValidShellResidentLimits(legacy,1,4,1024));
  auto vehicle=fe::ShellResidentLimits::Vehicle();
  EXPECT_TRUE(fe::ValidShellResidentLimits(vehicle,328344,359785,fe::MaxVehicleShellResidentDeviceBytes));
  for(unsigned fault=0;fault<5;++fault){auto bad=vehicle;
    if(fault==0)bad.max_parents=524289;if(fault==1)bad.max_nodes=524289;
    if(fault==2)bad.max_host_bytes=0;if(fault==3)bad.max_host_bytes++;
    EXPECT_FALSE(fe::ValidShellResidentLimits(bad,328344,359785,
      fe::MaxVehicleShellResidentDeviceBytes+(fault==4)));}
}
TEST(VehicleResidentLayout, SharedBackingIsCountedOnceButEqualIndependentInventoriesAreNot) {
  vehicle_catalog_test::Fixture f(257,73,1029,2);fe::ShellBatchBinding b,independent;
  ASSERT_EQ(b.Initialize(f.geometry.input(),fe::ShellHostBindingLimits::Vehicle()).status,fe::ShellBindingStatus::Success);
  ASSERT_EQ(independent.Initialize(f.geometry.input(),fe::ShellHostBindingLimits::Vehicle()).status,fe::ShellBindingStatus::Success);
  fe::ShellBatchPlasticityBinding c;
  ASSERT_EQ(c.InitializeCatalog(b,f.input(),f.limits()).status,fe::ShellPlasticityBindingStatus::Success);
  ASSERT_EQ(b.inventory(),independent.inventory());ASSERT_NE(b.inventory().words().data(),independent.inventory().words().data());
  std::size_t bytes=17,cat=19;
  ASSERT_TRUE(sd::RetainedScopeBytes(&b,&c,true,bytes,cat));
  EXPECT_EQ(bytes,b.host_bytes()-sizeof b);EXPECT_EQ(cat,c.host_bytes()-b.inventory().backing_bytes());
  ASSERT_TRUE(sd::RetainedScopeBytes(&independent,&c,true,bytes,cat));EXPECT_EQ(cat,c.host_bytes());
  ASSERT_TRUE(sd::RetainedScopeBytes(&b,&c,false,bytes,cat));EXPECT_EQ(bytes,b.host_bytes());EXPECT_EQ(cat,c.host_bytes());
  fe::ShellBatchBinding empty;EXPECT_FALSE(sd::RetainedScopeBytes(&empty,&c,true,bytes,cat));
  EXPECT_EQ(bytes,b.host_bytes());EXPECT_EQ(cat,c.host_bytes());
}
TEST(VehicleResidentLayout, TriangleIndexKeepsFirstSourceOccurrenceAndMalformedSubsetPrecedence) {
  using Key=sd::ResidentTriangleIndex::Key;
  const std::array<Key,5> nodes{{{10,20,30},{7,8,9},{30,10,20},{10,10,20},{70,70,80}}};
  sd::ResidentTriangleIndex index;index.Prepare(nodes.size(),[&](auto e){return nodes[e];});
  for(unsigned e=0;e<nodes.size();++e)
    EXPECT_EQ(index.DuplicateBefore(e,nodes[e],[&](auto i){return nodes[i];}),e==2||e==3);
  sd::ResidentNodeIdentityIndex ids;const std::uint64_t values[]{99,7,99,7};
  ids.Prepare(4,[&](auto i){return values[i];});EXPECT_EQ(ids.First(99),0u);EXPECT_EQ(ids.First(7),1u);
}
} // namespace vehicle_resident_test
