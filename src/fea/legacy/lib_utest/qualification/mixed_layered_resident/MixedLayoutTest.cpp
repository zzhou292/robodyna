#include "lib_src/elements/ShellMixedSectionStorage.h"
#include "lib_src/elements/ShellLayeredSectionValues.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/t3/T3BatchStorage.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace {
namespace fe=tl::fea;
namespace storage=fe::shell_batch_plasticity_detail;
template<class T> auto Bytes(const T& x) {
  std::array<unsigned char,sizeof(T)> bytes{};std::memcpy(bytes.data(),&x,sizeof x);return bytes;
}
TEST(MixedLayeredLayout, SourcePopulationFitsDirectIndexingButHardParentCeilingDoesNotPromiseBytes) {
  constexpr std::size_t cap=fe::MaxVehicleShellResidentDeviceBytes,nodes=359785;
  fe::qeph::batch_detail::Layout q;fe::t3::batch_detail::Layout t;
  ASSERT_TRUE(q.Initialize(328344,nodes,cap));ASSERT_TRUE(t.Initialize(21301,nodes,cap));
  storage::MixedLayout qs,ts;
  ASSERT_TRUE(qs.Initialize(328344,1024,cap-q.bytes));ASSERT_TRUE(ts.Initialize(21301,1024,cap-t.bytes));
  EXPECT_LT(q.bytes+qs.bytes,cap);EXPECT_LT(t.bytes+ts.bytes,cap);
  EXPECT_EQ(qs.law.count,328344u);EXPECT_EQ(qs.plastic_section[1].count,328344u);
  EXPECT_EQ(qs.elastic_section[1].count,328344u);
  storage::MixedLayout held;held.bytes=77;const auto before=Bytes(held);
  ASSERT_TRUE(q.Initialize(524288,524288,cap));
  EXPECT_FALSE(held.Initialize(524288,1024,cap-q.bytes));EXPECT_EQ(Bytes(held),before);
  EXPECT_FALSE(held.Initialize(std::numeric_limits<std::size_t>::max(),0,cap));EXPECT_EQ(Bytes(held),before);
  EXPECT_FALSE(held.Initialize(3,1025,cap));EXPECT_EQ(Bytes(held),before);
}
TEST(MixedLayeredLayout, ExactHostDeviceBudgetsRebaseDistinctTypedArraysAndPreserveFailureOutputs) {
  storage::MixedLayout layout;std::size_t host=0;
  ASSERT_TRUE(storage::MixedHostStorage::Forecast(7,6,4096,1<<20,1<<20,layout,host));
  storage::MixedLayout exact;std::size_t exact_host=0;
  ASSERT_TRUE(storage::MixedHostStorage::Forecast(7,6,4096,layout.bytes,host,exact,exact_host));
  EXPECT_EQ(exact_host,host);EXPECT_EQ(exact.bytes,layout.bytes);
  for(bool device:{false,true}) {
    const auto before=Bytes(exact);const auto old_host=exact_host;
    EXPECT_FALSE(storage::MixedHostStorage::Forecast(7,6,4096,layout.bytes-(device?1:0),host-(device?0:1),exact,exact_host));
    EXPECT_EQ(Bytes(exact),before);EXPECT_EQ(exact_host,old_host);
  }
  tl::util::HostArena arena;ASSERT_TRUE(arena.Initialize(layout.bytes));auto* initial=layout.Construct(arena);ASSERT_NE(initial,nullptr);
  for(unsigned n=0;n<7;++n)initial->law[n]=n%2?fe::ShellSectionLaw::LayeredLaw1Nip3:fe::ShellSectionLaw::LayeredLaw44Nip3;
  auto* fake=reinterpret_cast<void*>(std::uintptr_t(0x100000));const auto rebased=layout.Rebase(*initial,fake);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(rebased.elastic_section[1]),0x100000u+layout.elastic_section[1].offset);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(rebased.plastic.section[1]),0x100000u+layout.plastic_section[1].offset);
  EXPECT_NE(reinterpret_cast<void*>(rebased.elastic_section[1]),reinterpret_cast<void*>(rebased.plastic.section[1]));
  EXPECT_EQ(initial->law[6],fe::ShellSectionLaw::LayeredLaw44Nip3);
}
TEST(MixedLayeredLayout, ReadbackAvailabilityFollowsActiveLawAndChecksLateValues) {
  fe::ShellBatchLayeredSection none;EXPECT_EQ(none.plastic(),nullptr);EXPECT_EQ(none.elastic(),nullptr);
  fe::ShellBatchSectionState p;p.history.point[2].plastic_strain=.125;p.cumulative_plastic_work_J=17;
  fe::sections::ShellLayeredLaw1History e;e.point[2].stress[4]=123;
  auto plastic=fe::ShellBatchLayeredSection::Plastic(p),elastic=fe::ShellBatchLayeredSection::Elastic(e);
  EXPECT_EQ(plastic.elastic(),nullptr);ASSERT_NE(plastic.plastic(),nullptr);EXPECT_EQ(plastic.plastic()->history.point[2].plastic_strain,.125);
  EXPECT_EQ(elastic.plastic(),nullptr);ASSERT_NE(elastic.elastic(),nullptr);EXPECT_EQ(elastic.elastic()->point[2].stress[4],123);
  auto copy=elastic;copy=plastic;EXPECT_EQ(copy.elastic(),nullptr);EXPECT_EQ(copy.plastic()->cumulative_plastic_work_J,17);
  EXPECT_TRUE(storage::FiniteSection(p));EXPECT_TRUE(storage::FiniteSection(e));
  e.point[2].stress[4]=std::numeric_limits<double>::infinity();EXPECT_FALSE(storage::FiniteSection(e));
  p.history.point[2].filtered_rate_per_s=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(storage::FiniteSection(p));
}
} // namespace
