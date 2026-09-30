#include "FailureResidentSource.h"
#include "lib_src/elements/failure/ShellFailureStorage.h"
#include "lib_src/elements/failure/ShellFailureValues.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/t3/T3BatchStorage.h"
#include "lib_src/elements/ShellMixedSectionStorage.h"
#include <limits>
namespace resident_failure_test {
static_assert(!std::is_copy_constructible_v<storage::FailureHostStorage>);
static_assert(!std::is_copy_assignable_v<storage::FailureHostStorage>);
TEST(ResidentFailureLayout,ExplicitSidecarExactBudgetsAndRebasedSlabsPreserveLegacyLayout) {
  storage::MixedLayout old;ASSERT_TRUE(old.Initialize(7,6,1<<20));const auto old_bytes=old.bytes;
  fe::ShellBatchFailureLimits limits;storage::FailureLayout layout;std::size_t host=0;
  ASSERT_TRUE(storage::FailureHostStorage::Forecast(7,4096,1<<20,limits,layout,host));
  limits.max_host_bytes=host;limits.max_device_bytes=layout.bytes;
  storage::FailureLayout exact;std::size_t exact_host=0;
  ASSERT_TRUE(storage::FailureHostStorage::Forecast(7,4096,layout.bytes,limits,exact,exact_host));
  EXPECT_EQ(exact.bytes,layout.bytes);EXPECT_EQ(exact_host,host);
  auto before=plasticity_binding_test::Bytes(exact);
  for(bool device:{false,true}) {
    auto small=limits;if(device)--small.max_device_bytes;else --small.max_host_bytes;
    EXPECT_FALSE(storage::FailureHostStorage::Forecast(7,4096,layout.bytes,small,exact,exact_host));
    EXPECT_EQ(plasticity_binding_test::Bytes(exact),before);EXPECT_EQ(exact_host,host);
  }
  EXPECT_FALSE(exact.Initialize(std::numeric_limits<std::size_t>::max(),1<<20));
  tl::util::HostArena arena;ASSERT_TRUE(arena.Initialize(layout.bytes));auto* value=layout.Construct(arena);ASSERT_NE(value,nullptr);
  auto* fake=reinterpret_cast<void*>(std::uintptr_t(0x100000));auto rebased=layout.Rebase(fake);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(rebased.state[1]),0x100000+layout.state[1].offset);
  EXPECT_TRUE(value->state[1][6].active);EXPECT_EQ(value->state[1][6].policy(),fe::ShellFailurePolicy::None);
  storage::MixedLayout retained;ASSERT_TRUE(retained.Initialize(7,6,1<<20));EXPECT_EQ(retained.bytes,old_bytes);
}
TEST(ResidentFailureLayout,FullOriginalCountsFitCombinedExistingFamilyDeviceBudgets) {
  constexpr auto cap=fe::MaxVehicleShellResidentDeviceBytes;
  fe::qeph::batch_detail::Layout q;fe::t3::batch_detail::Layout t;storage::MixedLayout qm,tm;storage::FailureLayout qf,tf;
  ASSERT_TRUE(q.Initialize(328344,359785,cap));ASSERT_TRUE(t.Initialize(21301,359785,cap));
  ASSERT_TRUE(qm.Initialize(328344,1024,cap-q.bytes));ASSERT_TRUE(tm.Initialize(21301,1024,cap-t.bytes));
  ASSERT_TRUE(qf.Initialize(328344,cap-q.bytes-qm.bytes));ASSERT_TRUE(tf.Initialize(21301,cap-t.bytes-tm.bytes));
  EXPECT_EQ(qf.state[1].count,328344u);EXPECT_EQ(tf.state[1].count,21301u);
  RecordProperty("qeph_total_device_bytes",std::to_string(q.bytes+qm.bytes+qf.bytes));
  RecordProperty("t3_total_device_bytes",std::to_string(t.bytes+tm.bytes+tf.bytes));
  RecordProperty("failure_total_device_bytes",std::to_string(qf.bytes+tf.bytes));
}
TEST(ResidentFailureLayout,ActivityAndSavedVersusCurrentStressRejectLateCorruption) {
  fe::ShellBatchSectionState section;auto state=fe::ShellBatchFailureState::Constant();
  EXPECT_TRUE(storage::ValidFailureState(state,state.policy(),&section,0));
  state.constant_points()[2]={1,.25,false};state.current_force_point[2].stress[4]=123;
  EXPECT_TRUE(storage::ValidFailureState(state,state.policy(),&section,.25));
  EXPECT_FALSE(storage::ValidFailureState(state,state.policy(),&section,.125));
  state.active=false;EXPECT_FALSE(storage::ValidFailureState(state,state.policy(),&section,.25));state.active=true;
  state.current_force_point[2].stress[4]=std::numeric_limits<double>::infinity();
  EXPECT_FALSE(storage::ValidFailureState(state,state.policy(),&section,.25));
  state=fe::ShellBatchFailureState::Constant();*reinterpret_cast<unsigned char*>(&state.constant_points()[2].point_active)=2;
  EXPECT_FALSE(storage::ValidFailureEncoding(state));
}
} // namespace resident_failure_test
