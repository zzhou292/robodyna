#include "Fixture.h"
namespace kinchk_test {
TEST(PostKinChkValues, CinPenaltyAndAllObservedFieldsRemainSeparateFromKinet) {
  Fixture f;
  f.slaves.push_back({77,1,{9,37,47,8,8}});
  tied::PostKinChkResult out;
  ASSERT_TRUE(tied::PostKinChk(f.Input(),&out));
  ASSERT_EQ(out.slaves().count,f.slaves.size());
  for (std::size_t i = 0; i < f.slaves.size(); ++i) {
    Same(out.slaves().data[i].before,f.slaves[i]);
    EXPECT_EQ(out.slaves().data[i].kinet,f.slaves[i].kinematics.conditions);
  }
  EXPECT_FALSE(out.slaves().data[0].repeated_condition);
  EXPECT_FALSE(out.slaves().data[1].mixed_incompatible_conditions);
  EXPECT_TRUE(out.slaves().data[2].repeated_condition);
  EXPECT_TRUE(out.slaves().data[2].mixed_incompatible_conditions);
  EXPECT_EQ(out.source_instance_id(),73u);
  EXPECT_EQ(out.source_interface_id(),991u);
}
TEST(PostKinChkValues, ExcludedRolesPhaseLateIdentityAndDecodeAreAtomic) {
  Fixture f;
  tied::PostKinChkResult out;
  ASSERT_TRUE(tied::PostKinChk(f.Input(),&out));
  const auto* prior = out.slaves().data;
  for (int bit : {4,2048,4096}) {
    f.slaves.back().kinematics.conditions = bit;
    EXPECT_EQ(tied::PostKinChk(f.Input(),&out).status,tied::ClassificationStatus::NativeDomain);
    EXPECT_EQ(out.slaves().data,prior);
  }
  f.slaves.back().kinematics.conditions = 8;
  auto input = f.Input();
  input.profile = tied::KinChkProfile::Unspecified;
  EXPECT_FALSE(tied::PostKinChk(input,&out));
  input = f.Input();
  input.phase = tied::ClassificationPhase::RigidMembersRegistered;
  EXPECT_FALSE(tied::PostKinChk(input,&out));
  f.slaves.back().source_id = f.slaves.front().source_id;
  EXPECT_EQ(tied::PostKinChk(f.Input(),&out).row,1u);
  f.slaves.back().source_id = 19;
  f.decode.back() = 2;
  EXPECT_EQ(tied::PostKinChk(f.Input(),&out).row,8191u);
  f.decode.back() = 1;
  EXPECT_EQ(out.slaves().data,prior);
  ASSERT_TRUE(tied::PostKinChk(f.Input(),&out));
  Same(out.slaves().data[1].before,f.slaves[1]);
}
TEST(PostKinChkValues, ExactBudgetCountsBeforeBorrowedReadsAndOldPayload) {
  Fixture f;
  tied::KinChkForecast bytes;
  ASSERT_TRUE(tied::ForecastPostKinChk(f.slaves.size(),0,&bytes));
  tied::KinChkLimits limits;
  limits.max_host_bytes = bytes.startup_payload_bytes-1;
  auto poisoned = f.Input();
  poisoned.slaves.data = reinterpret_cast<const tied::KinChkSlave*>(16);
  tied::PostKinChkResult out;
  EXPECT_EQ(tied::PostKinChk(poisoned,&out,limits).status,tied::ClassificationStatus::ResourceLimit);
  ++limits.max_host_bytes;
  ASSERT_TRUE(tied::PostKinChk(f.Input(),&out,limits));
  const auto* prior = out.slaves().data;
  EXPECT_EQ(tied::PostKinChk(f.Input(),&out,limits).status,tied::ClassificationStatus::ResourceLimit);
  EXPECT_EQ(out.slaves().data,prior);
  ASSERT_TRUE(tied::ForecastPostKinChk(f.slaves.size(),out.forecast().owned_payload_bytes,&bytes));
  limits.max_host_bytes = bytes.startup_payload_bytes;
  auto borrowed = f.Input();
  borrowed.interface_decode = out.interface_decode();
  ASSERT_TRUE(tied::PostKinChk(borrowed,&out,limits));
}
}
