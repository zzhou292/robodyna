// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Transaction.h"
#include <gtest/gtest.h>
#include <type_traits>
namespace fe=tl::fea;
static_assert(!std::is_constructible_v<fe::NativeContactRosterEntry,
    fe::ShellPhysicalScratchParticipation*,std::uint64_t>);
TEST(NativeGroupHost, CountCapAndCanonicalEmptyRejectBeforeAnyEntryRead) {
  fe::ShellPhysicalScratchParticipationForecast out;out.total_host_bytes=987;
  fe::ShellPhysicalScratchRoster roster;
  roster.native_interfaces={reinterpret_cast<const fe::NativeContactRosterEntry*>(1),3};
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(roster,{},out).status,
      fe::ShellPublicationStatus::ResourceLimit);
  EXPECT_EQ(out.total_host_bytes,987u);
  roster.native_interfaces={nullptr,1};
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(roster,{},out).status,
      fe::ShellPublicationStatus::InvalidInput);
  fe::NativeContactRosterEntry empty;
  roster.native_interfaces={&empty,0};
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(roster,{},out).status,
      fe::ShellPublicationStatus::InvalidInput);
  roster.native_interfaces={&empty,1};
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(roster,{},out).status,
      fe::ShellPublicationStatus::InvalidInput);
  EXPECT_EQ(out.total_host_bytes,987u);
}
TEST(NativeGroupHost, LegacyTwoSlotsKeepActualBoundedForecastAndNoNativeView) {
  fe::ShellPhysicalScratchParticipation wall,self;
  fe::ShellPhysicalScratchRoster old{{&wall,1},{&self,2}};
  fe::ShellPhysicalScratchParticipationForecast out;
  ASSERT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(old,{},out).status,
      fe::ShellPublicationStatus::Success);
  EXPECT_EQ(out.configured_issuer_host_bytes,2*sizeof(fe::ShellPhysicalScratchParticipation));
  EXPECT_EQ(out.total_host_bytes,out.publication_host_bytes+out.configured_issuer_host_bytes);
  fe::ShellPhysicalScratchParticipationLimits limit{out.total_host_bytes};
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(old,limit,out).status,
      fe::ShellPublicationStatus::Success);
  --limit.max_host_bytes;
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(old,limit,out).status,
      fe::ShellPublicationStatus::ResourceLimit);
  EXPECT_EQ(old.native_interfaces.count,0u);EXPECT_EQ(old.native_interfaces.entries,nullptr);
}
