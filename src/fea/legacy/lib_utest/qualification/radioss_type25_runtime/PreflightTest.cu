// SPDX-License-Identifier: AGPL-3.0-or-later
#include "MovingCacheRig.h"
#include "FullLedgerRig.h"
#include "MixedRuntimeFixture.h"
#include <gtest/gtest.h>
namespace type25_preflight_test {
namespace n=tlfea::contact::radioss_type25;
void Same(const n::TransactionForecast& a,const n::TransactionForecast& b) {
  EXPECT_EQ(a.device_bytes,b.device_bytes);EXPECT_EQ(a.host_bytes,b.host_bytes);
  EXPECT_EQ(a.startup_host_bytes,b.startup_host_bytes);EXPECT_EQ(a.runtime_device_bytes,b.runtime_device_bytes);
  EXPECT_EQ(a.inventory_device_bytes,b.inventory_device_bytes);EXPECT_EQ(a.maintenance_device_bytes,b.maintenance_device_bytes);
  EXPECT_EQ(a.incidence_device_bytes,b.incidence_device_bytes);EXPECT_EQ(a.normal_device_bytes,b.normal_device_bytes);
  EXPECT_EQ(a.raw_pair_capacity,b.raw_pair_capacity);EXPECT_EQ(a.optimized_capacity,b.optimized_capacity);
  EXPECT_EQ(a.sliding_capacity,b.sliding_capacity);
}
template<class Source>
void ShortCaps(const n::TransactionConfig& config,const Source& source,const tl::fea::ShellPhysicalBinding& physical,
    n::TransactionLimits limits,const n::TransactionForecast& exact) {
  const auto original=limits;
  n::TransactionForecast output=exact;
  limits.max_device_bytes=exact.device_bytes-1;
  EXPECT_EQ(n::Transaction::Preflight(config,source,physical,output,limits).status,n::TransactionStatus::ResourceLimit);
  Same(output,exact);
  limits=original;limits.max_host_bytes=exact.startup_host_bytes-1;
  EXPECT_EQ(n::Transaction::Preflight(config,source,physical,output,limits).status,n::TransactionStatus::ResourceLimit);
  Same(output,exact);
}
TEST(NativeRuntimePreflight, MovingPlanBeforeOwnerMatchesExactCapInitialization) {
  moving_cache_test::Rig rig;auto limits=rig.Limits();n::TransactionForecast plan;
  EXPECT_EQ(rig.owner.accepted().owner_id,0u);
  ASSERT_EQ(n::Transaction::Preflight(rig.config,rig.source.Source(),rig.source.physical.physical,plan,limits).status,n::TransactionStatus::Ok);
  EXPECT_EQ(rig.owner.accepted().owner_id,0u);EXPECT_FALSE(rig.contact.source_info().available);
  EXPECT_GT(plan.normal_device_bytes,0u);
  ShortCaps(rig.config,rig.source.Source(),rig.source.physical.physical,limits,plan);
  limits.max_device_bytes=plan.device_bytes;limits.max_host_bytes=plan.startup_host_bytes;
  ASSERT_NO_THROW(rig.Initialize(limits));Same(rig.contact.allocations(),plan);
}
TEST(NativeRuntimePreflight, GenuineMixedPlanIncludesActivityAndMatchesActualAllocation) {
  type25_source_test::FullLedgerRig rig(true);
  // This existing fixture finalizes its declared T3-only secondary roster and
  // constraints while creating the physical owner. Freeze those genuine source
  // fields before forecasting the contact; the moving test covers pre-owner use.
  ASSERT_NO_THROW(rig.Initialize(false));
  type25_source_test::MixedRuntimeSource source(rig.fixture);n::TransactionForecast plan;
  const auto initial=rig.owner.accepted();EXPECT_FALSE(rig.contact.source_info().available);
  ASSERT_EQ(n::Transaction::Preflight(source.Config(),source.Source(),rig.fixture.physical,plan).status,n::TransactionStatus::Ok);
  EXPECT_EQ(rig.owner.accepted().owner_id,initial.owner_id);EXPECT_EQ(rig.owner.accepted().epoch,initial.epoch);
  ShortCaps(source.Config(),source.Source(),rig.fixture.physical,{},plan);
  n::TransactionLimits limits;
  limits.max_device_bytes=plan.device_bytes;limits.max_host_bytes=plan.startup_host_bytes;
  const auto made=rig.contact.Initialize(source.Config(),source.Source(),rig.owner,rig.publication,
      rig.fixture.physical,rig.Participants(),rig.Identity(),limits);
  ASSERT_EQ(made.status,n::TransactionStatus::Ok)<<made.message;Same(rig.contact.allocations(),plan);
}
TEST(NativeRuntimePreflight, FixedSourceAndMalformedInputRetainAdmissionAndFailureAtomicOutput) {
  type25_source_test::Fixture source;n::FixedMainSource fixed;
  static_cast<n::ContactSourceInput&>(fixed)=source.Source();
  n::TransactionForecast saved;saved.device_bytes=991;saved.host_bytes=87;saved.raw_pair_capacity=41;
  auto output=saved;
  EXPECT_EQ(n::Transaction::Preflight(source.Config(),fixed,source.physical.physical,output).status,n::TransactionStatus::SourceMismatch);
  Same(output,saved);
  for(auto& node:source.nodes)node.constraint=7;
  ASSERT_EQ(n::Transaction::Preflight(source.Config(),fixed,source.physical.physical,output).status,n::TransactionStatus::Ok);
  EXPECT_EQ(output.normal_device_bytes,0u);EXPECT_GT(output.device_bytes,0u);
  const auto exact=output;ShortCaps(source.Config(),fixed,source.physical.physical,{},exact);
  fixed.force_packet_size=0;
  EXPECT_NE(n::Transaction::Preflight(source.Config(),fixed,source.physical.physical,output).status,n::TransactionStatus::Ok);
  Same(output,exact);
}
}
