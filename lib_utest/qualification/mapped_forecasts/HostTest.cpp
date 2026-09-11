// SPDX-License-Identifier: MIT
#include "../qbat_mapped/Fixture.h"
#include "lib_src/elements/qeph/QephBatch.h"
#include "lib_src/elements/t3/T3Batch.h"
#include "lib_src/elements/qbat/QbatBatch.h"
#include "lib_src/elements/type25/Type25Batch.h"

namespace mapped_forecasts_test {
namespace fe = tl::fea;
using Fixture = qbat_mapped_test::Fixture;

template<class Config> Config Configuration(const Fixture& source,std::size_t count) {
  Config config;
  config.owner = source.Config().owner;
  config.configuration_id = 81;
  config.qualification_id = 82;
  config.element_count = count;
  return config;
}
void Partition(const fe::ShellMappedFootprint& f,const Fixture& source) {
  EXPECT_EQ(f.source_host_bytes,source.physical.owned_payload_bytes());
  EXPECT_GT(f.device_bytes,0u);
  EXPECT_GT(f.participant_host_bytes,0u);
  EXPECT_GT(f.startup_scratch_bytes,f.device_bytes);
  EXPECT_EQ(f.source_host_bytes + f.participant_host_bytes + f.startup_scratch_bytes,
            f.startup_host_bytes);
}

TEST(MappedForecast, AllFourPublicQueriesAndExactHostDeviceCaps) {
  Fixture source;
  auto q = Configuration<fe::qeph::QephBatchConfig>(source,2);
  auto t = Configuration<fe::t3::T3BatchConfig>(source,1);
  auto b = source.Config();
  auto s = Configuration<fe::type25::BatchConfig>(source,source.mechanics.springs.connection_count());
  q.usage = fe::qeph::BatchUsage::CoupledForces;
  t.usage = fe::t3::BatchUsage::CoupledForces;
  const auto cin = source.Witnesses();
  const auto qf = fe::qeph::QephBatch::ForecastMapped(q,source.physical,cin);
  const auto tf = fe::t3::T3Batch::ForecastMapped(t,source.physical,cin);
  const auto bf = fe::qbat::Batch::ForecastMapped(b,source.physical,cin);
  const auto sf = fe::type25::Batch::ForecastMapped(s,source.physical,cin,fe::type25::CapacityProfile::Legacy);
  ASSERT_EQ(qf.report.status,fe::qeph::BatchStatus::Success);
  ASSERT_EQ(tf.report.status,fe::t3::BatchStatus::Success);
  ASSERT_EQ(bf.report.status,fe::qbat::BatchStatus::Success);
  ASSERT_EQ(sf.report.status,fe::type25::BatchStatus::Success);
  for (const auto* f : {&qf.footprint,&tf.footprint,&bf.footprint,&sf.footprint}) Partition(*f,source);
  q.max_device_bytes = qf.footprint.device_bytes;
  t.max_device_bytes = tf.footprint.device_bytes;
  b.max_device_bytes = bf.footprint.device_bytes;
  s.max_device_bytes = sf.footprint.device_bytes;
  q.storage_limits.max_host_bytes = qf.footprint.startup_host_bytes;
  t.storage_limits.max_host_bytes = tf.footprint.startup_host_bytes;
  b.storage_limits.max_host_bytes = bf.footprint.startup_host_bytes;
  s.max_host_bytes = sf.footprint.startup_host_bytes;
  EXPECT_EQ(fe::qeph::QephBatch::ForecastMapped(q,source.physical,cin).report.status,fe::qeph::BatchStatus::Success);
  EXPECT_EQ(fe::t3::T3Batch::ForecastMapped(t,source.physical,cin).report.status,fe::t3::BatchStatus::Success);
  EXPECT_EQ(fe::qbat::Batch::ForecastMapped(b,source.physical,cin).report.status,fe::qbat::BatchStatus::Success);
  EXPECT_EQ(fe::type25::Batch::ForecastMapped(s,source.physical,cin,fe::type25::CapacityProfile::Legacy).report.status,
            fe::type25::BatchStatus::Success);
  --q.max_device_bytes;
  --t.max_device_bytes;
  --b.max_device_bytes;
  --s.max_device_bytes;
  EXPECT_EQ(fe::qeph::QephBatch::ForecastMapped(q,source.physical,cin).report.status,fe::qeph::BatchStatus::ResourceLimit);
  EXPECT_EQ(fe::t3::T3Batch::ForecastMapped(t,source.physical,cin).report.status,fe::t3::BatchStatus::ResourceLimit);
  EXPECT_EQ(fe::qbat::Batch::ForecastMapped(b,source.physical,cin).report.status,fe::qbat::BatchStatus::ResourceLimit);
  EXPECT_EQ(fe::type25::Batch::ForecastMapped(s,source.physical,cin,fe::type25::CapacityProfile::Legacy).report.status,
            fe::type25::BatchStatus::ResourceLimit);
  ++q.max_device_bytes;
  ++t.max_device_bytes;
  ++b.max_device_bytes;
  ++s.max_device_bytes;
  --q.storage_limits.max_host_bytes;
  --t.storage_limits.max_host_bytes;
  --b.storage_limits.max_host_bytes;
  --s.max_host_bytes;
  EXPECT_EQ(fe::qeph::QephBatch::ForecastMapped(q,source.physical,cin).report.status,fe::qeph::BatchStatus::ResourceLimit);
  EXPECT_EQ(fe::t3::T3Batch::ForecastMapped(t,source.physical,cin).report.status,fe::t3::BatchStatus::ResourceLimit);
  EXPECT_EQ(fe::qbat::Batch::ForecastMapped(b,source.physical,cin).report.status,fe::qbat::BatchStatus::ResourceLimit);
  EXPECT_EQ(fe::type25::Batch::ForecastMapped(s,source.physical,cin,fe::type25::CapacityProfile::Legacy).report.status,
            fe::type25::BatchStatus::ResourceLimit);
}

TEST(MappedForecast, InvalidBorrowedExtentAndFamilyNeverAllocateOrPublishCapacity) {
  Fixture source;
  auto config = Configuration<fe::t3::T3BatchConfig>(source,1);
  config.usage = fe::t3::BatchUsage::CoupledForces;
  auto cin = source.Witnesses();
  const auto original = qbat_binding_test::Bytes(config);
  cin.witness_count = SIZE_MAX;
  const auto rejected = fe::t3::T3Batch::ForecastMapped(config,source.physical,cin);
  EXPECT_NE(rejected.report.status,fe::t3::BatchStatus::Success);
  EXPECT_EQ(rejected.footprint.startup_host_bytes,0u);
  EXPECT_EQ(rejected.footprint.device_bytes,0u);
  EXPECT_EQ(qbat_binding_test::Bytes(config),original);
  cin = source.Witnesses();
  ASSERT_EQ(fe::t3::T3Batch::ForecastMapped(config,source.physical,cin).report.status,fe::t3::BatchStatus::Success);
  ++config.element_count;
  EXPECT_NE(fe::t3::T3Batch::ForecastMapped(config,source.physical,cin).report.status,fe::t3::BatchStatus::Success);
}

TEST(MappedForecast, PartitionRejectsOverflowAndKeepsExistingBudgetPolicy) {
  fe::ShellMappedFootprint output{1,2,3,4,5};
  const auto before = output;
  EXPECT_FALSE(fe::shell_mapped_detail::MakeFootprint(SIZE_MAX,SIZE_MAX-1,2,1,output));
  EXPECT_EQ(output.startup_host_bytes,before.startup_host_bytes);
  EXPECT_TRUE(fe::shell_mapped_detail::MakeFootprint(100,30,40,10,output));
  EXPECT_EQ(output.device_bytes,40u);
  EXPECT_EQ(output.participant_host_bytes,20u);
  EXPECT_EQ(output.startup_scratch_bytes,50u);
}
} // namespace mapped_forecasts_test
