// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/elements/type13/resident/Storage.h"
#include "lib_src/elements/type13/resident/mapped/Startup.h"

namespace physical_publication_test {
namespace {
fe::type13::BatchConfig BeamConfig(const Fixture& source) {
  fe::type13::BatchConfig config;
  config.owner.owner_id = 1;
  config.owner.node_count = source.domain.node_count();
  config.owner.fixed_dt = H;
  config.owner.has_rotations = true;
  config.owner.has_rotation_presence = true;
  config.owner.temporal_scheme = fe::NodalTemporalScheme::StaggeredHalfKickStart;
  config.owner.velocity_phase = fe::NodalVelocityPhase::Collocated;
  config.owner.rigid_groups.source_instance_id = 1;
  config.owner.rigid_groups.group_count = 2;
  config.owner.rigid_groups.member_count = 6;
  config.owner.rigid_groups.part_group_count = 1;
  config.owner.rigid_groups.plain_source_instance_id = 29;
  config.configuration_id = Configuration;
  config.qualification_id = Qualification;
  config.assembly = fe::type13::BatchAssembly::CinNativeStiffness;
  return config;
}
}
TEST(Type13MappedValues, CompletePhysicalScopePreservesStrictLegacyAdmission) {
  Fixture source;
  ASSERT_FALSE(::testing::Test::HasFailure());
  const auto config = BeamConfig(source);
  fe::type13::BatchForecast forecast;
  EXPECT_EQ(fe::type13::Batch::Forecast(config, source.beam_coefficients, forecast).status,
            fe::type13::BatchStatus::InvalidInput);
  ASSERT_TRUE(fe::type13::Batch::ForecastMapped(config, source.physical, source.rigid,
                                              source.WitnessSource(), forecast));
  EXPECT_GT(forecast.startup_host_bytes, forecast.device_bytes);
  RecordProperty("mapped_tiny_startup_bytes", std::to_string(forecast.startup_host_bytes));
  RecordProperty("mapped_tiny_device_bytes", std::to_string(forecast.device_bytes));
  fe::type13::BatchMappedLimits limits{forecast.startup_host_bytes};
  const auto previous = forecast;
  --limits.max_host_bytes;
  EXPECT_EQ(fe::type13::Batch::ForecastMapped(config, source.physical, source.rigid,
      source.WitnessSource(), forecast, limits).status, fe::type13::BatchStatus::ResourceLimit);
  EXPECT_EQ(forecast.startup_host_bytes, previous.startup_host_bytes);
  EXPECT_EQ(forecast.device_bytes, previous.device_bytes);
  ++limits.max_host_bytes;
  ASSERT_TRUE(fe::type13::Batch::ForecastMapped(config, source.physical, source.rigid,
                                              source.WitnessSource(), forecast, limits));
  auto local_cap = config;
  local_cap.limits.max_host_bytes = 1;
  EXPECT_EQ(fe::type13::Batch::ForecastMapped(local_cap, source.physical, source.rigid,
      source.WitnessSource(), forecast).status, fe::type13::BatchStatus::ResourceLimit);
  auto bad_config = config;
  bad_config.limits.max_nodes = fe::type13::BatchLimits{}.max_nodes + 1;
  EXPECT_EQ(fe::type13::Batch::ForecastMapped(bad_config, source.physical, source.rigid,
      source.WitnessSource(), forecast).status, fe::type13::BatchStatus::ResourceLimit);
}
TEST(Type13MappedValues, RosterAndRetainedTailFailureLeaveOutputsUnchanged) {
  Fixture source;
  const auto config = BeamConfig(source);
  auto cin = source.WitnessSource();
  fe::type13::BatchForecast output{71,73};
  cin.range_count = 2;
  EXPECT_EQ(fe::type13::Batch::ForecastMapped(config, source.physical, source.rigid,
      cin, output).status, fe::type13::BatchStatus::InvalidInput);
  EXPECT_EQ(output.device_bytes,71u);
  EXPECT_EQ(output.startup_host_bytes,73u);
  const auto* tail = &source.rigid.members().data()[source.rigid.members().size()-1];
  const auto old_node = tail->domain_node;
  auto* overlap = reinterpret_cast<fe::type13::BatchForecast*>(
      const_cast<fe::RigidBindingMember*>(tail));
  EXPECT_EQ(fe::type13::Batch::ForecastMapped(config, source.physical, source.rigid,
      source.WitnessSource(), *overlap).status, fe::type13::BatchStatus::InvalidInput);
  EXPECT_EQ(tail->domain_node,old_node);
  ASSERT_TRUE(fe::type13::Batch::ForecastMapped(config, source.physical, source.rigid,
      source.WitnessSource(), output));
}
} // namespace physical_publication_test
