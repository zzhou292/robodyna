// SPDX-License-Identifier: MIT
#include "../rigid_assembly_owner/Fixture.h"
#include <limits>

namespace physical_owner_forecast_test {
using namespace rigid_assembly_owner_test;
TEST(PhysicalOwnerForecast, ExactCompleteCapAndNoBorrowedArrayRead) {
  Fixture source;
  auto config = source.Config();
  auto cin = source.Cin();
  // Forecast is descriptive: absence of physical values cannot become owner authority.
  cin.mass = nullptr;
  cin.inertia = nullptr;
  cin.witness_ranges = nullptr;
  cin.witnesses = nullptr;
  const auto good = fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin);
  ASSERT_EQ(good.report.status,Code::Ok) << good.report.message;
  EXPECT_GT(good.device_bytes,0u);
  EXPECT_GT(good.owner_host_bytes,0u);
  EXPECT_GT(good.source_host_bytes,0u);
  EXPECT_GT(good.startup_scratch_bytes,0u);
  EXPECT_EQ(good.startup_host_bytes,
      good.source_host_bytes + good.owner_host_bytes + good.startup_scratch_bytes);
  config.max_device_bytes = good.device_bytes;
  cin.limits.max_host_bytes = good.startup_host_bytes;
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin).report.status,Code::Ok);
  --config.max_device_bytes;
  const auto rejected = fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin);
  EXPECT_EQ(rejected.report.status,Code::ResourceLimit);
  EXPECT_EQ(rejected.device_bytes,0u);
  ++config.max_device_bytes;
  --cin.limits.max_host_bytes;
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin).report.status,Code::ResourceLimit);
  ++cin.limits.max_host_bytes;
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin).report.status,Code::Ok);
}
TEST(PhysicalOwnerForecast, CompleteSourceProfileAndCountControls) {
  Fixture source;
  auto config = source.Config();
  const auto cin = source.Cin();
  config.rigid_limits = fe::NodalRigidOwnerLimits::Vehicle();
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin).report.status,Code::ResourceLimit);
  config = source.Config();
  config.node_count = SIZE_MAX;
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin).report.status,Code::ResourceLimit);
  config = source.Config();
  config.fixed_dt = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin).report.status,Code::InvalidInput);
  config = source.Config();
  auto bad = cin;
  bad.witness_count = SIZE_MAX;
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config,source.binding,bad).report.status,Code::ResourceLimit);
  config.temporal_scheme = fe::NodalTemporalScheme::VelocityFirst;
  EXPECT_EQ(fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin).report.status,Code::UnsupportedTemporalScheme);
}
TEST(PhysicalOwnerForecast, OptionalPresenceAndCaptureUseExistingLayouts) {
  Fixture source;
  auto config = source.Config();
  config.capture_force_stage_accelerations = false;
  const auto plain = fe::FENodalState::ForecastAssemblyCin(config,source.binding,source.Cin(),false);
  const auto presence = fe::FENodalState::ForecastAssemblyCin(config,source.binding,source.Cin(),true);
  ASSERT_EQ(plain.report.status,Code::Ok);
  ASSERT_EQ(presence.report.status,Code::Ok);
  EXPECT_EQ(presence.device_bytes-plain.device_bytes,config.node_count);
  config.capture_force_stage_accelerations = true;
  const auto capture = fe::FENodalState::ForecastAssemblyCin(config,source.binding,source.Cin(),true);
  ASSERT_EQ(capture.report.status,Code::Ok);
  EXPECT_GT(capture.device_bytes,presence.device_bytes);
  EXPECT_EQ(capture.startup_host_bytes,presence.startup_host_bytes);
}
} // namespace physical_owner_forecast_test
