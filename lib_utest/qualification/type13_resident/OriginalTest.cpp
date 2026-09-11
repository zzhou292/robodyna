// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeCompare.h"
#include "../type13_model/OriginalSource.h"

namespace type13_resident_test {
TEST(Type13ResidentOriginal, All4442InitialForcePacketsAnd7493EndpointsMatchNative) {
  type13_model_test::Source input;
  Source source;
  ASSERT_TRUE(source.Initialize(input.Input()));
  ASSERT_EQ(source.model.connection_count(), 4442u);
  ASSERT_EQ(source.domain.node_count(), 7493u);
  auto config = Config(source.domain.node_count());
  t::BatchForecast forecast;
  ASSERT_TRUE(t::Batch::Forecast(config, source.contributions, forecast));
  RecordProperty("device_bytes", std::to_string(forecast.device_bytes));
  RecordProperty("startup_host_bytes", std::to_string(forecast.startup_host_bytes));
  Startup startup;
  ASSERT_TRUE(startup.Initialize(config, source));
  std::size_t round_trip_components = 0;
  std::vector<bool> endpoints(source.domain.node_count());
  for (std::size_t e = 0; e < source.model.connection_count(); ++e) {
    SCOPED_TRACE(e);
    const auto& element = startup.header.model.elements[e];
    const auto& property = *source.model.property(element.property);
    t::NativeEndpointKinematics nodes[2];
    for (unsigned local = 0; local < 2; ++local) {
      nodes[local].position = element.original_position_native[local];
      endpoints[element.nodes[local]] = true;
      t::NativeEndpointKinematics round_trip;
      ASSERT_TRUE(detail::FromSI(source.model.units(), element.reference.position_m[local],
                                 {}, {}, round_trip));
      const double original[3] = {nodes[local].position.x, nodes[local].position.y, nodes[local].position.z};
      const double converted[3] = {round_trip.position.x, round_trip.position.y, round_trip.position.z};
      for (unsigned k = 0; k < 3; ++k) {
        round_trip_components += !fe::shell_startup_detail::SameBits(original[k], converted[k]);
      }
    }
    const auto native = type13_recurrence_test::NativeEvaluate(property, element.reference,
        Virgin(element.reference), nodes, 0, true);
    Agreement(startup.header.slab[0][e], native);
    EXPECT_TRUE(detail::ValidResult(property, startup.header.slab[0][e]));
  }
  EXPECT_EQ(std::count(endpoints.begin(), endpoints.end(), true), 7493);
  EXPECT_GT(round_trip_components, 0u);
  RecordProperty("endpoint_incidence_round_trip_components", std::to_string(round_trip_components));
  auto short_cap = config;
  short_cap.limits.max_host_bytes = forecast.startup_host_bytes - 1;
  t::BatchForecast unchanged{11, 13};
  EXPECT_EQ(t::Batch::Forecast(short_cap, source.contributions, unchanged).status, t::BatchStatus::ResourceLimit);
  EXPECT_EQ(unchanged.startup_host_bytes, 13u);
  short_cap.limits.max_host_bytes = forecast.startup_host_bytes;
  EXPECT_TRUE(t::Batch::Forecast(short_cap, source.contributions, unchanged));
}
} // namespace type13_resident_test
