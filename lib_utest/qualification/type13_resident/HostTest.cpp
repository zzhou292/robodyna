// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>
#include <type_traits>

namespace type13_resident_test {
static_assert(!std::is_copy_constructible_v<t::Batch>);
static_assert(!std::is_copy_assignable_v<t::Batch>);

TEST(Type13ResidentHost, CompleteArenaCapsFailAtomicallyAndRetry) {
  t::BatchLimits limits;
  detail::ArenaLayout full;
  ASSERT_TRUE(detail::MakeLayout(1024, 8192, limits, full));
  EXPECT_LT(full.bytes, limits.max_device_bytes);
  RecordProperty("maximum_device_payload_bytes", std::to_string(full.bytes));
  detail::ArenaLayout original;
  ASSERT_TRUE(detail::MakeLayout(1, 4442, limits, original));
  RecordProperty("original4442_device_payload_bytes", std::to_string(original.bytes));
  RecordProperty("evaluation_bytes", std::to_string(sizeof(t::Evaluation)));
  auto cap = limits;
  cap.max_device_bytes = full.bytes - 1;
  detail::ArenaLayout sentinel;
  sentinel.bytes = 19;
  EXPECT_FALSE(detail::MakeLayout(1024, 8192, cap, sentinel));
  EXPECT_EQ(sentinel.bytes, 19u);
  cap.max_device_bytes = full.bytes;
  ASSERT_TRUE(detail::MakeLayout(1024, 8192, cap, sentinel));
  EXPECT_EQ(sentinel.bytes, full.bytes);
  ++limits.max_connections;
  EXPECT_FALSE(detail::MakeLayout(1, 1, limits, sentinel));
  EXPECT_EQ(sentinel.bytes, full.bytes);
}

TEST(Type13ResidentHost, ImmutableSourceBuildsAuthenticCompleteTT0Cache) {
  type13_model_test::Fixture input;
  Source source;
  ASSERT_TRUE(source.Initialize(input.Input()));
  auto config = Config(source.domain.node_count());
  config.startup = {fe::ShellBatchStartupKind::ReferenceUniformTranslation, {8, -2, 3}};
  Startup original;
  ASSERT_TRUE(original.Initialize(config, source));
  input.nodes.back().position_native.x = 987654;
  input.declaration.input.channels[5].stiffness = -1;
  Startup repeat;
  ASSERT_TRUE(repeat.Initialize(config, source));
  EXPECT_EQ(original.diagnostics.active_count, 2u);
  for (std::size_t e = 0; e < source.model.connection_count(); ++e) {
    Exact(original.header.slab[0][e], repeat.header.slab[0][e]);
    Exact(original.header.slab[0][e], original.header.slab[1][e]);
    EXPECT_TRUE(detail::ValidResult(*source.model.property(0), original.header.slab[0][e]));
    EXPECT_EQ(original.header.model.elements[e].nodes[1],
              source.contributions.records()[2 * e + 1].value.global_node);
  }
}

TEST(Type13ResidentHost, FullHostForecastRejectsLastByteAndPreservesOutput) {
  type13_model_test::Fixture input;
  Source source;
  ASSERT_TRUE(source.Initialize(input.Input()));
  auto config = Config(source.domain.node_count());
  t::BatchForecast good;
  ASSERT_TRUE(t::Batch::Forecast(config, source.contributions, good));
  EXPECT_GT(good.startup_host_bytes, source.contributions.owned_payload_bytes());
  t::BatchForecast output{17, 29};
  config.limits.max_host_bytes = good.startup_host_bytes - 1;
  EXPECT_EQ(t::Batch::Forecast(config, source.contributions, output).status,
            t::BatchStatus::ResourceLimit);
  EXPECT_EQ(output.device_bytes, 17u);
  EXPECT_EQ(output.startup_host_bytes, 29u);
  config.limits.max_host_bytes = good.startup_host_bytes;
  ASSERT_TRUE(t::Batch::Forecast(config, source.contributions, output));
  EXPECT_EQ(output.startup_host_bytes, good.startup_host_bytes);
  config.owner.rigid_groups = {2, 1, 2};
  EXPECT_EQ(t::Batch::Forecast(config, source.contributions, output).status,
            t::BatchStatus::InvalidInput);
  config.owner.rigid_groups = {};
  config.owner.velocity_phase = fe::NodalVelocityPhase::PreviousMidpoint;
  EXPECT_EQ(t::Batch::Forecast(config, source.contributions, output).status,
            t::BatchStatus::InvalidInput);
}

TEST(Type13ResidentHost, TT0RetainsRawSourceBitsAndSubsequentAdapterIsExplicit) {
  type13_model_test::Fixture input;
  input.nodes[2].position_native.x = 2000.123;
  Source source;
  ASSERT_TRUE(source.Initialize(input.Input()));
  Startup startup;
  ASSERT_TRUE(startup.Initialize(Config(3), source));
  const auto& element = startup.header.model.elements[1];
  const auto original = element.original_position_native[1];
  EXPECT_TRUE(fe::shell_startup_detail::SameBits(original.x, input.nodes[2].position_native.x));
  t::NativeEndpointKinematics adapted;
  ASSERT_TRUE(detail::FromSI(source.model.units(), element.reference.position_m[1],
                             {2, 3, 4}, {5, 6, 7}, adapted));
  EXPECT_DOUBLE_EQ(adapted.position.x, element.reference.position_m[1].x / .001);
  EXPECT_NE(adapted.position.x, original.x);
  const auto before = adapted;
  EXPECT_FALSE(detail::FromSI(source.model.units(),
      {std::numeric_limits<double>::infinity(), 0, 0}, {}, {}, adapted));
  EXPECT_DOUBLE_EQ(adapted.position.x, before.position.x);
}

TEST(Type13ResidentHost, RemovalSeparatesCachedForceFromEndpointStiffness) {
  type13_recurrence_test::Case packet(false, .01);
  t::Evaluation initial;
  ASSERT_EQ(t::InitializeForce(packet.property, packet.reference, packet.nodes, initial), t::Status::Success);
  type13_recurrence_test::Mode(packet, 0, .2, 0, 1e-5);
  t::Evaluation removed;
  ASSERT_EQ(t::Evaluate(packet.property, packet.reference, initial.native_history,
                        packet.nodes, 1e-5, removed), t::Status::Success);
  ASSERT_TRUE(removed.newly_failed);
  ASSERT_FALSE(removed.native_history.active);
  EXPECT_NE(removed.local_force_N.x, 0);
  double translation = -1, rotation = -1;
  ASSERT_TRUE(detail::EndpointStiffness(removed, translation, rotation));
  EXPECT_EQ(translation, 0);
  EXPECT_EQ(rotation, 0);
  EXPECT_GT(removed.stability.translation_stiffness_N_per_m, 0);
  ASSERT_TRUE(detail::ValidResult(packet.property, removed));
  removed.signed_work_J[5] = 7;
  EXPECT_FALSE(detail::ValidResult(packet.property, removed));
}
} // namespace type13_resident_test
