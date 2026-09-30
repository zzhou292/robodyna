// SPDX-License-Identifier: MIT
#include "Oracle.h"
#include "lib_src/elements/qeph/mapped/Startup.h"
#include "lib_src/elements/failure/ShellFailureReadback.h"

namespace qeph_activity_test {
namespace {
namespace side = fe::shell_batch_plasticity_detail;
struct SectionRead {
  side::SetupReport result{side::SetupStatus::Success,"OK"};
  unsigned calls = 0;
  side::SetupReport ReadSections(unsigned,std::size_t,cudaStream_t,double) {
    ++calls;
    return result;
  }
};
struct ReadState {
  q::QephBatchConfig config;
  cudaStream_t stream = nullptr;
  SectionRead sections;
  SectionRead* plasticity = &sections;
  q::BatchReport pending{q::BatchStatus::Success,"OK"};
  unsigned default_calls = 0, runtime_calls = 0;
  q::BatchReport PendingError() const { return pending; }
  q::BatchReport Runtime(cudaError_t,const char* message) {
    ++runtime_calls;
    return {q::BatchStatus::DeviceFailure,message};
  }
  q::BatchReport ValidateMappedSections(unsigned) {
    ++default_calls;
    return {q::BatchStatus::Success,"OK"};
  }
};
}
TEST(QephMappedActivityHost,ReadFailureCallbackPreservesFreshSectionAndErrorPhases) {
  ReadState state;
  unsigned callbacks = 0;
  const auto validate = [&](unsigned) {
    ++callbacks;
    EXPECT_GT(state.sections.calls,0u);
    return q::BatchReport{q::BatchStatus::Success,"OK"};
  };
  ASSERT_EQ(side::ReadFailure(state,0,0).status,q::BatchStatus::Success);
  EXPECT_EQ(state.default_calls,1u);
  ASSERT_EQ(side::ReadFailure(state,0,0,validate).status,q::BatchStatus::Success);
  EXPECT_EQ(callbacks,1u);
  state.pending = {q::BatchStatus::DeviceFailure,"pending"};
  const auto previous = state.sections.calls;
  EXPECT_EQ(side::ReadFailure(state,0,0,validate).status,q::BatchStatus::DeviceFailure);
  EXPECT_EQ(state.sections.calls,previous);
  state.pending = {q::BatchStatus::Success,"OK"};
  state.sections.result = {side::SetupStatus::NonfiniteResult,"section"};
  EXPECT_EQ(side::ReadFailure(state,0,0,validate).status,q::BatchStatus::NonfiniteResult);
  state.sections.result = {side::SetupStatus::DeviceFailure,"copy",cudaErrorInvalidValue};
  EXPECT_EQ(side::ReadFailure(state,0,0,validate).status,q::BatchStatus::DeviceFailure);
  EXPECT_EQ(state.runtime_calls,1u);
  EXPECT_EQ(callbacks,1u);
  EXPECT_EQ(state.default_calls,1u);
}
TEST(QephMappedActivityHost,CompactTailIsContiguousInclusiveAndAbsentFromLegacy) {
  m::ActivityLayout layout;
  ASSERT_TRUE(layout.Initialize(13,324094,1u<<20));
  EXPECT_EQ(layout.active.offset,layout.first_invalid.offset+sizeof(std::uint32_t));
  EXPECT_EQ(layout.bytes-layout.first_invalid.offset,m::ActivityBytes(324094));
  const auto exact = layout.bytes;
  EXPECT_FALSE(layout.Initialize(13,324094,exact-1));
  EXPECT_EQ(layout.bytes,exact);
  EXPECT_TRUE(layout.Initialize(13,324094,exact));
  EXPECT_FALSE(layout.Initialize(SIZE_MAX,324094,SIZE_MAX));
  EXPECT_FALSE(layout.Initialize(0,SIZE_MAX,SIZE_MAX));
  q::batch_detail::Layout legacy, mapped;
  ASSERT_TRUE(legacy.Initialize(324094,376930,fe::MaxVehicleShellResidentDeviceBytes));
  ASSERT_TRUE(mapped.InitializeMapped(324094,376930,fe::MaxVehicleShellResidentDeviceBytes));
  EXPECT_EQ(legacy.assembly.activity.bytes,0u);
  EXPECT_EQ(mapped.assembly.activity.active.count,324094u);
  EXPECT_TRUE(mapped.InitializeMapped(324094,376930,mapped.bytes));
  EXPECT_FALSE(mapped.InitializeMapped(324094,376930,mapped.bytes-1));
  RecordProperty("compact_copy_bytes",std::to_string(m::ActivityBytes(324094)));
  RecordProperty("complete_mapped_device_bytes",std::to_string(mapped.bytes));
}

TEST(QephMappedActivityHost,FrozenValidationKeepsForcePhaseAheadOfRoleAndLastParent) {
  qt_mapped_test::Fixture fixture;
  qt_mapped_test::MakeSkinFixture(fixture);
  Oracle oracle;
  oracle.physical = &fixture.Physical();
  oracle.accepted_stamp = fixture.Config().owner;
  oracle.config = qt_mapped_test::Config<qt_mapped_test::Quad>(fixture,oracle.accepted_stamp);
  q::mapped::Forecast forecast;
  ASSERT_EQ(q::mapped::MakeForecast(oracle.config,fixture.Physical(),fixture.Witnesses(),{},256,forecast).status,
      q::BatchStatus::Success);
  tl::util::HostArena arena;
  ASSERT_TRUE(arena.Initialize(forecast.device.bytes));
  auto* storage = forecast.device.Construct(arena);
  ASSERT_NE(storage,nullptr);
  ASSERT_EQ(q::mapped::BuildModel(oracle.config,fixture.Physical(),*storage).status,q::BatchStatus::Success);
  oracle.staging.assign(storage->slab[0].element,storage->slab[0].element+oracle.config.element_count);
  oracle.histories.sections = {fe::ShellBatchLayeredSection::RigidSkin(),
      fe::ShellBatchLayeredSection::Plastic({})};
  oracle.histories.failures.resize(oracle.config.element_count);
  for (unsigned fault = 0; fault < 5; ++fault) {
    const auto saved = oracle.staging;
    const auto sections = oracle.histories.sections;
    if (fault == 1 || fault == 3) oracle.histories.sections[0] = fe::ShellBatchLayeredSection::Plastic({});
    if (fault == 2 || fault == 3) oracle.staging.back().internal_force[3].x = std::numeric_limits<double>::quiet_NaN();
    if (fault == 4) oracle.staging[0].internal_force[0].x = 1;
    const auto expected = serial::ValidateMappedSections(oracle,0);
    SameReport(oracle.Compact(0),expected);
    if (fault == 3) EXPECT_EQ(expected.element,oracle.staging.size()-1);
    oracle.staging = saved;
    oracle.histories.sections = sections;
  }
  SameReport(oracle.Compact(0),serial::ValidateMappedSections(oracle,0));
}

TEST(QephMappedActivityHost,CompleteForecastChargesCompactHostBackingAndRejectsOneByteShort) {
  qt_mapped_test::Fixture fixture;
  const auto config = qt_mapped_test::Config<qt_mapped_test::Quad>(fixture,fixture.Config().owner);
  q::mapped::Forecast baseline;
  ASSERT_EQ(q::mapped::MakeForecast(config,fixture.Physical(),fixture.Witnesses(),{},256,baseline).status,
      q::BatchStatus::Success);
  auto exact = config;
  exact.storage_limits.max_host_bytes = baseline.host_bytes;
  q::mapped::Forecast next;
  ASSERT_EQ(q::mapped::MakeForecast(exact,fixture.Physical(),fixture.Witnesses(),{},256,next).status,
      q::BatchStatus::Success);
  const auto saved = next.host_bytes;
  --exact.storage_limits.max_host_bytes;
  EXPECT_EQ(q::mapped::MakeForecast(exact,fixture.Physical(),fixture.Witnesses(),{},256,next).status,
      q::BatchStatus::ResourceLimit);
  EXPECT_EQ(next.host_bytes,saved);
}
} // namespace qeph_activity_test
