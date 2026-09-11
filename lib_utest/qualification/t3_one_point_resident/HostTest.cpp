#include "../t3_one_point/Fixture.h"
#include "../plasticity_binding/PlasticityBindingFixture.h"
#include "lib_src/elements/t3/T3BatchOnePointSection.h"
#include "lib_src/elements/t3/T3BatchStorage.h"
#include "lib_src/elements/one_point/ShellOnePointStorage.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"

namespace t3_one_point_resident_test {
namespace fe = tl::fea;
namespace storage = fe::shell_batch_plasticity_detail;
namespace pure = t3_one_point_test;
using pure::Bytes;

TEST(T3OnePointResidentHost, CompleteFailureBindingRequiresActualOnePointConstantPolicy) {
  plasticity_binding_test::Fixture f;
  f.sections[1].through_thickness_points = 1;
  f.sections[1].formulation = fe::ShellSectionFormulation::OneThicknessPoint;
  fe::ShellBatchBinding geometry;
  ASSERT_EQ(geometry.Initialize(f.collection()).status, fe::ShellBindingStatus::Success);
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_EQ(catalog.InitializeSections(geometry, f.catalog()).status, fe::ShellPlasticityBindingStatus::Success);
  fe::ShellFailureParentInput rows[2];
  rows[0].source = f.parents[0];
  rows[1].source = f.parents[1];
  for (unsigned fault = 0; fault < 4; ++fault) {
    rows[0].policy = fe::ShellFailurePolicy::ConstantAllPoints;
    rows[0].constant.failure_strain = 2.5;
    rows[0].tab1 = {};
    if (fault == 0) { rows[0].policy = fe::ShellFailurePolicy::None; rows[0].constant = {}; }
    if (fault == 1) rows[0].constant.failure_strain = 0;
    if (fault == 2) rows[0].policy = fe::ShellFailurePolicy::Tab1AnyPoint;
    if (fault == 3) rows[0].tab1.table.failure_strain = .2;
    fe::ShellBatchFailureBinding rejected;
    const auto before = Bytes(rejected);
    const auto report = rejected.Initialize(catalog, rows, 2);
    EXPECT_EQ(report.status, fe::ShellPlasticityBindingStatus::InvalidMaterial) << fault;
    EXPECT_EQ(report.entry, 0u);
    EXPECT_EQ(Bytes(rejected), before);
    rows[0].policy = fe::ShellFailurePolicy::ConstantAllPoints;
    rows[0].constant.failure_strain = 2.5;
    rows[0].tab1 = {};
    EXPECT_EQ(rejected.Initialize(catalog, rows, 2).status, fe::ShellPlasticityBindingStatus::Success);
  }
}

TEST(T3OnePointResidentHost, ExactLayoutCapsAndTypedAvailabilityHaveNoNip3Aliases) {
  storage::OnePointLayout layout;
  ASSERT_TRUE(layout.Initialize(7, 1u << 20));
  EXPECT_EQ(layout.bytes, sizeof(storage::OnePointDeviceStorage) + 14 * sizeof(fe::ShellBatchOnePointSectionState));
  const auto old = Bytes(layout);
  EXPECT_FALSE(layout.Initialize(7, layout.bytes - 1));
  EXPECT_EQ(Bytes(layout), old);
  EXPECT_FALSE(layout.Initialize(std::numeric_limits<std::size_t>::max(), 1u << 20));
  EXPECT_EQ(Bytes(layout), old);
  storage::OnePointLayout exact;
  ASSERT_TRUE(exact.Initialize(7, layout.bytes));
  tl::util::HostArena arena;
  ASSERT_TRUE(arena.Initialize(layout.bytes));
  auto* host = layout.Construct(arena);
  ASSERT_NE(host, nullptr);
  host->section[1][6].point.reported_thickness_m = .0005;
  const auto typed = fe::ShellBatchLayeredSection::OnePoint(host->section[1][6]);
  ASSERT_NE(typed.one_point(), nullptr);
  EXPECT_EQ(typed.law(), fe::ShellSectionLaw::Law44Nip1);
  EXPECT_EQ(typed.plastic(), nullptr);
  EXPECT_EQ(typed.elastic(), nullptr);
  EXPECT_EQ(typed.one_point()->point.reported_thickness_m, .0005);
  std::size_t legacy = 0, one_point = 0;
  fe::ShellBatchFailureLimits failure;
  ASSERT_TRUE(storage::HostStorage::ForecastFailureSections(7, 3, sizeof(fe::ShellBatchFailureBinding),
      1u << 20, 1u << 24, failure, legacy));
  ASSERT_TRUE(storage::HostStorage::ForecastFailureSections(7, 3, sizeof(fe::ShellBatchFailureBinding),
      1u << 20, 1u << 24, failure, one_point, true));
  EXPECT_GT(one_point, legacy + layout.bytes);
  storage::MixedLayout mixed;
  storage::FailureLayout failed;
  storage::OnePointLayout large;
  fe::t3::batch_detail::Layout resident;
  constexpr auto cap = fe::MaxVehicleShellResidentDeviceBytes;
  ASSERT_TRUE(resident.Initialize(fe::MaxVehicleShellResidentParents, fe::MaxVehicleShellResidentNodes, cap));
  ASSERT_TRUE(large.Initialize(fe::MaxVehicleShellResidentParents, cap));
  ASSERT_TRUE(failed.Initialize(fe::MaxVehicleShellResidentParents, cap - large.bytes));
  EXPECT_FALSE(mixed.Initialize(fe::MaxVehicleShellResidentParents, 1024,
      cap - resident.bytes - large.bytes - failed.bytes)); // The complete family includes its shell slabs.
}

TEST(T3OnePointResidentHost, AdapterAdvancesOnlyGenuinePointAndPreservesBothOutputsOnFailure) {
  pure::Fixture f;
  auto history = f.Virgin();
  fe::sections::PointParameters parameters[1]{f.material};
  fe::ShellSectionLaw law[1]{fe::ShellSectionLaw::Law44Nip1};
  fe::ShellBatchSectionState untouched[2][1];
  untouched[0][0].history.point[2].plastic_strain = 777;
  untouched[1][0].history.point[2].plastic_strain = 999;
  const auto before_nip3 = Bytes(untouched);
  storage::MixedDeviceStorage mixed;
  mixed.law = law;
  mixed.plastic.parameters = parameters;
  mixed.plastic.section[0] = untouched[0];
  mixed.plastic.section[1] = untouched[1];
  fe::ShellFailurePolicy policy[1]{fe::ShellFailurePolicy::ConstantAllPoints};
  fe::sections::ConstantFailureParameters failure_parameters[1]{f.failure};
  storage::FailureDeviceStorage failure;
  failure.policy = policy;
  failure.parameters = failure_parameters;
  // NIP3 dynamic failure pointers deliberately absent: this branch must not read them.
  fe::ShellBatchOnePointSectionState state[2][1];
  state[0][0].point.reported_thickness_m = f.reference.input.thickness;
  storage::OnePointDeviceStorage point{{state[0], state[1]}};
  unsigned slab = 0;
  for (unsigned step = 0; step < 96; ++step) {
    const auto interval = pure::Path(f, step);
    fe::t3::OnePointForceTrial oracle;
    ASSERT_EQ(fe::t3::EvaluateOnePointLaw44Force(f.reference, f.material, f.failure,
        history, interval, oracle), fe::t3::Status::kSuccess);
    fe::t3::ForceTrial force;
    ASSERT_EQ(fe::t3::batch_detail::EvaluateOnePointSection(f.reference, history.shell(), interval,
        mixed, failure, point, slab, 0, force), fe::t3::Status::kSuccess);
    const auto& actual = state[1 - slab][0];
    EXPECT_EQ(pure::StateValues({force.proposed_history.data(), actual.point.saved,
        actual.point.failure.history, actual.cumulative_plastic_work_J}),
        pure::StateValues(oracle.proposed_history.values()));
    EXPECT_TRUE(storage::ValidOnePointState(actual, f.material, interval.base_time + interval.dt));
    history = oracle.proposed_history;
    slab = 1 - slab;
  }
  EXPECT_EQ(Bytes(untouched), before_nip3);
  fe::t3::ForceTrial output;
  const auto before_output = Bytes(output);
  const auto before_state = Bytes(state);
  auto bad = pure::Path(f, 96);
  bad.angular_velocity[2].z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(fe::t3::batch_detail::EvaluateOnePointSection(f.reference, history.shell(), bad,
      mixed, failure, point, slab, 0, output), fe::t3::Status::kSuccess);
  EXPECT_EQ(Bytes(output), before_output);
  EXPECT_EQ(Bytes(state), before_state);
  EXPECT_EQ(fe::t3::batch_detail::EvaluateOnePointSection(f.reference, history.shell(), pure::Path(f, 96),
      mixed, failure, point, slab, 0, output), fe::t3::Status::kSuccess);
  auto corrupt = state[1 - slab][0];
  corrupt.point.saved.stress[0] += 1;
  EXPECT_FALSE(storage::ValidOnePointState(corrupt, f.material, 97 * pure::Dt));
  corrupt = state[1 - slab][0];
  *reinterpret_cast<unsigned char*>(&corrupt.point.failure.failed_now) = 2;
  EXPECT_FALSE(storage::ValidOnePointState(corrupt, f.material, 97 * pure::Dt));
}
} // namespace t3_one_point_resident_test
#include "ResidentSource.h"
namespace t3_one_point_resident_test {
TEST(T3OnePointResidentHost, OriginalTriangleAndDistinctSyntheticNeighborsPrepareCompleteSourceScope) {
  Source source;
  fe::ShellBatchBinding geometry;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  ASSERT_TRUE(source.Prepare(geometry, catalog, failure));
  EXPECT_EQ(geometry.t3_source_id(1), 2357656u);
  EXPECT_EQ(geometry.t3_count(), 2u);
  EXPECT_EQ(geometry.qeph_count(), 2u);
  fe::ShellSectionCounts count;
  ASSERT_TRUE(catalog.Counts(fe::ShellBindingFamily::T3, &count));
  EXPECT_EQ(count.law44_nip1, 1u);
  EXPECT_EQ(count.law1, 1u);
  pure::Fixture original;
  for (unsigned n = 0; n < 3; ++n) {
    EXPECT_EQ(geometry.t3_reference(1).input.node_ids[n], original.reference.input.node_ids[n]);
    EXPECT_EQ(Bytes(geometry.t3_reference(1).input.position[n]), Bytes(original.reference.input.position[n]));
  }
}
}
