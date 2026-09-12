#include "MechanicsFixture.h"
#include "../RunState.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>

namespace crash::cases::vehicle_run::test {
TEST(VehicleRunMechanicsDocument, LegacySummaryFieldsAndVersionedScalarObjectRoundTrip) {
    records::test::Directory directory;
    Result result;
    result.session_initialized = true;
    result.loop.kind = StopKind::IntervalLimit;
    result.loop.progress.accepted = {1, MechanicsStamp(1).time};
    auto step = MechanicsStep(1);
    step.mechanics.beam18.plastic_work_increment_j = .03125;
    ObserveAcceptedMechanics(result.loop.progress.mechanics, step, MechanicsStamp(1));
    Config config;
    const auto record = detail::WriteSummary(directory.path, config, Plan(config), {}, result);
    const auto bytes = output::ReadBounded(directory.path / record.file, SummaryByteCap);
    output::Document document;
    document.Parse(bytes.c_str());
    ASSERT_FALSE(document.HasParseError());
    EXPECT_STREQ(document["schema"].GetString(), "robo_dyna.vehicle_run_summary.v1");
    EXPECT_EQ(document["accepted_intervals"].GetUint64(), 1u);
    EXPECT_EQ(document["actual_completed_time_s"].GetDouble(), MechanicsStamp(1).time);
    EXPECT_FALSE(document["contact_observations_available"].GetBool());
    const auto& mechanics = document["accepted_mechanics"];
    EXPECT_STREQ(mechanics["schema"].GetString(), "robo_dyna.accepted_mechanics_summary.v1");
    ASSERT_TRUE(mechanics["available"].GetBool());
    EXPECT_EQ(mechanics["last_attempt"].GetUint64(), 4u);
    EXPECT_EQ(mechanics["solids"]["families"].Size(), 5u);
    const auto& family = mechanics["solids"]["families"][2];
    EXPECT_STREQ(family["family"].GetString(), "solid6z_law42");
    EXPECT_EQ(family["included_hourglass_work"]["last_increment_j"].GetDouble(), -.125);
    const auto& plastic = mechanics["beam18"]["included_plastic_work"];
    EXPECT_EQ(plastic["first_positive_epoch"].GetUint64(), 1u);
    EXPECT_EQ(plastic["first_positive_time_s"].GetDouble(), MechanicsStamp(1).time);
    EXPECT_EQ(plastic["accepted_increment_sum_j"].GetDouble(), .03125);
    EXPECT_EQ(mechanics["motion"]["physical_nodes"].GetUint64(), 100u);
    EXPECT_FALSE(mechanics.HasMember("total_energy_j"));
    EXPECT_FALSE(mechanics.HasMember("maximum_plastic_strain"));
    EXPECT_LE(record.bytes, SummaryByteCap);
    ::testing::Test::RecordProperty("mechanics_totals_bytes", sizeof(MechanicsTotals));
    ::testing::Test::RecordProperty("summary_bytes", record.bytes);
}
TEST(VehicleRunMechanicsDocument, InitialUnavailableAndLegacyNoBeamDoNotInventValues) {
    const auto initial = detail::MechanicsDocument({});
    EXPECT_FALSE(initial["available"].GetBool());
    EXPECT_FALSE(initial.HasMember("solids"));
    EXPECT_FALSE(initial.HasMember("motion"));
    MechanicsTotals totals;
    auto step = MechanicsStep(1, false);
    step.mechanics.solids.parent_count[4] = 0;
    step.mechanics.solids.native_internal_work_increment_j[4] = 0;
    ObserveAcceptedMechanics(totals, step, MechanicsStamp(1));
    const auto legacy = detail::MechanicsDocument(totals);
    EXPECT_FALSE(legacy["has_beam18"].GetBool());
    EXPECT_FALSE(legacy.HasMember("beam18"));
    EXPECT_FALSE(legacy["solids"]["families"][4]["available"].GetBool());
    EXPECT_FALSE(legacy["solids"]["families"][4].HasMember("native_work"));
    EXPECT_FALSE(legacy["solids"]["included_law36_law44_plastic_work"].HasMember("first_positive_epoch"));
}
} // namespace crash::cases::vehicle_run::test
