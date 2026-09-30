#include "MechanicsFixture.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace crash::cases::vehicle_run::test {
TEST(VehicleRunMechanics, SignedChannelsRemainSeparateAndOnsetUsesActualAcceptedClock) {
    MechanicsTotals totals;
    ObserveAcceptedMechanics(totals, MechanicsStep(1), MechanicsStamp(1));
    EXPECT_EQ(totals.solids.native_work[1].accepted_increment_sum_j, 2);
    EXPECT_EQ(totals.solids.hourglass_work[1].accepted_increment_sum_j, .25);
    EXPECT_EQ(totals.solids.hourglass_work[2].accepted_increment_sum_j, -.125);
    EXPECT_EQ(totals.solids.rhs_kick_work.last_increment_j, -.5);
    EXPECT_EQ(totals.beam18.native_work[1].last_increment_j, -2);
    EXPECT_EQ(totals.solids.metal_plastic_work.first_positive_epoch, 0u);
    auto second = MechanicsStep(2);
    second.mechanics.solids.native_internal_work_increment_j[1] = -3;
    second.mechanics.solids.plastic_work_increment_j = .0625;
    second.mechanics.beam18.plastic_work_increment_j = .125;
    second.mechanics.beam18.minimum_native_dt_s = .625;
    second.uniform_motion.maximum_position_error = .005;
    ObserveAcceptedMechanics(totals, second, MechanicsStamp(2));
    EXPECT_EQ(totals.intervals, 2u);
    EXPECT_EQ(totals.last_attempt, 5u);
    EXPECT_EQ(totals.solids.native_work[1].accepted_increment_sum_j, -1);
    EXPECT_EQ(totals.solids.native_work[1].peak_absolute_increment_j, 3);
    EXPECT_EQ(totals.solids.metal_plastic_work.first_positive_epoch, 2u);
    EXPECT_EQ(totals.solids.metal_plastic_work.first_positive_time_s, MechanicsStamp(2).time);
    EXPECT_EQ(totals.beam18.plastic_work.work.accepted_increment_sum_j, .125);
    EXPECT_EQ(totals.beam18.native_step.minimum_observed_s, .625);
    EXPECT_EQ(totals.motion.translation_departure_m.last, .005);
    EXPECT_EQ(totals.motion.translation_departure_m.peak, .01);
    ObserveAcceptedMechanics(totals, MechanicsStep(3), MechanicsStamp(3));
    EXPECT_EQ(totals.solids.metal_plastic_work.work.last_increment_j, 0);
    EXPECT_EQ(totals.solids.metal_plastic_work.work.accepted_increment_sum_j, .0625);
    EXPECT_EQ(totals.solids.metal_plastic_work.first_positive_epoch, 2u);
}
TEST(VehicleRunMechanics, MissingParticipantStaleClockAndForeignSourcesPreserveThenRetry) {
    MechanicsTotals totals;
    ObserveAcceptedMechanics(totals, MechanicsStep(1), MechanicsStamp(1));
    const auto before = MechanicsJson(totals);
    for (unsigned fault = 0; fault < 17; ++fault) {
        SCOPED_TRACE(fault);
        auto step = MechanicsStep(2);
        auto stamp = MechanicsStamp(2);
        switch (fault) {
            case 0: step.mechanics.has_solids = false; break;
            case 1: step.mechanics.has_qeph = false; break;
            case 2: step.mechanics.has_beam18 = false; break;
            case 3: step.mechanics.beam18.phase = tl::fea::beam18::BatchPhase::Accepted; break;
            case 4: step.mechanics.solids.has_completed_interval = false; break;
            case 5: step.mechanics.solids.attempt = totals.last_attempt; break;
            case 6: step.mechanics.type13.attempt++; break;
            case 7: step.mechanics.solids.source_instance_id++; break;
            case 8: step.mechanics.beam18.parent_count++; break;
            case 9: step.mechanics.solids.parent_count[4]++; break;
            case 10: stamp.reactions_valid = false; break;
            case 11: stamp.owner_id++; break;
            case 12: step.base.time = std::nextafter(step.base.time, 1.); break;
            case 13: step.mechanics.has_type45 = false; break;
            case 14: step.mechanics.type45.valid = false; break;
            case 15: step.mechanics.beam18.base_velocity_time = 0; break;
            case 16: step.uniform_motion.nodes--; break;
        }
        EXPECT_THROW(ObserveAcceptedMechanics(totals, step, stamp), std::invalid_argument);
        EXPECT_EQ(MechanicsJson(totals), before);
    }
    EXPECT_THROW(ObserveAcceptedMechanics(totals, MechanicsStep(1), MechanicsStamp(1)), std::invalid_argument);
    EXPECT_THROW(ObserveAcceptedMechanics(totals, MechanicsStep(3), MechanicsStamp(3)), std::invalid_argument);
    ObserveAcceptedMechanics(totals, MechanicsStep(2), MechanicsStamp(2));
    EXPECT_EQ(totals.intervals, 2u);
    EXPECT_EQ(totals.solids.native_work[0].accepted_increment_sum_j, 2);
}
TEST(VehicleRunMechanics, LateNonfiniteNegativePlasticAndOverflowDoNotPartiallyAccumulate) {
    const double infinity = std::numeric_limits<double>::infinity();
    MechanicsTotals totals;
    ObserveAcceptedMechanics(totals, MechanicsStep(1), MechanicsStamp(1));
    const auto before = MechanicsJson(totals);
    for (unsigned fault = 0; fault < 7; ++fault) {
        auto step = MechanicsStep(2);
        if (fault == 0) step.mechanics.solids.native_internal_work_increment_j[4] = infinity;
        if (fault == 1) step.mechanics.solids.physical_hourglass_work_increment_j[2] = infinity;
        if (fault == 2) step.mechanics.solids.plastic_work_increment_j = -.001;
        if (fault == 3) step.mechanics.beam18.internal_drift_work_j = infinity;
        if (fault == 4) step.mechanics.beam18.minimum_native_dt_s = 0;
        if (fault == 5) step.uniform_motion.maximum_spin = infinity;
        if (fault == 6) step.uniform_motion.maximum_velocity_error = -1;
        EXPECT_THROW(ObserveAcceptedMechanics(totals, step, MechanicsStamp(2)), std::invalid_argument);
        EXPECT_EQ(MechanicsJson(totals), before);
    }
    totals.beam18.native_work[1].accepted_increment_sum_j = std::numeric_limits<double>::max();
    auto overflow = MechanicsStep(2);
    overflow.mechanics.beam18.native_internal_work_increment_j[1] = std::numeric_limits<double>::max();
    const auto seeded = MechanicsJson(totals);
    EXPECT_THROW(ObserveAcceptedMechanics(totals, overflow, MechanicsStamp(2)), std::invalid_argument);
    EXPECT_EQ(MechanicsJson(totals), seeded);
    ObserveAcceptedMechanics(totals, MechanicsStep(2), MechanicsStamp(2));
    EXPECT_EQ(totals.intervals, 2u);
}
TEST(VehicleRunMechanics, LegacyEmptyExtendedFamiliesAndAbsentBeamStayExplicit) {
    MechanicsTotals totals;
    auto step = MechanicsStep(1, false);
    step.mechanics.has_type45 = false;
    for (unsigned family : {3u, 4u}) {
        step.mechanics.solids.parent_count[family] = 0;
        step.mechanics.solids.native_internal_work_increment_j[family] = 0;
    }
    // Unconsumed absent beam values do not become observations or admissions.
    step.mechanics.beam18.native_internal_work_increment_j[0] = std::numeric_limits<double>::quiet_NaN();
    ObserveAcceptedMechanics(totals, step, MechanicsStamp(1));
    EXPECT_FALSE(totals.has_beam18);
    EXPECT_EQ(totals.beam18.parents, 0u);
    EXPECT_EQ(totals.solids.parents[4], 0u);
    auto invalid = MechanicsStep(1, false);
    invalid.mechanics.solids.parent_count[4] = 0;
    MechanicsTotals fresh;
    EXPECT_THROW(ObserveAcceptedMechanics(fresh, invalid, MechanicsStamp(1)), std::invalid_argument);
    EXPECT_FALSE(fresh.available);
}
} // namespace crash::cases::vehicle_run::test
