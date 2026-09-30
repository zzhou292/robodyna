#include "../Run.h"
#include "OriginalFixture.h"
#include "case/vehicle_wall/LoadedWall.h"
#include "case/vehicle_dynamics/StructuralLimiterReport.h"
#include "case/vehicle_dynamics/limiter/Identity.h"
#include <iostream>
namespace crash::cases::vehicle_run::test {
TEST(VehicleLimiterValues, OnlySameAcceptedOwnerEpochTimeAndCapturedAttemptMatch) {
    vehicle_dynamics::StepObservation step;
    step.base.owner_id=41;step.base.epoch=2;step.base.time=4e-7;
    step.proposed_time=6e-7;step.structural_step_limit=2.3e-7;
    auto& receipt=step.structural_limiter;
    receipt.owner_id=41;receipt.base_epoch=2;receipt.base_time_s=4e-7;
    receipt.owner_fixed_dt_s=2e-7;receipt.attempt=7;
    receipt.values.kind=tl::fea::NodalCinLimitKind::OrdinaryTranslation;
    receipt.values.minimum_dt_s=2.3e-7;
    auto accepted=step.base;
    accepted.epoch=3;accepted.time=6e-7;accepted.fixed_dt=2e-7;
    EXPECT_TRUE(vehicle_dynamics::limiter::MatchesAccepted(step,accepted));
    auto wrong=accepted;wrong.owner_id++;
    EXPECT_FALSE(vehicle_dynamics::limiter::MatchesAccepted(step,wrong));
    wrong=accepted;wrong.epoch++;
    EXPECT_FALSE(vehicle_dynamics::limiter::MatchesAccepted(step,wrong));
    wrong=accepted;wrong.time=8e-7;
    EXPECT_FALSE(vehicle_dynamics::limiter::MatchesAccepted(step,wrong));
    receipt.attempt=0;
    EXPECT_FALSE(vehicle_dynamics::limiter::MatchesAccepted(step,accepted));
    receipt.attempt=7;receipt.values.kind=tl::fea::NodalCinLimitKind::Unavailable;
    EXPECT_FALSE(vehicle_dynamics::limiter::MatchesAccepted(step,accepted));
}
TEST(VehicleRunSupports, ActualSuccessfulLimiterRetainsSourceThroughDiscardRetryAndAcceptance) {
    auto settings=vehicle_wall::LoadedWallSettings();
    settings.leading_gap_m=1e-6;
    settings.requested_duration_s=.005;
    settings.stiffness_per_area=1e10;
    settings.maximum_penetration_m=.002;
    const auto source=Source(PhysicalProfile::VehicleSupportsV5,&settings);
    auto config=vehicle_wall::LoadedWallConfig();
    config.startup.reserved_step_s=2e-7;
    config.structural.capture_limiter=true;
    const auto forecast=vehicle_wall::LoadedWall::Preflight(source.setup,config,{},&source.joints);
    auto owner=vehicle_wall::LoadedWall::Prepare(source.setup,config,{},&source.joints);
    EXPECT_EQ(owner.allocations().device_bytes,forecast.device_bytes);
    EXPECT_THROW(vehicle_dynamics::StructuralLimiterReport(owner),std::exception);
    const auto candidate=owner.PrepareStep();
    ASSERT_NE(candidate.structural_limiter.values.kind,tl::fea::NodalCinLimitKind::Unavailable);
    EXPECT_EQ(candidate.structural_limiter.values.minimum_dt_s,candidate.structural_step_limit);
    EXPECT_THROW(vehicle_dynamics::StructuralLimiterReport(owner),std::exception);
    owner.DiscardStep();
    const auto retry=owner.PrepareStep();
    EXPECT_EQ(retry.structural_limiter.values.minimum_dt_s,candidate.structural_limiter.values.minimum_dt_s);
    EXPECT_EQ(retry.structural_limiter.source_node_id,candidate.structural_limiter.source_node_id);
    EXPECT_NE(retry.structural_limiter.attempt,candidate.structural_limiter.attempt);
    owner.CommitStep();
    EXPECT_THROW(vehicle_dynamics::StructuralLimiterReport(owner,1),std::exception);
    const auto first=vehicle_dynamics::StructuralLimiterReport(owner);
    output::Document document;
    document.Parse<rapidjson::kParseFullPrecisionFlag>(first.data(),first.size());
    ASSERT_FALSE(document.HasParseError());
    EXPECT_EQ(document["physical_nodes"].GetUint64(),376930u);
    EXPECT_EQ(document["owner_id"].GetUint64(),owner.accepted().owner_id);
    EXPECT_EQ(document["minimum_dt_s"].GetDouble(),retry.structural_step_limit);
    EXPECT_FALSE(document["incident_source_elements"].Empty());
    EXPECT_EQ(first,vehicle_dynamics::StructuralLimiterReport(owner,first.size()));
    EXPECT_THROW(vehicle_dynamics::StructuralLimiterReport(owner,first.size()-1),std::exception);
    std::cout<<"CIN_STRUCTURAL_LIMITER "<<first<<std::endl;
    RecordProperty("first_limiter_json",first);
    owner.PrepareStep();
    owner.CommitStep();
    const auto next=vehicle_dynamics::StructuralLimiterReport(owner);
    std::cout<<"CIN_STRUCTURAL_LIMITER "<<next<<std::endl;
    RecordProperty("second_limiter_json",next);
    RecordProperty("device_bytes",std::to_string(forecast.device_bytes));
    EXPECT_EQ(owner.allocations().device_bytes,forecast.device_bytes);
}
} // namespace crash::cases::vehicle_run::test
