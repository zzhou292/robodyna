#include "../LoadedWall.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <iostream>
namespace crash::cases::vehicle_wall::test {
TEST(VehicleLoadedWallOriginal, FirstNonzeroContactUsesActualCompleteOwnerAndDiscardRetry) {
    const auto& attachments=vehicle_startup::physical_attachments::test::Actual();
    const auto execution=vehicle_runtime::Execution::Prepare(attachments.physical());
    const auto joint_source=modelio::type45::VehicleType45Source::Prepare(attachments.physical().source_domain(),
        modelio::type45::Policy::OriginalDirectSdiType45V1);
    const auto joints=vehicle_runtime::JointModel::Prepare(attachments.physical(),joint_source);
    ASSERT_EQ(joints.model().joints().size(),38u);
    const char* path=std::getenv("ROBO_VEHICLE_WALL");
    ASSERT_NE(path,nullptr);
    const auto bytes=case_data::ReadPinnedWallManifest(path);
    case_data::CanonicalWall original;
    std::istringstream input(bytes);
    ASSERT_EQ(original.Load(input).status,case_data::WallStatus::Ok);
    auto settings=LoadedWallSettings();
    settings.leading_gap_m=1e-6; // Explicit first-contact qualification; not the default 20 mm run gap.
    settings.requested_duration_s=.005;
    const auto setup=VehicleWallSetup::Prepare(execution,attachments,original,bytes,settings);
    auto config=LoadedWallConfig();
    config.timing.enabled=true;
    const auto forecast=LoadedWall::Preflight(setup,config,{},&joints);
    RecordProperty("complete_host_upper_bound",std::to_string(forecast.peak_host_upper_bound));
    RecordProperty("complete_device_bytes",std::to_string(forecast.device_bytes));
    RecordProperty("scratch_publication_host_bytes",
        std::to_string(forecast.participation.publication_host_bytes));
    std::cout<<"Loaded complete preview before owner allocation: host="<<forecast.peak_host_upper_bound
             <<" device="<<forecast.device_bytes<<" joints="<<joints.model().joints().size()<<std::endl;
    RuntimeLimits short_cap;
    short_cap.host_bytes=forecast.peak_host_upper_bound-1;
    EXPECT_THROW(LoadedWall::Prepare(setup,config,short_cap,&joints),std::runtime_error);
    auto simulation=LoadedWall::Prepare(setup,config,{},&joints);
    ASSERT_NE(simulation.wall_setup(),nullptr);
    EXPECT_EQ(simulation.wall_setup()->geometry().weights()->parent_count(),349645u);
    EXPECT_EQ(simulation.allocations().device_bytes,forecast.device_bytes);
    const auto initial=simulation.accepted();
    const auto rejected=simulation.PrepareStep();
    ASSERT_TRUE(rejected.wall.enabled);
    EXPECT_TRUE(rejected.mechanics.has_type45);
    EXPECT_EQ(rejected.mechanics.type45.joint_count,38u);
    EXPECT_TRUE(rejected.mechanics.type45.automatic_stiffness_initialized);
    EXPECT_TRUE(rejected.wall.accepted.valid);
    EXPECT_TRUE(rejected.wall.prepared.valid);
    EXPECT_EQ(rejected.wall.accepted.contact.resultant.value,0);
    EXPECT_GT(rejected.wall.prepared.contact.resultant.value,0);
    EXPECT_GT(rejected.wall.prepared.contact.potential.value,0);
    EXPECT_GT(rejected.wall.prepared.contact.maximum_penetration,0);
    EXPECT_GT(rejected.structural_step_limit,0);
    EXPECT_LE(initial.fixed_dt,rejected.structural_step_limit);
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(initial,simulation.accepted()));
    simulation.DiscardStep();
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(initial,simulation.accepted()));
    EXPECT_THROW(simulation.last_accepted_step(),std::runtime_error);
    const auto retry=simulation.PrepareStep();
    EXPECT_TRUE(retry.mechanics.type45.automatic_stiffness_initialized);
    EXPECT_EQ(retry.wall.prepared.contact.kick_dt,.5*initial.fixed_dt);
    EXPECT_EQ(retry.wall.prepared.contact.resultant.value,rejected.wall.prepared.contact.resultant.value);
    EXPECT_EQ(retry.wall.prepared.contact.potential.value,rejected.wall.prepared.contact.potential.value);
    EXPECT_EQ(retry.wall.prepared.contact.drift_work,rejected.wall.prepared.contact.drift_work);
    EXPECT_EQ(retry.wall.prepared.interval_tree_used,rejected.wall.prepared.interval_tree_used);
    simulation.CommitStep();
    EXPECT_EQ(simulation.accepted().epoch,1);
    const auto loaded=simulation.PrepareStep();
    EXPECT_EQ(loaded.wall.prepared.contact.kick_dt,initial.fixed_dt);
    EXPECT_GT(loaded.wall.accepted.contact.resultant.value,0);
    EXPECT_GT(loaded.wall.accepted.current_response_rate_upper,0);
    EXPECT_TRUE(loaded.wall.prepared.prepared_activity_available);
    EXPECT_EQ(loaded.wall.prepared.accepted_active_parents,loaded.wall.accepted.accepted_active_parents);
    EXPECT_EQ(loaded.wall.prepared.removed_potential.value,0);
    simulation.CommitStep();
    EXPECT_EQ(simulation.accepted().epoch,2);
    EXPECT_EQ(simulation.last_accepted_step().wall.prepared.contact.resultant.value,loaded.wall.prepared.contact.resultant.value);
    EXPECT_EQ(simulation.allocations().device_bytes,forecast.device_bytes);
    const auto timing=simulation.timing();
    ASSERT_TRUE(timing.enabled);
    EXPECT_FALSE(timing.counter_saturated);
    EXPECT_EQ(timing.clock_failures,0u);
    EXPECT_EQ(timing.backward_samples,0u);
    using Stage=vehicle_dynamics::StepStage;
    for(auto stage:{Stage::PrepareStep,Stage::AssembleQeph,Stage::AdvanceCin,Stage::EvaluateType45,Stage::EvaluateWall}) {
        const auto& row=timing.total[static_cast<std::size_t>(stage)];
        EXPECT_EQ(row.calls,3u);
        EXPECT_EQ(row.valid_samples,3u);
    }
    EXPECT_EQ(timing.total[static_cast<std::size_t>(Stage::Commit)].calls,2u);
    for(std::size_t i=0;i<timing.total.size();++i) {
        RecordProperty(std::string("stage_ns_")+vehicle_dynamics::StepStageNames[i],std::to_string(timing.total[i].wall_ns));
        RecordProperty(std::string("stage_calls_")+vehicle_dynamics::StepStageNames[i],std::to_string(timing.total[i].calls));
    }
    RecordProperty("type45_joints",std::to_string(loaded.mechanics.type45.joint_count));
    RecordProperty("physical_nodes",std::to_string(simulation.accepted().node_count));
    RecordProperty("shell_parents",std::to_string(setup.geometry().weights()->parent_count()));
    RecordProperty("completed_intervals",std::to_string(simulation.accepted().epoch));
    RecordProperty("declared_gap_m","0.000001");
    RecordProperty("transverse_margin_m","0.25");
    std::ostringstream completed;
    completed<<std::setprecision(17)<<simulation.accepted().time;
    RecordProperty("completed_seconds",completed.str());
    std::ostringstream force,penetration,potential;
    force<<std::setprecision(17)<<loaded.wall.accepted.contact.resultant.value;
    penetration<<std::setprecision(17)<<loaded.wall.prepared.contact.maximum_penetration;
    potential<<std::setprecision(17)<<loaded.wall.prepared.contact.potential.value;
    RecordProperty("accepted_force_n",force.str());
    RecordProperty("candidate_penetration_m",penetration.str());
    RecordProperty("candidate_potential_j",potential.str());
    RecordProperty("first_interval_tree_used",retry.wall.prepared.interval_tree_used ? "true" : "false");
    RecordProperty("second_interval_tree_used",loaded.wall.prepared.interval_tree_used ? "true" : "false");
    RecordProperty("scope","two complete loaded contact intervals and discard/retry; no complete 5 ms run");
}
} // namespace crash::cases::vehicle_wall::test
