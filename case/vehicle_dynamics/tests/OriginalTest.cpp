#include "../VehiclePhysicalDynamics.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <iomanip>
#include <sstream>
namespace crash::cases::vehicle_dynamics::test {
namespace {
const vehicle_runtime::Attachments& Source() {return vehicle_startup::physical_attachments::test::Actual();}
const vehicle_runtime::Execution& Shells() {
    static const auto execution=vehicle_runtime::Execution::Prepare(Source().physical());return execution;
}
void CheckMotion(const StepObservation& observation) {
    const auto& motion=observation.uniform_motion;
    std::cout<<"Complete physical free flight t="<<observation.proposed_time
        <<" position_error="<<motion.maximum_position_error<<" velocity_error="<<motion.maximum_velocity_error
        <<" orientation_error="<<motion.maximum_orientation_error<<" spin="<<motion.maximum_spin<<std::endl;
    ASSERT_EQ(motion.nodes,372435);
    ASSERT_LT(motion.maximum_position_error,1e-8);
    ASSERT_LT(motion.maximum_velocity_error,1e-5);
    ASSERT_LT(motion.maximum_orientation_error,1e-8);
    ASSERT_LT(motion.maximum_spin,1e-5);
    ASSERT_TRUE(observation.mechanics.valid);
    ASSERT_FALSE(observation.mechanics.kinetic_available);
}
}
TEST(VehiclePhysicalDynamicsOriginal, ForecastCompleteHostWorkspaceBeforeAnyAllocation) {
    const auto f=VehiclePhysicalDynamics::Preflight(Shells(),Source());
    RecordProperty("workspace_bytes",std::to_string(f.workspace_bytes));
    RecordProperty("peak_host_upper_bound",std::to_string(f.peak_host_upper_bound));
    RecordProperty("explicit_device_bytes",std::to_string(f.startup.device_bytes));
    Config exact;exact.workspace_bytes=f.workspace_bytes;
    EXPECT_EQ(VehiclePhysicalDynamics::Preflight(Shells(),Source(),exact).workspace_bytes,f.workspace_bytes);
    --exact.workspace_bytes;
    EXPECT_THROW(VehiclePhysicalDynamics::Prepare(Shells(),Source(),exact),std::runtime_error);
}
TEST(VehiclePhysicalDynamicsOriginal, CompleteFreeFlightDiscardRetryAndSinglePublication) {
    auto simulation=VehiclePhysicalDynamics::Prepare(Shells(),Source());
    const auto allocations=simulation.allocations();
    const auto initial=simulation.accepted();
    EXPECT_THROW(simulation.CommitStep(),std::runtime_error);
    const auto candidate=simulation.PrepareStep();
    ASSERT_NO_FATAL_FAILURE(CheckMotion(candidate));
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(initial,simulation.accepted()));
    EXPECT_THROW(simulation.PrepareStep(),std::runtime_error);
    simulation.DiscardStep();
    EXPECT_FALSE(simulation.has_prepared_step());
    const auto retry=simulation.PrepareStep();
    EXPECT_EQ(retry.proposed_time,candidate.proposed_time);
    EXPECT_EQ(retry.uniform_motion.maximum_position_error,candidate.uniform_motion.maximum_position_error);
    EXPECT_EQ(retry.uniform_motion.maximum_velocity_error,candidate.uniform_motion.maximum_velocity_error);
    simulation.CommitStep();
    for(unsigned step=1;step<4;++step) {
        ASSERT_NO_FATAL_FAILURE(CheckMotion(simulation.PrepareStep()));
        simulation.CommitStep();
    }
    EXPECT_EQ(simulation.accepted().epoch,4);
    EXPECT_EQ(simulation.last_accepted_step().mechanics.solids.epoch,4);
    EXPECT_EQ(simulation.last_accepted_step().mechanics.type13.epoch,4);
    EXPECT_EQ(simulation.allocations().device_bytes,allocations.device_bytes);
    EXPECT_EQ(simulation.allocations().device_allocations,allocations.device_allocations);
    RecordProperty("epoch",std::to_string(simulation.accepted().epoch));
    std::ostringstream time;time<<std::setprecision(17)<<simulation.accepted().time;
    RecordProperty("completed_physical_seconds",time.str());
}
} // namespace crash::cases::vehicle_dynamics::test
