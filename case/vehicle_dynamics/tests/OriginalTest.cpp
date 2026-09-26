#include "../VehiclePhysicalDynamics.h"
#include "../ExecutionAccess.h"
#include "case/vehicle_runtime/CaptureAccess.h"
#include <cstring>
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <iomanip>
#include <limits>
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
    RecordProperty("explicit_device_bytes",std::to_string(f.startup.device_bytes+f.motion.device_bytes));
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
TEST(VehiclePhysicalDynamicsOriginal, CompletePostCinStructuralStepAdmission) {
    Config config;
    config.structural={tl::fea::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8};
    auto bad=config;bad.structural.factor=0;
    EXPECT_THROW(VehiclePhysicalDynamics::Preflight(Shells(),Source(),bad),std::runtime_error);
    auto simulation=VehiclePhysicalDynamics::Prepare(Shells(),Source(),config);
    const auto allocations=simulation.allocations();
    try {
        const auto& candidate=simulation.PrepareStep();
        ASSERT_NO_FATAL_FAILURE(CheckMotion(candidate));
        ASSERT_GE(candidate.structural_step_limit,config.startup.reserved_step_s);
        ASSERT_LT(candidate.structural_step_limit,std::numeric_limits<double>::max());
        std::ostringstream limit;limit<<std::setprecision(17)<<candidate.structural_step_limit;
        RecordProperty("post_cin_structural_limit_seconds",limit.str());
        simulation.CommitStep();
        EXPECT_EQ(simulation.accepted().epoch,1u);
        EXPECT_EQ(simulation.allocations().device_bytes,allocations.device_bytes);
        EXPECT_EQ(simulation.allocations().device_allocations,allocations.device_allocations);
    } catch(const StepSizeError& error) {
        FAIL()<<"Complete model requires smaller configured dt; measured post-CIN limit="
            <<std::setprecision(17)<<error.limit()<<" at physical node="<<error.node();
    }
}
TEST(VehiclePhysicalDynamicsOriginal, InitialObserverReferenceUsesExactCompleteSourceDomainCoordinates) {
    auto startup=vehicle_runtime::VehiclePhysicalStartup::Prepare(Shells(),Source());
    const auto& domain=*Shells().model().coefficients().domain();
    std::vector<double> positions(3*domain.node_count()),velocity(3*domain.node_count());
    const auto stamp=vehicle_runtime::detail::CaptureAccess::Nodes(startup,
        positions.data(),velocity.data(),domain.node_count());
    ASSERT_EQ(stamp.epoch,0u);
    ASSERT_EQ(stamp.node_count,domain.node_count());
    for(std::size_t i=0;i<domain.node_count();++i) {
        const auto source=domain.nodes()[i].position;
        const double expected[]{source.x,source.y,source.z};
        ASSERT_EQ(std::memcmp(positions.data()+3*i,expected,sizeof(expected)),0)<<i;
    }
    tl::fea::NodalUniformMotionObserver observer;
    // Existing private implementation bridge; this test reads/initializes the
    // observer only and adds no public setter, callback or receipt authority.
    auto& owner=ExecutionAccess::Get(startup).owner;
    ASSERT_EQ(observer.Initialize(owner).status,tl::fea::NodalStatus::Ok);
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(owner.accepted(),stamp));
    RecordProperty("source_nodes_checked",std::to_string(domain.node_count()));
}
} // namespace crash::cases::vehicle_dynamics::test
