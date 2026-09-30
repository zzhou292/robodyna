#include "../VehiclePhysicalStartup.h"
#include "../JointRuntime.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
namespace crash::cases::vehicle_runtime::test {
namespace {
const auto& Attach() { return vehicle_startup::physical_attachments::test::Actual(); }
const Execution& ShellExecution() {
    static const auto value=Execution::Prepare(Attach().physical());
    return value;
}
const JointModel& Joints() {
    static const auto value=[] {
        const auto source=modelio::type45::VehicleType45Source::Prepare(Attach().physical().source_domain(),
            modelio::type45::Policy::OriginalDirectSdiType45V1);
        return JointModel::Prepare(Attach().physical(),source);
    }();
    return value;
}
Config CaseConfig() {
    Config config;
    config.reserved_step_s=3e-7; // Initial descriptor only; the runtime gate does not advance.
    return config;
}
}
TEST(VehicleJointRuntimeOriginal, CompleteForecastRetainsAllSourceRowsAndChecksCapsBeforeAllocation) {
    const auto& execution=ShellExecution();
    const auto& joints=Joints();
    auto config=CaseConfig();
    const auto baseline=VehiclePhysicalStartup::Preflight(execution,Attach(),config);
    const auto value=VehiclePhysicalStartup::Preflight(execution,Attach(),config,&joints);
    EXPECT_FALSE(baseline.has_type45);
    EXPECT_TRUE(value.has_type45);
    EXPECT_EQ(joints.model().joints().size(),38u);
    EXPECT_EQ(joints.source().data().rows.size(),44u);
    EXPECT_EQ(joints.source().data().boundaries,6u);
    EXPECT_EQ(value.device_bytes,baseline.device_bytes+value.joints.device_bytes);
    EXPECT_EQ(value.retained_source_upper_bound,baseline.retained_source_upper_bound);
    EXPECT_EQ(value.joint_source_bytes,joints.additional_owned_payload_bytes());
    EXPECT_GT(value.joint_source_bytes,joints.source().data().owned_payload_bytes);
    EXPECT_LT(value.joint_source_bytes,1u<<20);
    config.limits.host_bytes=value.peak_host_upper_bound;
    config.limits.device_bytes=value.device_bytes;
    EXPECT_EQ(VehiclePhysicalStartup::Preflight(execution,Attach(),config,&joints).peak_host_upper_bound,
        config.limits.host_bytes);
    --config.limits.host_bytes;
    EXPECT_THROW(VehiclePhysicalStartup::Prepare(execution,Attach(),config,&joints),std::runtime_error);
    config.limits.host_bytes=value.peak_host_upper_bound;
    --config.limits.device_bytes;
    EXPECT_THROW(VehiclePhysicalStartup::Prepare(execution,Attach(),config,&joints),std::runtime_error);
    config=CaseConfig();config.limits.joints.max_joints=37;
    EXPECT_THROW(VehiclePhysicalStartup::Prepare(execution,Attach(),config,&joints),std::runtime_error);
    EXPECT_EQ(joints.source_rows().size(),38u);
    RecordProperty("complete_host_upper",std::to_string(value.peak_host_upper_bound));
    RecordProperty("complete_device_bytes",std::to_string(value.device_bytes));
    RecordProperty("joint_source_bytes",std::to_string(value.joint_source_bytes));
    RecordProperty("joint_device_bytes",std::to_string(value.joints.device_bytes));
}
TEST(VehicleJointRuntimeOriginal, ActualSeventhParticipantVirginReadbackAndExactRetry) {
    auto config=CaseConfig();
    const auto forecast=VehiclePhysicalStartup::Preflight(ShellExecution(),Attach(),config,&Joints());
    config.limits.host_bytes=forecast.peak_host_upper_bound;
    config.limits.device_bytes=forecast.device_bytes;
    auto startup=VehiclePhysicalStartup::Prepare(ShellExecution(),Attach(),config,&Joints());
    const auto initial=startup.accepted();
    const auto first=startup.InspectInitial();
    EXPECT_EQ(first.type45_joints,38u);
    EXPECT_EQ(first.type25_connections,2828u);
    EXPECT_EQ(first.type13_connections,4442u);
    EXPECT_EQ(first.shell_parents,349645u);
    EXPECT_EQ(first.nodes,372435u);
    EXPECT_EQ(first.allocations.device_bytes,forecast.device_bytes);
    --config.limits.device_bytes;
    EXPECT_THROW(startup=VehiclePhysicalStartup::Prepare(ShellExecution(),Attach(),config,&Joints()),std::runtime_error);
    const auto repeated=startup.InspectInitial();
    EXPECT_EQ(repeated.type45_joints,first.type45_joints);
    EXPECT_EQ(repeated.allocations.device_allocations,first.allocations.device_allocations);
    EXPECT_EQ(startup.accepted().owner_id,initial.owner_id);
    EXPECT_EQ(startup.accepted().epoch,0u);
    EXPECT_EQ(startup.accepted().time,0);
    RecordProperty("joint_count",std::to_string(first.type45_joints));
    RecordProperty("explicit_device_bytes",std::to_string(first.allocations.device_bytes));
    RecordProperty("explicit_allocations",std::to_string(first.allocations.device_allocations));
}
} // namespace crash::cases::vehicle_runtime::test
