#include "../VehicleWallStartup.h"
#include "../Artifacts.h"
#include "../RuntimeIdentity.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cstdlib>
#include <iomanip>
#include <sstream>
namespace crash::cases::vehicle_wall::test {
namespace {
const auto& Source() { return vehicle_startup::physical_attachments::test::Actual(); }
const auto& Execution() {
    static const auto value=vehicle_runtime::Execution::Prepare(Source().physical());
    return value;
}
struct WallInput {
    std::string bytes;
    case_data::CanonicalWall wall;
    WallInput() {
        const char* path=std::getenv("ROBO_VEHICLE_WALL");
        output::Require(path && *path,"Explicit original canonical wall fixture required");
        bytes=case_data::ReadPinnedWallManifest(path);
        std::istringstream input(bytes);
        output::Require(wall.Load(input).status==case_data::WallStatus::Ok,"Original wall read failed");
    }
};
const VehicleWallSetup& ActualWall() {
    static const auto value=[] {
        WallInput original;
        return VehicleWallSetup::Prepare(Execution(),Source(),original.wall,original.bytes);
    }();
    return value;
}
void Observe(const VehicleWallSetup& setup,const RuntimeForecast& forecast) {
    const auto bounds=setup.geometry().reference_bounds();
    std::cout<<std::setprecision(17)<<"Surface box ["<<bounds[0].x<<","<<bounds[0].y<<","<<bounds[0].z
        <<"] ["<<bounds[1].x<<","<<bounds[1].y<<","<<bounds[1].z<<"] original coverage: "
        <<setup.original_coverage_report().message<<" selected coverage: "<<setup.coverage_report().message
        <<" setup_host="<<setup.forecast().peak_host_upper_bound
        <<" complete_host="<<forecast.peak_host_upper_bound<<" complete_device="<<forecast.device_bytes
        <<" contact_device="<<forecast.contact.device_bytes<<std::endl;
}
}
TEST(VehicleWallOriginal, CompleteSourceCoverageAndForecastBeforeContactAllocation) {
    const auto& setup=ActualWall();
    EXPECT_EQ(&setup.execution().physical(),&Execution().physical());
    EXPECT_EQ(&setup.attachments().witnesses(),&Source().witnesses());
    EXPECT_EQ(setup.geometry().weights()->parent_count(),349645u);
    EXPECT_EQ(setup.geometry().weights()->node_count(),359785u);
    EXPECT_EQ(setup.geometry().weights()->global_node_count(),372435u);
    EXPECT_EQ(setup.wall().view().vertex_count,62u);
    EXPECT_EQ(setup.wall().view().triangle_count,100u);
    EXPECT_EQ(setup.selected_wall_view().vertex_count,4u);
    EXPECT_EQ(setup.selected_wall_view().triangle_count,2u);
    EXPECT_EQ(setup.coverage_report().status,tlfea::contact::PlanarContactStatus::Ok);
    EXPECT_TRUE(setup.coverage().covered);
    // A separately rebuilt equal catalog is not the exact immutable authority
    // retained by a prospective/actual owner. No second geometry is needed.
    {
        const auto independent=vehicle_runtime::Execution::Prepare(Source().physical());
        EXPECT_THROW(detail::CheckSharedSource(setup,independent,Source()),std::runtime_error);
        EXPECT_NO_THROW(detail::CheckSharedSource(setup,Execution(),Source()));
    }
    const auto forecast=VehicleWallStartup::Preview(setup);
    Observe(setup,forecast);
    EXPECT_EQ(forecast.contact.device_bytes,1087555232u);
    RuntimeLimits exact;
    exact.host_bytes=forecast.peak_host_upper_bound;
    exact.device_bytes=forecast.device_bytes;
    EXPECT_EQ(VehicleWallStartup::Preview(setup,{},exact).device_bytes,forecast.device_bytes);
    --exact.host_bytes;
    EXPECT_THROW(VehicleWallStartup::Preview(setup,{},exact),std::runtime_error);
    WallInput original;
    Limits limits;
    limits.host_bytes=setup.forecast().peak_host_upper_bound;
    EXPECT_EQ(VehicleWallSetup::Preflight(Execution(),Source(),original.wall,original.bytes,{},limits).peak_host_upper_bound,
        limits.host_bytes);
    --limits.host_bytes;
    EXPECT_THROW(VehicleWallSetup::Prepare(Execution(),Source(),original.wall,original.bytes,{},limits),std::runtime_error);
    original.bytes.back()='!';
    EXPECT_THROW(VehicleWallSetup::Preflight(Execution(),Source(),original.wall,original.bytes),std::runtime_error);
    RecordProperty("original_coverage",setup.original_coverage_report().message);
    RecordProperty("selected_profile","envelope-rectangle-v1");
    RecordProperty("setup_host_upper",std::to_string(setup.forecast().peak_host_upper_bound));
    RecordProperty("complete_host_upper",std::to_string(forecast.peak_host_upper_bound));
    RecordProperty("complete_device_bytes",std::to_string(forecast.device_bytes));
    RecordProperty("contact_device_bytes",std::to_string(forecast.contact.device_bytes));
    if (const char* directory=std::getenv("ROBO_VEHICLE_WALL_ARTIFACTS")) {
        ASSERT_NO_THROW(WriteSetupArtifacts(directory,setup));
        EXPECT_THROW(WriteSetupArtifacts(directory,setup),std::runtime_error);
    }
}
TEST(VehicleWallOriginal, ActualInitialOwnerContactIdentityBudgetAndRetry) {
    const auto& setup=ActualWall();
    const auto preview=VehicleWallStartup::Preview(setup);
    vehicle_dynamics::Config config;
    config.startup.reserved_step_s=3e-7; // Constructor candidate only; this gate advances no interval.
    auto dynamics=vehicle_dynamics::VehiclePhysicalDynamics::Prepare(Execution(),Source(),config);
    const auto initial=dynamics.accepted();
    const auto allocations=dynamics.allocations();
    const auto forecast=VehicleWallStartup::Preflight(setup,dynamics);
    EXPECT_EQ(forecast.device_bytes,preview.device_bytes);
    EXPECT_EQ(forecast.peak_host_upper_bound,preview.peak_host_upper_bound);
    RuntimeLimits exact;
    exact.host_bytes=forecast.peak_host_upper_bound;
    exact.device_bytes=forecast.device_bytes-1;
    EXPECT_THROW(VehicleWallStartup::Prepare(setup,dynamics,exact),std::runtime_error);
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(initial,dynamics.accepted()));
    ++exact.device_bytes;
    auto contact=VehicleWallStartup::Prepare(setup,dynamics,exact);
    EXPECT_EQ(contact.contact_allocations().device_bytes,forecast.contact.device_bytes);
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(initial,contact.initial_stamp()));
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(initial,dynamics.accepted()));
    EXPECT_EQ(dynamics.allocations().device_bytes,allocations.device_bytes);
    --exact.host_bytes;
    EXPECT_THROW(contact=VehicleWallStartup::Prepare(setup,dynamics,exact),std::runtime_error);
    EXPECT_EQ(contact.contact_allocations().device_bytes,forecast.contact.device_bytes);
    EXPECT_FALSE(dynamics.has_prepared_step());
    RecordProperty("actual_contact_device_bytes",std::to_string(contact.contact_allocations().device_bytes));
    RecordProperty("epoch",std::to_string(dynamics.accepted().epoch));
    std::ostringstream step;
    step<<std::setprecision(17)<<dynamics.accepted().fixed_dt;
    RecordProperty("reserved_step_s",step.str());
}
} // namespace crash::cases::vehicle_wall::test
