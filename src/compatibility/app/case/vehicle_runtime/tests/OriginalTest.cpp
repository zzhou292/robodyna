#include "../VehiclePhysicalStartup.h"
#include "../Packing.h"
#include "../SourceRoles.h"
#include "case/vehicle_startup/physical_attachments/tests/Support.h"
namespace crash::cases::vehicle_runtime::test {
namespace {
const Attachments& Source() { return vehicle_startup::physical_attachments::test::Actual(); }
const Execution& Shells() {
    static const auto value=Execution::Prepare(Source().physical());
    return value;
}
void Print(const Forecast& f) {
    std::cout << "VehiclePhysicalStartup source_upper=" << f.retained_source_upper_bound
              << " retained_upper=" << f.retained_host_upper_bound << " peak_host_upper=" << f.peak_host_upper_bound
              << " exact_device=" << f.device_bytes << " packing=" << f.packing_bytes << std::endl;
    for (std::size_t i=0;i<f.participants.size();++i)
        std::cout << "  participant " << i << " exact_device=" << f.participants[i].device_bytes
                  << " incremental_host_upper=" << f.participants[i].participant_host_bytes
                  << " scratch_upper=" << f.participants[i].startup_scratch_bytes << std::endl;
}
}
TEST(VehiclePhysicalRuntimeOriginal, ForecastAndSourceRoles) {
    const auto& source=Source();
    const auto& execution=Shells();
    const auto f=VehiclePhysicalStartup::Preflight(execution,source);
    Print(f);
    RecordProperty("host_peak_upper_bound",std::to_string(f.peak_host_upper_bound));
    RecordProperty("exact_device_bytes",std::to_string(f.device_bytes));
    RecordProperty("retained_source_upper_bound",std::to_string(f.retained_source_upper_bound));
    EXPECT_LE(f.peak_host_upper_bound,Config{}.limits.host_bytes);
    EXPECT_LE(f.device_bytes,Config{}.limits.device_bytes);
    EXPECT_EQ(execution.execution().counts().material_points,1037877);
    const auto roles=ResolveSourceRoles(source);
    const auto packed=detail::PackOwner(execution.model().coefficients(),execution.model().rigid_assembly(),
        roles,{InitialSpeedMps,0,0},f.packing_bytes);
    EXPECT_EQ(packed.mass.size(),372435);
    Config exact;
    exact.limits.host_bytes=f.peak_host_upper_bound;
    exact.limits.device_bytes=f.device_bytes;
    EXPECT_EQ(VehiclePhysicalStartup::Preflight(execution,source,exact).device_bytes,f.device_bytes);
    --exact.limits.host_bytes;
    EXPECT_THROW(VehiclePhysicalStartup::Prepare(execution,source,exact),std::runtime_error);
    exact.limits.host_bytes=f.peak_host_upper_bound;
    --exact.limits.device_bytes;
    EXPECT_THROW(VehiclePhysicalStartup::Prepare(execution,source,exact),std::runtime_error);
}
TEST(VehiclePhysicalRuntimeOriginal, CompleteInitialOwnerAndTypedReadback) {
    const auto& source=Source();
    const auto& execution=Shells();
    const auto f=VehiclePhysicalStartup::Preflight(execution,source);
    Print(f);
    Config exact;
    exact.limits.host_bytes=f.peak_host_upper_bound;
    exact.limits.device_bytes=f.device_bytes;
    auto owner=VehiclePhysicalStartup::Prepare(execution,source,exact);
    ASSERT_EQ(owner.allocations().device_bytes,f.device_bytes);
    const auto before=owner.accepted();
    const auto view=owner.InspectInitial();
    EXPECT_EQ(view.nodes,372435);
    EXPECT_GT(view.absent_rotations,0);
    EXPECT_EQ(view.cin_secondaries,11165);
    EXPECT_EQ(view.shell_parents,349645);
    EXPECT_EQ(view.rigid_skins,5102);
    EXPECT_EQ(view.material_points,1037877);
    EXPECT_EQ(view.one_point_parents,1);
    EXPECT_EQ(view.four_point_parents,4250);
    EXPECT_EQ(view.three_point_parents,340292);
    EXPECT_EQ(view.type25_connections,2828);
    EXPECT_EQ(view.type13_connections,4442);
    EXPECT_EQ(view.solid_parents,2412);
    EXPECT_EQ(view.stamp.owner_id,before.owner_id);
    EXPECT_EQ(view.stamp.epoch,0);
    EXPECT_EQ(view.stamp.time,0);
    Config short_cap=exact;
    --short_cap.limits.device_bytes;
    EXPECT_THROW(owner=VehiclePhysicalStartup::Prepare(execution,source,short_cap),std::runtime_error);
    EXPECT_EQ(owner.accepted().owner_id,before.owner_id);
    EXPECT_EQ(owner.allocations().device_bytes,f.device_bytes);
    const auto retry=owner.InspectInitial();
    EXPECT_EQ(retry.material_points,view.material_points);
    EXPECT_EQ(retry.allocations.device_allocations,view.allocations.device_allocations);
    RecordProperty("explicit_device_bytes",std::to_string(view.allocations.device_bytes));
    RecordProperty("explicit_device_allocations",std::to_string(view.allocations.device_allocations));
    RecordProperty("absent_ordinary_rotations",std::to_string(view.absent_rotations));
    RecordProperty("owner_id",std::to_string(view.stamp.owner_id));
    RecordProperty("epoch",std::to_string(view.stamp.epoch));
}
} // namespace crash::cases::vehicle_runtime::test
