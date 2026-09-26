#include "Report.h"
#include "../tests/ActualSources.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_native_contact::test {
namespace {
void CheckPreparationGuard(const Forecast& plan) {
    output::Require(plan.host_preparation_ceiling <= GuardBytes - ExportBytes,
                    "Complete host preparation plus report exceeds the unchanged18GiB qualification guard");
}
template<class F> void RunQualification(const char* name, F body) {
    const auto destination = Destination();
    auto doc = Document(name);
    const auto begin = Clock::now();
    try {
        const auto source = ActualSources();
        output::Number(doc, "source_construction_s", Seconds(begin));
        Sources(doc, source);
        body(doc, source);
        output::Boolean(doc, "completed", true);
    } catch (const std::exception& error) {
        Failure(doc, error);
        output::Number(doc, "total_elapsed_s", Seconds(begin));
        output::WriteJson(destination / "case.json", doc);
        throw;
    }
    output::Number(doc, "total_elapsed_s", Seconds(begin));
    output::WriteJson(destination / "case.json", doc);
}
}
TEST(NativeVehicleCaseActual, SourceAndHostPreparationForecastBeforeFieldAllocation) {
    RunQualification("host_preparation_forecast", [](auto& doc, const auto& source) {
        const auto start = Clock::now();
        const auto plan = VehicleContactStartup::ForecastPreparation(source.owner, source.self, source.wall, source.controls);
        Plan(doc, plan, false);
        output::Number(doc, "host_preparation_forecast_s", Seconds(start));
        CheckPreparationGuard(plan);
        EXPECT_EQ(plan.sources.nodes, 376934u);
        EXPECT_EQ(plan.sources.interfaces[0].native_id, source.controls.self_interface_id());
        EXPECT_EQ(plan.sources.interfaces[1].native_id, source.controls.wall_interface_id());
        EXPECT_EQ(plan.sources.interfaces[0].native_storage_ordinal, 1u);
        EXPECT_EQ(plan.sources.interfaces[1].native_storage_ordinal, 3u);
        EXPECT_GT(plan.tied.mains, 0u);
        EXPECT_EQ(plan.tied.rows, 11165u);
        auto short_cap = Config{};
        short_cap.dynamics.startup.limits.host_bytes = plan.host_preparation_ceiling - 1;
        EXPECT_THROW(VehicleContactStartup::ForecastPreparation(source.owner, source.self, source.wall,
            source.controls, short_cap), std::exception);
        auto bad_horizon = Config{};
        bad_horizon.requested_duration_s = 0;
        EXPECT_THROW(VehicleContactStartup::ForecastPreparation(source.owner, source.self, source.wall,
            source.controls, bad_horizon), std::exception);
    });
}
TEST(NativeVehicleCaseActual, FinalRemovalPlansExposeCompleteRuntimeAndSequentialSeedPeaksWithoutOwner) {
    RunQualification("complete_case_forecast", [](auto& doc, const auto& source) {
        const auto early = VehicleContactStartup::ForecastPreparation(source.owner, source.self, source.wall, source.controls);
        CheckPreparationGuard(early);
        const auto start = Clock::now();
        const auto prepared = VehicleContactStartup::Prepare(source.owner, source.self, source.wall, source.controls);
        output::Number(doc, "host_case_preparation_s", Seconds(start));
        Plan(doc, prepared.forecast(), true);
        EXPECT_GT(prepared.forecast().prepared_source_retained, 0u);
        EXPECT_GT(prepared.forecast().steady_device_bytes, prepared.forecast().physical.startup.device_bytes);
        EXPECT_GE(prepared.forecast().peak_device_bytes, prepared.forecast().steady_device_bytes);
        EXPECT_EQ(prepared.source_input(Role::Self).main_node_count, source.self.main_source().main_nodes().size());
        EXPECT_EQ(prepared.source_input(Role::MeshWall).main_node_count, 4u);
        EXPECT_EQ(prepared.source_input(Role::Self).native_interface_id, source.controls.self_interface_id());
        EXPECT_EQ(prepared.source_input(Role::MeshWall).native_interface_id, source.controls.wall_interface_id());
        // A false aggregate fit is an honest forecast result, not authorization
        // to allocate an owner or to increase a configured/guarded device cap.
    });
}
TEST(NativeVehicleCaseActual, SequentialGenuineInitialStatesReportWarmSignsAndTiedResetWithoutOwner) {
    RunQualification("initial_state_census", [](auto& doc, const auto& source) {
        const auto early = VehicleContactStartup::ForecastPreparation(source.owner, source.self, source.wall, source.controls);
        CheckPreparationGuard(early);
        const auto start = Clock::now();
        const auto prepared = VehicleContactStartup::Prepare(source.owner, source.self, source.wall, source.controls);
        output::Number(doc, "host_case_preparation_s", Seconds(start));
        Plan(doc, prepared.forecast(), true);
        output::Require(prepared.forecast().census_peak_host_bytes <= GuardBytes - ExportBytes,
                        "Source census plus report exceeds the unchanged18GiB qualification guard");
        const auto census_start = Clock::now();
        const auto rows = prepared.CensusInitialStates();
        output::Number(doc, "gpu_source_census_s", Seconds(census_start));
        output::Value table(rapidjson::kArrayType);
        for (std::size_t i = 0; i < rows.size(); ++i) {
            output::Document one;
            one.SetObject();
            InitialValues(one, rows[i]);
            table.PushBack(output::Value(one, doc.GetAllocator()), doc.GetAllocator());
            EXPECT_EQ(rows[i].result.status, n::initial_source::Status::Ok);
            EXPECT_TRUE(rows[i].result.counts_complete);
            EXPECT_EQ(rows[i].identity.nodes, 376934u);
            EXPECT_EQ(rows[i].identity.source.source, prepared.interface_order()[i].native_id);
            EXPECT_EQ(rows[i].result.diagnostics.warm_after_tied + rows[i].result.diagnostics.tied_reset,
                      rows[i].result.diagnostics.warm_before_tied);
        }
        doc.AddMember("initial_states", table, doc.GetAllocator());
    });
}
} // namespace crash::cases::vehicle_native_contact::test
