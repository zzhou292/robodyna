#include "V5Fixture.h"
#include <map>

namespace crash::cases::vehicle_startup::connectivity::test {
TEST(VehicleConnectivityV5, AllRetainedShellsAndHistoricalRegionsReachComputedMainComponent) {
    const auto& value = ActualV5();
    const auto& data = value.data();
    const auto& domain = value.source().physical().source_domain().domain();
    std::map<std::uint64_t,std::size_t> shell_components;
    for (const auto& row : data.relations) {
        if (row.kind > Kind::Qbat) continue;
        const auto label = data.transfer_label[data.slots[row.slot_offset]];
        for (std::size_t slot = 0; slot < row.slot_count; ++slot)
            ASSERT_EQ(data.transfer_label[data.slots[row.slot_offset+slot]],label) << row.source_id;
        ++shell_components[label];
    }
    ASSERT_FALSE(shell_components.empty());
    const auto main = std::max_element(shell_components.begin(),shell_components.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; });
    RecordProperty("shell_transfer_components",shell_components.size());
    RecordProperty("computed_main_label",std::to_string(main->first));
    RecordProperty("main_shell_parents",main->second);
    EXPECT_EQ(shell_components.size(),1u);
    EXPECT_EQ(main->second,349645u);
    // These original NIDs locate the seven historical regions. Neither their
    // old component labels nor a guessed main label is supplied to the graph.
    const std::uint64_t historical[]{2114321,2118226,2159133,2159217,2159443,2163548,2163575};
    for (const auto nid : historical) {
        const auto node = domain.Find(nid);
        ASSERT_NE(node,SIZE_MAX) << nid;
        EXPECT_EQ(data.transfer_label[node],main->first) << "Historical region NID " << nid;
        RecordProperty("region_"+std::to_string(nid),std::to_string(data.transfer_label[node]));
    }
    // Serialize the full immutable graph, even if a reachability expectation
    // above fails: a disconnected result remains reviewable source evidence.
    const auto report = ReportJson(value);
    RecordProperty("source_relations",data.counts.relations);
    RecordProperty("ordered_support_slots",data.counts.slots);
    RecordProperty("graph_owned_bytes",value.forecast().owned_bytes);
    RecordProperty("extra_phase_max_bytes",value.forecast().extra_bytes);
    RecordProperty("inclusive_total_bytes",value.forecast().total_bytes);
    RecordProperty("report_bytes",report.size());
    RecordProperty("report_sha256",output::Sha256(report));
    EXPECT_LE(report.size(),Limits{}.report_bytes);
    if (const auto* path = std::getenv("ROBO_DYNA_CONNECTIVITY_V5_REPORT"); path && *path)
        output::WriteBytes(path,report);
}
} // namespace crash::cases::vehicle_startup::connectivity::test
