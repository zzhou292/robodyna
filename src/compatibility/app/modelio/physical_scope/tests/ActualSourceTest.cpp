#include "ActualSupport.h"
#include <fstream>
#include <numeric>

namespace crash::modelio::physical_scope {
using test::Inputs;
using test::Actual;
TEST(PhysicalScopeOriginal,CompleteGroupsAndSourceRolesRetainAllRowsAndIncidence) {
    const auto& scope = Actual();
    const auto& data = scope.data();
    ASSERT_EQ(data.plain_groups.size(), 759);
    ASSERT_EQ(data.part_roots.size(), 20);
    ASSERT_EQ(data.spotwelds.size(), 2828);
    ASSERT_EQ(data.rigid_skin.size(), 5102);
    EXPECT_EQ(data.node_roles.size(), 393165);
    EXPECT_EQ(data.counts.role_nodes[0], 359785);
    EXPECT_EQ(data.counts.role_nodes[1], 7493);
    EXPECT_EQ(data.point_masses.size(), 155);
    EXPECT_EQ(scope.point_mass_source().data().consumed.size(), 54);
    std::size_t plain_members = 0, root_members = 0;
    for (const auto& group : data.plain_groups) plain_members += group.members.size();
    for (const auto& group : data.part_roots) root_members += group.members.size();
    EXPECT_EQ(plain_members, 7539);
    EXPECT_EQ(root_members, 5452);
    EXPECT_GE(data.counts.with_type25_nodes, data.counts.baseline_nodes);
    EXPECT_GE(data.counts.plain_complete_after, data.counts.plain_complete_before);
    EXPECT_EQ(data.counts.roots_complete_before, 20);
    for (const auto& row : data.rigid_skin) EXPECT_LT(row.root_index, data.part_roots.size());
    std::size_t slots = 0;
    for (const auto& node : data.evidence) {
        for (const auto& incidence : node.incidence) {
            EXPECT_GT(incidence.local_slots, 0);
            if (incidence.orientation_only) EXPECT_FALSE(incidence.selected);
            ++slots;
        }
    }
    RecordProperty("evidence_incidences", slots);
    RecordProperty("baseline_nodes", data.counts.baseline_nodes);
    RecordProperty("with_type25_nodes", data.counts.with_type25_nodes);
    RecordProperty("plain_complete_before", data.counts.plain_complete_before);
    RecordProperty("plain_complete_after", data.counts.plain_complete_after);
    RecordProperty("forecast_bytes", scope.forecast().total_bytes);
    RecordProperty("owned_payload_bytes", data.owned_payload_bytes);
    const auto* report = std::getenv("ROBO_PHYSICAL_SCOPE_REPORT");
    if (report && *report) {
        output::Require(!std::filesystem::exists(report), "Physical census report path already exists");
        const auto bytes = ReportJson(scope);
        std::ofstream stream(report, std::ios::binary);
        output::Require(bool(stream.write(bytes.data(), bytes.size())), "Physical census report write failed");
    }
}
TEST(PhysicalScopeOriginal,ExactCapRetryAndImmutableInputLifetimes) {
    const auto& input = Inputs();
    auto scope = Actual();
    const auto* before = scope.data().node_roles.data();
    Limits limits;
    limits.host_bytes = scope.forecast().total_bytes - 1;
    EXPECT_THROW(scope = PhysicalScope::Prepare(input.masses, input.tied, input.beams, input.solids, limits), std::runtime_error);
    EXPECT_EQ(scope.data().node_roles.data(), before);
    limits.host_bytes += 1;
    EXPECT_NO_THROW(scope = PhysicalScope::Prepare(input.masses, input.tied, input.beams, input.solids, limits));
    EXPECT_EQ(scope.data().node_roles, Actual().data().node_roles);
    EXPECT_EQ(&scope.tied_source().data(), &input.tied.data());
    EXPECT_EQ(&scope.point_mass_source().rigid_source().data(), &input.rigid.data());
    limits = {};
    limits.spotwelds = 2827;
    EXPECT_THROW(PhysicalScope::Prepare(input.masses, input.tied, input.beams, input.solids, limits), std::runtime_error);
    EXPECT_EQ(scope.data().plain_groups.back().id, Actual().data().plain_groups.back().id);
}
} // namespace crash::modelio::physical_scope
