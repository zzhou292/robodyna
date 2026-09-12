#include "Support.h"
#include "modelio/solid_source/Internal.h"
#include <set>

namespace crash::cases::vehicle_startup::physical_model::extended_test {
TEST(VehicleExtendedPhysicalOriginal, ExactCanonicalDomainAndCompleteAffectedGroupsKeepAllSourceEvidence) {
    const auto& source = Domain();
    const auto& domain = source.domain();
    EXPECT_EQ(source.policy(), DomainPolicy);
    EXPECT_EQ(source.source().solid_source().data().policy, solid_source::Policy::OriginalExtendedSolidsV4);
    const auto ids = solid_source::detail::Decode<std::uint64_t>(Inputs().solids.canonical().data(), "node_ids");
    const auto xyz = solid_source::detail::Decode<double>(Inputs().solids.canonical().data(), "node_positions");
    const auto& scope = source.source().data();
    std::set<std::uint64_t> selected_mass_nodes;
    const auto point_masses = modelio::point_mass::VehiclePointMassSource::Prepare(Scope(), domain);
    for (const auto& record : point_masses.contributions().records())
        selected_mass_nodes.insert(record.source.source_node_id);
    std::size_t n = 0;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        const bool expected = (scope.node_roles[i] & (modelio::physical_scope::PhysicalRoles |
            modelio::physical_scope::ProvisionalType25)) || selected_mass_nodes.count(ids[i]);
        if (!expected) { EXPECT_EQ(domain.Find(ids[i]), SIZE_MAX); continue; }
        ASSERT_LT(n, domain.node_count());
        EXPECT_EQ(domain.nodes()[n].source_id, ids[i]);
        EXPECT_EQ(domain.Find(ids[i]), n);
        Same(domain.nodes()[n].position.x, xyz[3*i]);
        Same(domain.nodes()[n].position.y, xyz[3*i+1]);
        Same(domain.nodes()[n].position.z, xyz[3*i+2]);
        ++n;
    }
    EXPECT_EQ(n, domain.node_count());
    EXPECT_EQ(domain.node_count(), 376634u);
    std::size_t affected = 0, complete = 0, restricted = 0, omitted = 0, members = 0;
    for (const auto& selected : source.plain_groups()) {
        const auto& original = scope.plain_groups.at(selected.source_group);
        ASSERT_EQ(selected.members.size() + selected.excluded_members.size(), original.members.size());
        complete += selected.disposition == domain_source::GroupDisposition::Complete;
        restricted += selected.disposition == domain_source::GroupDisposition::Restricted;
        omitted += selected.disposition == domain_source::GroupDisposition::Omitted;
        members += selected.members.size();
        if (domain_source::detail::RequiresCompleteGroup(DomainPolicy, original.id)) {
            ++affected;
            EXPECT_EQ(selected.disposition, domain_source::GroupDisposition::Complete);
            EXPECT_EQ(selected.case_node_set_id, original.node_set_id);
            EXPECT_TRUE(selected.excluded_members.empty());
        }
        std::size_t kept = 0, excluded = 0;
        for (const auto& member : original.members) {
            if (kept < selected.members.size() && selected.members[kept] == member.node) {
                EXPECT_NE(domain.Find(member.node), SIZE_MAX); ++kept;
            } else {
                ASSERT_LT(excluded, selected.excluded_members.size());
                EXPECT_EQ(selected.excluded_members[excluded++], member.node);
                EXPECT_EQ(domain.Find(member.node), SIZE_MAX);
            }
        }
    }
    EXPECT_EQ(affected, 6u);
    const auto counts = source.counts();
    EXPECT_EQ(counts.complete_groups, complete); EXPECT_EQ(counts.restricted_groups, restricted);
    EXPECT_EQ(counts.omitted_groups, omitted); EXPECT_EQ(counts.plain_members, members);
    EXPECT_EQ(complete + restricted + omitted, 759u); EXPECT_EQ(counts.retained_point_masses, 150u);
    EXPECT_EQ(complete, 733u); EXPECT_EQ(restricted, 22u); EXPECT_EQ(omitted, 4u);
    EXPECT_EQ(members, 7473u);
    const auto& original = Inputs().rigid.topology();
    const auto& topology = source.topology();
    ASSERT_EQ(topology.part_count(), 22u); ASSERT_EQ(topology.root_count(), 20u);
    ASSERT_EQ(topology.member_count(), 5452u);
    for (std::size_t i = 0; i < topology.member_count(); ++i) {
        EXPECT_EQ(topology.original_members()[i], original.original_members()[i]);
        EXPECT_EQ(topology.root_members()[i], original.root_members()[i]);
        EXPECT_NE(domain.Find(topology.original_members()[i]), SIZE_MAX);
    }
    RecordProperty("domain_nodes", domain.node_count());
    RecordProperty("complete_groups", complete); RecordProperty("restricted_groups", restricted);
    RecordProperty("omitted_groups", omitted); RecordProperty("plain_members", members);
    RecordProperty("retained_point_masses", counts.retained_point_masses);
    RecordProperty("domain_owned_bytes", domain.owned_payload_bytes());
    RecordProperty("source_forecast", source.forecast().total_bytes);
    RecordProperty("solid_source_forecast", Inputs().solids.forecast().total_bytes);
}
} // namespace crash::cases::vehicle_startup::physical_model::extended_test
