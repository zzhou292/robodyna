#include "../Internal.h"
#include "../Policy.h"
#include <gtest/gtest.h>

namespace crash::modelio::physical_domain {
namespace {
constexpr auto Extended = Policy::RetainedShellAssembliesExtendedSolidsV4;
struct Fixture {
    std::vector<physical_scope::Group> groups;
    std::vector<physical_scope::PointMass> masses;
    Fixture() {
        for (const auto id : {2200175u, 2200176u, 2200177u, 2200178u, 2200666u, 2200667u}) {
            physical_scope::Group group;
            group.id = id; group.node_set_id = id + 100;
            group.members = {{id * 10u, physical_scope::Solid}, {id * 10u + 1, physical_scope::Solid},
                             {id * 10u + 2, physical_scope::Solid}};
            groups.push_back(std::move(group));
        }
        groups[4].members.back() = {2406558, 0};
        groups[5].members.back() = {2406557, 0};
        masses = {{2409447, 2406557, 0, SIZE_MAX, 0}, {2409448, 2406558, 1, SIZE_MAX, 0}};
        for (std::size_t i = 2; i < 150; ++i)
            masses.push_back({100 + i, 200 + i, i, SIZE_MAX, physical_scope::Shell});
    }
    detail::Selection Prepare() const {
        auto next = detail::Select(groups, masses);
        detail::CheckSelection(groups, masses, next, Extended);
        return next;
    }
};
}
TEST(VehiclePhysicalDomainProfile, ExplicitSolidPoliciesAndCompleteAffectedGroupsPreserveOriginalSets) {
    EXPECT_EQ(detail::SolidPolicy(Policy::RetainedShellAssembliesV1),
              solid_source::Policy::OriginalAdhesive18RubberHephS6zV1);
    EXPECT_EQ(detail::SolidPolicy(Extended), solid_source::Policy::OriginalExtendedSolidsV4);
    EXPECT_THROW(detail::SolidPolicy(static_cast<Policy>(99)), std::runtime_error);
    const Fixture f;
    const auto selected = f.Prepare();
    EXPECT_EQ(selected.counts.complete_groups, 6u);
    EXPECT_EQ(selected.counts.retained_point_masses, 150u);
    for (std::size_t i = 0; i < f.groups.size(); ++i) {
        EXPECT_EQ(selected.groups[i].case_node_set_id, f.groups[i].node_set_id);
        EXPECT_TRUE(selected.groups[i].excluded_members.empty());
        EXPECT_EQ(selected.groups[i].members.back(), f.groups[i].members.back().node);
    }
}
TEST(VehiclePhysicalDomainProfile, LatePartialGroupCannotReplacePriorCompleteSelectionAndRetries) {
    Fixture f;
    auto value = f.Prepare();
    f.groups[3].members.back().roles = 0; // Still two retained members: the old cut rule would accept it.
    const auto clipped = detail::Select(f.groups, f.masses);
    ASSERT_EQ(clipped.groups[3].disposition, GroupDisposition::Restricted);
    EXPECT_NO_THROW(detail::CheckSelection(f.groups, f.masses, clipped, Policy::RetainedShellAssembliesV1));
    EXPECT_THROW(value = f.Prepare(), std::runtime_error);
    EXPECT_EQ(value.groups[3].members.size(), 3u);
    f.groups[3].members.back().roles = physical_scope::Solid;
    EXPECT_NO_THROW(value = f.Prepare());
    auto wrong_order = value;
    std::swap(wrong_order.groups[5].members[0], wrong_order.groups[5].members[1]);
    EXPECT_THROW(detail::CheckSelection(f.groups, f.masses, wrong_order, Extended), std::runtime_error);
}
TEST(VehiclePhysicalDomainProfile, MissingLiteralPointOrGroupDoesNotCreateMassOrMembership) {
    Fixture f;
    auto value = f.Prepare();
    f.masses[1].node = 9999;
    EXPECT_THROW(value = f.Prepare(), std::runtime_error);
    EXPECT_EQ(value.point_nodes.count(2406558), 1u);
    f = Fixture(); f.groups.pop_back();
    EXPECT_THROW(value = f.Prepare(), std::runtime_error);
    EXPECT_EQ(value.groups.size(), 6u);
    f = Fixture();
    EXPECT_NO_THROW(value = f.Prepare());
}
TEST(VehiclePhysicalDomainProfile, SupportsRequiresActualBeamMembersAndAllFourRodMassCards) {
    constexpr auto supports = Policy::RetainedShellAssembliesVehicleSupportsV5;
    EXPECT_EQ(detail::SolidPolicy(supports), solid_source::Policy::OriginalVehicleSupportsV5);
    Fixture f;
    physical_scope::Group beam_group;
    beam_group.id = 2200123; beam_group.node_set_id = 2200223;
    beam_group.members = {{9901, physical_scope::Beam18Endpoint}, {9902, physical_scope::Beam18Endpoint}};
    f.groups.push_back(beam_group);
    const std::uint64_t ids[]{2409489,2409491,2409492,2409494};
    const std::uint64_t nodes[]{2348766,2348765,2348729,2348802};
    for (unsigned i=0;i<4;++i) f.masses.push_back({ids[i],nodes[i],150+i,SIZE_MAX,physical_scope::Beam18Endpoint});
    const auto prepare = [&] {
        auto selected = detail::Select(f.groups,f.masses);
        detail::CheckSelection(f.groups,f.masses,selected,supports);
        return selected;
    };
    auto accepted = prepare();
    EXPECT_EQ(accepted.counts.retained_point_masses,154u);
    EXPECT_EQ(accepted.groups.back().members,(std::vector<std::uint64_t>{9901,9902}));
    f.groups.back().members.push_back({9903,physical_scope::BeamOrientation});
    EXPECT_THROW(accepted=prepare(),std::runtime_error);
    EXPECT_EQ(accepted.groups.back().members.size(),2u);
    f.groups.back().members.pop_back();
    f.masses.back().node=9904;
    EXPECT_THROW(accepted=prepare(),std::runtime_error);
    EXPECT_EQ(accepted.point_nodes.count(nodes[3]),1u);
    f.masses.back().node=nodes[3];
    EXPECT_NO_THROW(accepted=prepare());
}
} // namespace crash::modelio::physical_domain
