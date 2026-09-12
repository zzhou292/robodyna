#include "../Internal.h"
#include <gtest/gtest.h>

namespace crash::modelio::physical_domain {
namespace {
std::vector<physical_scope::Group> Groups() {
    std::vector<physical_scope::Group> groups(3);
    for (std::size_t i=0;i<groups.size();++i) {groups[i].id=101+i;groups[i].node_set_id=201+i;}
    groups[0].members={{1,physical_scope::Shell},{2,0},{3,0}};
    groups[1].members={{4,physical_scope::Solid},{5,physical_scope::Type13Endpoint}};
    groups[2].members={{6,0},{7,0}};
    return groups;
}
std::vector<physical_scope::PointMass> Masses() {
    return {{301,2,0,SIZE_MAX,0},{302,6,1,SIZE_MAX,0},{303,8,2,SIZE_MAX,physical_scope::Shell}};
}
}
TEST(VehiclePhysicalDomainValues,RestrictsOnlyExplicitlyUnrepresentedMembersAndKeepsSourceOrder) {
    const auto selected=detail::Select(Groups(),Masses());
    EXPECT_EQ(selected.counts.complete_groups,1);
    EXPECT_EQ(selected.counts.restricted_groups,1);
    EXPECT_EQ(selected.counts.omitted_groups,1);
    EXPECT_EQ(selected.counts.plain_members,4);
    EXPECT_EQ(selected.counts.retained_point_masses,2);
    ASSERT_EQ(selected.groups.size(),3);
    EXPECT_EQ(selected.groups[0].members,(std::vector<std::uint64_t>{1,2}));
    EXPECT_EQ(selected.groups[0].excluded_members,(std::vector<std::uint64_t>{3}));
    EXPECT_NE(selected.groups[0].case_node_set_id,201);
    EXPECT_EQ(selected.groups[1].case_node_set_id,202);
    EXPECT_EQ(selected.groups[2].case_node_set_id,0);
    EXPECT_EQ(selected.groups[2].excluded_members,(std::vector<std::uint64_t>{6,7}));
    EXPECT_EQ(selected.point_nodes.count(6),0);
    EXPECT_EQ(selected.point_nodes.count(8),1);
}
TEST(VehiclePhysicalDomainValues,SingletonAndGeneratedNamespaceCollisionsRejectWithoutPartialResult) {
    auto groups=Groups();
    auto selected=detail::Select(groups,Masses());
    groups[0].members[1].node=9;
    EXPECT_THROW(selected=detail::Select(groups,Masses()),std::runtime_error);
    EXPECT_EQ(selected.groups[0].members.size(),2);
    groups=Groups();groups.back().node_set_id=0x5952000000000001ULL;
    EXPECT_THROW(selected=detail::Select(groups,Masses()),std::runtime_error);
    groups=Groups();groups.back().members.back().roles=256;
    EXPECT_THROW(selected=detail::Select(groups,Masses()),std::runtime_error);
    EXPECT_NO_THROW(selected=detail::Select(Groups(),Masses()));
}
TEST(VehiclePhysicalDomainValues,PartTopologyKeepsOriginalMembersExtrasAndMergeWithSelectedOtherInventory) {
    namespace native=tl::fea::rigid;
    const std::uint64_t a[]{1001,1002},b[]{1003,1004},extra[]{1005},expected[]{1001,1002,1003,1004,1005};
    const std::uint64_t others[]{1,2,3,4,5,6,7};
    const native::PartTopologyPartInput parts[]{{500,a,2},{501,b,2}};
    const native::PartTopologyExtraInput extras[]{ {501,601,extra,1} };
    const native::PartTopologyMerge merges[]{ {500,501} };
    native::PartTopologyInput input{71,parts,2,extras,1,merges,1,expected,5,others,7};
    native::NodalRigidPartTopology original,result;
    ASSERT_TRUE(original.Initialize(input));
    const auto selected=detail::Select(Groups(),Masses());
    ASSERT_NO_THROW(detail::PrepareTopology(original,selected,Limits{}.topology_bytes,result));
    EXPECT_EQ(result.part_count(),2);
    EXPECT_EQ(result.root_count(),1);
    EXPECT_EQ(result.member_count(),5);
    EXPECT_EQ(result.other_rigid_member_count(),4);
    EXPECT_EQ(result.merges()[0].child_part_id,501);
    EXPECT_EQ(result.extras()[0].source_node_set_id,601);
    for(std::size_t n=0;n<5;++n) EXPECT_EQ(result.original_members()[n],original.original_members()[n]);
    const std::uint64_t retained[]{1,2,4,5};
    for(std::size_t n=0;n<4;++n) EXPECT_EQ(result.other_rigid_members()[n],retained[n]);
}
} // namespace crash::modelio::physical_domain
