#include "../Internal.h"
#include <gtest/gtest.h>
namespace crash::modelio::vehicle::rigid_part {
namespace {
SourceData Fixture() {
    SourceData d;
    Body a;a.source_part_id=41;a.part_nodes={7,9};a.extra_nodes={15};a.node_set_id=61;
    Body b;b.source_part_id=43;b.part_nodes={20,21};b.extra_nodes={26};b.node_set_id=63;
    Body c;c.source_part_id=44;c.part_nodes={30,31};
    d.bodies={a,b,c};d.merges={{41,43}};d.plain_rigid_members={101,103};return d;
}
}
TEST(VehicleRigidTopology, RootMappingPreservesOriginalBodiesExtrasAndTwoPrimaries) {
    auto d=Fixture();tl::fea::rigid::NodalRigidPartTopology t;
    detail::InitializeTopology(d,t,{});
    ASSERT_EQ(t.root_count(),2);EXPECT_EQ(t.parts()[0].root_index,t.parts()[1].root_index);
    EXPECT_NE(t.parts()[0].root_index,t.parts()[2].root_index);
    EXPECT_EQ(t.roots()[0].original_primary_count,2);
    const std::vector<std::uint64_t> expected{7,9,15,20,21,26,30,31};
    EXPECT_EQ(std::vector<std::uint64_t>(t.root_members(),t.root_members()+t.member_count()),expected);
    d.bodies.clear();EXPECT_EQ(t.member_count(),8);EXPECT_EQ(t.parts()[1].source_part_id,43);
}
TEST(VehicleRigidTopology, LastMemberOverlapOrMissingRootFailsBeforePublicationAndRetries) {
    auto d=Fixture();d.bodies.back().part_nodes.back()=103;
    tl::fea::rigid::NodalRigidPartTopology t;
    EXPECT_THROW(detail::InitializeTopology(d,t,{}),std::runtime_error);EXPECT_FALSE(t.prepared());
    d=Fixture();d.merges[0].child_part_id=99;
    EXPECT_THROW(detail::InitializeTopology(d,t,{}),std::runtime_error);EXPECT_FALSE(t.prepared());
    d=Fixture();detail::InitializeTopology(d,t,{});EXPECT_TRUE(t.prepared());
}
}
