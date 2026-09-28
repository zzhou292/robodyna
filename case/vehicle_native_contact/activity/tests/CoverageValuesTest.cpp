#include "../Coverage.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_native_contact::activity::coverage {
TEST(NativeActivityCoverage, WrongSourceConnectivityAndDuplicatesDoNotPublishFlags) {
    const std::vector<std::uint64_t> rows{100,10,1,2,3,3};std::vector<std::uint8_t> selected(1);
    std::uint64_t nodes[]{1,2,3,3};
    EXPECT_THROW(CheckRecord(rows,6,0,999,nodes,4,selected),std::exception);EXPECT_EQ(selected[0],0);
    nodes[2]=9;EXPECT_THROW(CheckRecord(rows,6,0,100,nodes,4,selected),std::exception);EXPECT_EQ(selected[0],0);
    nodes[2]=3;EXPECT_NO_THROW(CheckRecord(rows,6,0,100,nodes,4,selected));EXPECT_EQ(selected[0],1);
    EXPECT_THROW(CheckRecord(rows,6,0,100,nodes,4,selected),std::exception);
}
TEST(NativeActivityCoverage, ExclusionAuditDistinguishesExecutedAndSharedEndpointRows) {
    const tl::fea::NodalDomainNode nodes[]{{1,{0,0,0}},{2,{1,0,0}}};tl::fea::NodalNodeDomain domain;
    ASSERT_TRUE(domain.Initialize({1,nodes,2}));
    const std::vector<std::uint64_t> rows{101,10,1,2,999,102,10,3,4,1,103,10,2,9,888};
    const auto result=Finish(rows,5,2,{1,0,0},domain);
    EXPECT_EQ(result.original,3u);EXPECT_EQ(result.executed,1u);EXPECT_EQ(result.excluded,2u);
    EXPECT_EQ(result.excluded_touching_owner,1u); // Axis-only node1 on102 is excluded.
    const std::uint64_t executed[]{101},excluded[]{102,103};
    EXPECT_EQ(result.executed_ids_sha256,output::Sha256(output::arrays::Encode({output::arrays::Scalar::UInt64,1,1,{}},executed,1)));
    EXPECT_EQ(result.excluded_ids_sha256,output::Sha256(output::arrays::Encode({output::arrays::Scalar::UInt64,2,1,{}},excluded,2)));
}
TEST(NativeActivityCoverage, UnknownFlagsAndDuplicateOriginalIdentitiesRemainClosed) {
    const tl::fea::NodalDomainNode node{1,{0,0,0}};tl::fea::NodalNodeDomain domain;ASSERT_TRUE(domain.Initialize({1,&node,1}));
    EXPECT_THROW(Finish({100,10,1,2},4,2,{2},domain),std::exception);
    EXPECT_THROW(Finish({100,10,1,2,100,20,1,3},4,2,{1,1},domain),std::exception);
}
}
