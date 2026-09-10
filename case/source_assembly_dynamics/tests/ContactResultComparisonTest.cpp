#include "ContactResultComparison.h"
#include <gtest/gtest.h>
#include <cstring>

namespace crash::cases::source_assembly_dynamics::test {
TEST(ContactResultComparison, PaddingOnlyDifferencesRemainVisibleAndEveryStoredFieldMustMatchBits) {
    using Point=tlfea::contact::NodalWallPointResult;
    using Row=tl::fea::stability::RowContribution;
    Point a{},b{};std::memcpy(&b,&a,sizeof(Point));
    // Object-representation writes touch only the known row/point tail padding.
    constexpr auto row_tail=offsetof(Point,row)+offsetof(Row,valid)+sizeof(bool);
    constexpr auto row_end=offsetof(Point,row)+sizeof(Row);
    constexpr auto point_tail=offsetof(Point,valid)+sizeof(bool);
    static_assert(row_tail<row_end&&point_tail<sizeof(Point));
    auto* bytes=reinterpret_cast<unsigned char*>(&b);
    for(auto i=row_tail;i<row_end;++i)bytes[i]^=0x5a;
    for(auto i=point_tail;i<sizeof(Point);++i)bytes[i]^=0xa5;
    ASSERT_NE(std::memcmp(&a,&b,sizeof(Point)),0);
    auto d=ContactDifference(a,b);EXPECT_EQ(d.field_bytes,0u);
    EXPECT_EQ(d.padding_bytes,row_end-row_tail+sizeof(Point)-point_tail);EXPECT_EQ(d.first_padding_offset,row_tail);
    // Signed zero is numerically equal but is a real stored-field bit change.
    b.row.damping[tl::fea::stability::MaxNodes-1]=-0.;d=ContactDifference(a,b);
    EXPECT_GT(d.field_bytes,0u);EXPECT_STREQ(d.first_field,"row.damping");
    EXPECT_EQ(d.padding_bytes,row_end-row_tail+sizeof(Point)-point_tail);
    RecordProperty("point_size",sizeof(Point));RecordProperty("row_tail_begin",row_tail);
    RecordProperty("row_tail_end",row_end);RecordProperty("point_tail_begin",point_tail);
}
}
