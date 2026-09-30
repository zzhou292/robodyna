#include "NativeOracle.h"
#include "../Internal.h"
#include "../../tests/TinyFixture.h"
#include <algorithm>
#include <numeric>

extern "C" void my_orders_(int*, unsigned*, unsigned*, unsigned*, int*, int*);
namespace crash::modelio::tied_shell::test {
TEST(TiedPackingNative, MixedFamiliesPreserveTuplesAndBothNativeNodeSortBranches) {
    const std::vector<std::array<int,4>> q4{{9,1,8,2}, {4,3,7,6}};
    const std::vector<std::array<int,3>> t3{{8,2,7}, {2,7,5}};
    for (const int nodes : {16,1000}) {
        SCOPED_TRACE(nodes);
        const auto native = ObserveNative(nodes, q4, t3, {16,12,15,11,14,10,13,9});
        EXPECT_EQ(native.order, (std::vector<int>{3,1,2,0}));
        EXPECT_EQ(native.master_nodes, (std::vector<int>{1,2,3,4,5,6,7,8,9}));
        EXPECT_EQ(native.slave_nodes, (std::vector<int>{9,10,11,12,13,14,15,16}));
        EXPECT_EQ(native.rect, (std::vector<int>{2,7,5,5, 4,3,7,6, 8,2,7,7, 9,1,8,2}));
        EXPECT_TRUE(native.cleared);
    }
    EXPECT_EQ(ObserveNative(100, {}, {{8,2,7}}, {40,30}).rect,
              (std::vector<int>{8,2,7,7}));
    EXPECT_EQ(ObserveNative(100, {{9,1,8,2}}, {}, {40,30}).rect,
              (std::vector<int>{9,1,8,2}));
}
TEST(TiedPackingNative, FifthWordAndStableCompleteTiesAreNotElementIdOrder) {
    std::vector<unsigned> data{9,8,7,6,30, 9,8,7,6,2, 9,8,7,6,2, 1,8,7,6,99};
    std::vector<unsigned> work(70000), index(8);
    int mode = 0, count = 4, width = 5;
    my_orders_(&mode, work.data(), data.data(), index.data(), &count, &width);
    EXPECT_EQ((std::vector<unsigned>(index.begin(), index.begin()+4)),
              (std::vector<unsigned>{4,2,3,1}));
    const auto native = ObserveNative(100, {{9,8,7,6}, {9,8,7,6}}, {{2,3,4}}, {40,30});
    EXPECT_EQ(native.order, (std::vector<int>{2,0,1}));
}
TEST(TiedPackingNative, ProductionTinySourcePermutationMatchesIndependentNative) {
    const TinyFixture fixture;
    const auto declaration = fixture.Prepare();
    const auto packed = packing_detail::Build(fixture.canonical, declaration, {});
    // Independent original NID rank: 10,20,30,40,50,60,70,80,90,100,110,120.
    const auto native = ObserveNative(12, {{9,1,8,2}}, {{8,2,7}}, {12,6,9,10,3,5,11,4});
    EXPECT_EQ(packed.master_rows, (std::vector<std::uint32_t>(native.order.begin(), native.order.end())));
    EXPECT_EQ(native.master_nodes, (std::vector<int>{1,2,7,8,9}));
    EXPECT_EQ(native.slave_nodes, (std::vector<int>{3,4,5,6,9,10,11,12}));
    EXPECT_TRUE(native.cleared);
}
} // namespace crash::modelio::tied_shell::test
