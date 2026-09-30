#include "NativeOracle.h"
#include <gtest/gtest.h>
#include <algorithm>

namespace crash::modelio::tied_shell::test {
using search_detail::RankedCoefficient;
TEST(TiedSearchGeometryNative, BothOriginalLayerFamiliesRetainEquivalentWinnersInAllPermutations) {
    for (const auto family : {SearchShellFamily::Q4, SearchShellFamily::T3}) {
        const std::array<SourceId,4> nodes = family == SearchShellFamily::Q4 ?
            std::array<SourceId,4>{90,10,70,30} : std::array<SourceId,4>{90,10,70,70};
        const std::vector<RankedCoefficient> c{{.5,250,0,0,family}, {2.28,70000,0,0,family},
                                             {2.28,70000,0,0,family}};
        std::vector<bool> winners;
        const auto expected = search_detail::ResolveEquivalent(c, winners);
        std::vector<unsigned> order{0,1,2};
        do {
            const auto native = NativeGeometry(nodes, {nodes,nodes,nodes}, c, order);
            EXPECT_EQ(native.consumed[0], expected);
            EXPECT_EQ(native.consumed[1], expected);
            const auto winner = native.selected[family == SearchShellFamily::Q4 ? 0 : 1];
            ASSERT_GE(winner, 0);
            EXPECT_TRUE(winners[winner]);
            const auto a = std::find(order.begin(), order.end(), 1u);
            const auto b = std::find(order.begin(), order.end(), 2u);
            EXPECT_EQ(winner, int(family == SearchShellFamily::Q4 ? *(std::min(a,b)) : *(std::max(a,b))));
        } while (std::next_permutation(order.begin(), order.end()));
    }
}
TEST(TiedSearchGeometryNative, OriginalCallerPrecedenceAndCrossFamilyCounterexampleRemainExplicit) {
    const std::array<SourceId,4> q{1,2,3,4}, t{1,2,3,3};
    for (double part : {0.,-0.,8.}) {
        for (double element : {0.,-0.,7.}) {
            const std::vector<RankedCoefficient> c{{2,20,part,element}};
            const auto native = NativeGeometry(q, {q}, c, {0});
            EXPECT_EQ(native.consumed[0], search_detail::ConsumedThickness(c[0]));
            EXPECT_EQ(native.consumed[1], native.consumed[0]);
        }
    }
    // Shared native DXM allows both selected indices to survive. The callers
    // deliberately choose different families; the first app profile rejects.
    const std::vector<RankedCoefficient> mixed{{3,20,9,0,SearchShellFamily::Q4},
        {2,20,8,0,SearchShellFamily::T3}};
    const auto native = NativeGeometry(t, {q,t}, mixed, {0,1});
    EXPECT_EQ(native.selected[0], 0);
    EXPECT_EQ(native.selected[1], 1);
    EXPECT_EQ(native.consumed[0], 9);
    EXPECT_EQ(native.consumed[1], 8);
    std::vector<bool> prior{true};
    EXPECT_THROW(search_detail::ResolveEquivalent(mixed, prior), std::exception);
    EXPECT_EQ(prior, (std::vector<bool>{true}));
}
} // namespace crash::modelio::tied_shell::test
