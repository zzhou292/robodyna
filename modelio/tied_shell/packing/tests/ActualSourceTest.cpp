#include "../../TiedShellPacking.h"
#include "NativeOracle.h"
#include "output/full_shell/static_bundle/tests/ActualMappingSupport.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <iostream>
#include <type_traits>

namespace crash::modelio::tied_shell::test {
namespace {
const TiedShellDeclaration& Declaration() {
    static const auto value = [] {
        auto inputs = source::test::ActualInputs();
        inputs.scope_report.bytes = 13212691;
        inputs.scope_report.sha256 = "fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0";
        const auto canonical = source::CanonicalSource::Read(inputs);
        const auto member = output::ReadBounded(std::getenv("ROBO_STATIC_MEMBER"), 42846753);
        return TiedShellDeclaration::Prepare(canonical, member);
    }();
    return value;
}
template<class T> std::vector<T> Decode(const source::CanonicalData& source, const char* name) {
    const auto& array = source::FindArray(source, name);
    return output::arrays::Decode<T>(array.descriptor, array.bytes);
}
}
TEST(TiedPackingActual, CompleteOriginalPermutationMatchesIndependentNativeAndCensus) {
    const auto& declaration = Declaration();
    const auto& source = declaration.canonical().data();
    const auto& d = declaration.data();
    const auto forecast = TiedShellPacking::Forecast(declaration);
    std::cout << "Tied packing startup forecast: " << forecast << " bytes\n";
    const auto packed = TiedShellPacking::Prepare(declaration);
    ASSERT_EQ(packed.data().master_rows.size(), 171813u);
    ASSERT_EQ(d.slave_nodes.size(), 11165u);
    ASSERT_EQ(d.master_nodes.size(), 183457u);
    EXPECT_EQ(&packed.declaration().data(), &d);
    EXPECT_EQ(&packed.declaration().canonical().data(), &source);
    EXPECT_EQ(packed.data().startup_budget_bytes, forecast);
    EXPECT_EQ(d.search, NativeReadiness::Unresolved);
    EXPECT_EQ(d.classification, NativeReadiness::Unresolved);
    const auto records = Decode<SourceId>(source, "shells_records");
    const auto ids = Decode<SourceId>(source, "node_ids");
    auto ordered_ids = ids;
    std::sort(ordered_ids.begin(), ordered_ids.end());
    const auto native_node = [&](SourceId id) {
        const auto found = std::lower_bound(ordered_ids.begin(), ordered_ids.end(), id);
        output::Require(found != ordered_ids.end() && *found == id, "Missing source oracle node");
        return static_cast<int>(found-ordered_ids.begin()+1);
    };
    std::vector<std::array<int,4>> q4;
    std::vector<std::array<int,3>> t3;
    std::vector<std::uint32_t> native_to_declaration, triangles;
    for (std::size_t row = 0; row < d.masters.size(); ++row) {
        const auto& parent = d.masters[row];
        const auto* nodes = records.data()+6*parent.canonical_index+2;
        if (parent.arity == 4) {
            q4.push_back({native_node(nodes[0]),native_node(nodes[1]),native_node(nodes[2]),native_node(nodes[3])});
            native_to_declaration.push_back(row);
        } else {
            t3.push_back({native_node(nodes[0]),native_node(nodes[1]),native_node(nodes[2])});
            triangles.push_back(row);
        }
    }
    native_to_declaration.insert(native_to_declaration.end(), triangles.begin(), triangles.end());
    std::vector<int> slaves;
    for (auto i = d.slave_nodes.rbegin(); i != d.slave_nodes.rend(); ++i)
        slaves.push_back(native_node(i->id));
    const auto native = ObserveNative(ids.size(), q4, t3, slaves);
    ASSERT_EQ(native.order.size(), packed.data().master_rows.size());
    std::vector<SourceId> ordered_eids;
    for (std::size_t rank = 0; rank < native.order.size(); ++rank) {
        const auto row = packed.data().master_rows[rank];
        ASSERT_LT(native.order[rank], native_to_declaration.size());
        ASSERT_EQ(row, native_to_declaration[native.order[rank]]) << rank;
        ASSERT_EQ(packed.data().master_ranks[row], rank);
        const auto& parent = d.masters[row];
        const auto* nodes = records.data()+6*parent.canonical_index+2;
        for (unsigned local = 0; local < 4; ++local)
            ASSERT_EQ(native.rect[4*rank+local], native_node(nodes[local])) << rank << ':' << local;
        ordered_eids.push_back(parent.id);
    }
    ASSERT_EQ(native.master_nodes.size(), d.master_nodes.size());
    for (std::size_t i = 0; i < d.master_nodes.size(); ++i)
        ASSERT_EQ(native.master_nodes[i], native_node(ids[d.master_nodes[i]])) << i;
    ASSERT_EQ(native.slave_nodes.size(), d.slave_nodes.size());
    for (std::size_t i = 0; i < d.slave_nodes.size(); ++i)
        ASSERT_EQ(native.slave_nodes[i], native_node(d.slave_nodes[i].id)) << i;
    EXPECT_TRUE(native.cleared);
    const output::arrays::Layout layout{output::arrays::Scalar::UInt64, ordered_eids.size(), 1, {}};
    const auto bytes = output::arrays::Encode(layout, ordered_eids.data(), ordered_eids.size());
    EXPECT_EQ(output::Sha256(bytes), "8e57ad5fc8eee6c58a8bf8518c52813db0e0ce1ca6ecf444ba65ec650f3d5cc6");
    EXPECT_EQ(ordered_eids.front(), 2313681u);
    EXPECT_EQ(ordered_eids.back(), 2492895u);
    std::cout << "Tied packing retained payload: " << packed.data().owned_payload_bytes << " bytes\n";
}
TEST(TiedPackingActual, PublicFactoryPreservesLifetimeAndBudgetFailureRetry) {
    static_assert(!std::is_copy_assignable_v<TiedShellPacking>);
    const auto& declaration = Declaration();
    const auto prior = TiedShellPacking::Prepare(declaration);
    auto limits = PackingLimits{};
    limits.host_bytes = TiedShellPacking::Forecast(declaration)-1;
    EXPECT_THROW(TiedShellPacking::Prepare(declaration, limits), std::exception);
    limits = {};
    limits.masters = 171812;
    EXPECT_THROW(TiedShellPacking::Prepare(declaration, limits), std::exception);
    const auto retry = [&] {
        auto local = declaration;
        return TiedShellPacking::Prepare(local);
    }();
    EXPECT_EQ(prior.data().master_rows, retry.data().master_rows);
    EXPECT_EQ(prior.data().master_ranks, retry.data().master_ranks);
    EXPECT_EQ(&retry.declaration().data(), &declaration.data());
    EXPECT_EQ(retry.declaration().data().slaves.back().id, 2201691u);
}
} // namespace crash::modelio::tied_shell::test
