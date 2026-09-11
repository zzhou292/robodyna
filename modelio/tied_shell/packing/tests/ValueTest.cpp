#include "../Internal.h"
#include "../../tests/TinyFixture.h"
#include <numeric>

namespace crash::modelio::tied_shell::test {
namespace {
PackingData Pack(const TinyFixture& fixture, const Data& declaration, PackingLimits limits = {}) {
    return packing_detail::Build(fixture.canonical, declaration, limits);
}
void RewriteShells(TinyFixture& fixture, const std::vector<SourceId>& values) {
    auto& array = fixture.canonical.arrays[2];
    array.bytes = output::arrays::Encode(array.descriptor.layout, values.data(), values.size());
    array.descriptor.sha256 = output::Sha256(array.bytes);
}
}
TEST(TiedPackingValues, NativeTupleOrderDiffersFromSourceAndRetainsTrueTriangle) {
    const TinyFixture fixture;
    const auto declaration = fixture.Prepare();
    const auto value = Pack(fixture, declaration);
    EXPECT_EQ(value.master_rows, (std::vector<std::uint32_t>{1,0}));
    EXPECT_EQ(value.master_ranks, (std::vector<std::uint32_t>{1,0}));
    EXPECT_EQ(declaration.masters[value.master_rows[0]].arity, 3u);
    EXPECT_EQ(declaration.slave_nodes.front().id, 30u);
    EXPECT_EQ(declaration.slave_nodes.back().id, 120u);
    EXPECT_EQ(declaration.search, NativeReadiness::Unresolved);
    EXPECT_EQ(declaration.classification, NativeReadiness::Unresolved);
    EXPECT_EQ(value.owned_payload_bytes, sizeof(PackingData)+4*sizeof(std::uint32_t));
}
TEST(TiedPackingValues, RejectsUnsupportedOptionsWithoutChangingDeclaration) {
    TinyFixture fixture(true);
    const auto declaration = fixture.Prepare();
    EXPECT_THROW(Pack(fixture, declaration), std::exception);
    fixture = TinyFixture{};
    auto d = fixture.Prepare();
    d.sources[d.contact_source].cards[1].second = "         1";
    EXPECT_THROW(Pack(fixture, d), std::exception);
    EXPECT_EQ(d.slave_nodes.back().id, 120u);
    EXPECT_EQ(Pack(fixture, fixture.Prepare()).master_rows[0], 1u);
}
TEST(TiedPackingValues, EqualTupleNeedsNativeRegistrationNotElementIdFallback) {
    TinyFixture fixture;
    auto d = fixture.Prepare();
    const auto prior = Pack(fixture, d);
    RewriteShells(fixture, {5010,100,90,10,80,20, 5011,101,90,10,80,20});
    d.masters.back().arity = 4;
    try {
        Pack(fixture, d);
        FAIL();
    } catch (const std::exception& error) {
        const std::string message = error.what();
        EXPECT_NE(message.find("native registration"), std::string::npos);
        EXPECT_NE(message.find("5010"), std::string::npos);
        EXPECT_NE(message.find("5011"), std::string::npos);
    }
    EXPECT_EQ(prior.master_rows, (std::vector<std::uint32_t>{1,0}));
    fixture = TinyFixture{};
    EXPECT_EQ(Pack(fixture, fixture.Prepare()).master_rows, prior.master_rows);
}
TEST(TiedPackingValues, LateIdentityTopologyAndNodeOrderRejectThenRetry) {
    TinyFixture fixture;
    const auto d = fixture.Prepare();
    const auto prior = Pack(fixture, d);
    auto broken = d;
    ++broken.masters.back().id;
    EXPECT_THROW(Pack(fixture, broken), std::exception);
    broken = d;
    std::swap(broken.master_nodes[0], broken.master_nodes[1]);
    EXPECT_THROW(Pack(fixture, broken), std::exception);
    broken = d;
    ++broken.slave_nodes.back().id;
    EXPECT_THROW(Pack(fixture, broken), std::exception);
    RewriteShells(fixture, {5010,100,90,10,80,20, 5011,101,80,80,70,70});
    EXPECT_THROW(Pack(fixture, d), std::exception);
    fixture = TinyFixture{};
    EXPECT_EQ(Pack(fixture, d).master_rows, prior.master_rows);
}
TEST(TiedPackingValues, CountAndByteAdmissionPrecedesBorrowedDecode) {
    TinyFixture fixture;
    const auto d = fixture.Prepare();
    const auto budget = packing_detail::Preflight(fixture.canonical, d, {});
    auto limits = PackingLimits{};
    limits.host_bytes = budget-1;
    fixture.canonical.arrays[2].bytes.back() ^= 1;
    try {
        Pack(fixture, d, limits);
        FAIL();
    } catch (const std::exception& error) {
        EXPECT_NE(std::string(error.what()).find("host byte cap"), std::string::npos);
    }
    limits = {};
    limits.masters = 1;
    EXPECT_THROW(Pack(fixture, d, limits), std::exception);
    limits = {};
    ++limits.nodes;
    EXPECT_THROW(Pack(fixture, d, limits), std::exception);
    EXPECT_THROW(Pack(fixture, d), std::exception); // The late source-byte hash is still wrong.
    fixture = TinyFixture{};
    EXPECT_EQ(Pack(fixture, d).startup_budget_bytes, budget);
}
} // namespace crash::modelio::tied_shell::test
