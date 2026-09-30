#include "../Internal.h"
#include <gtest/gtest.h>

namespace crash::cases::vehicle_self_contact::native::nodal_seed::test {
namespace {
Provenance Binding() {
    Provenance result;
    result.canonical_sha256 = std::string(64, 'a');
    result.import_source_digest = std::string(64, 'b');
    result.spring_mapping_digest = std::string(64, 'c');
    result.units = {.001, 1000., 1.};
    return result;
}
detail::Packed Packet() {
    detail::Packed result;
    Contributor row;
    row.kind = ContributorKind::Type45;
    row.channel = Channel::DirectStiffness;
    row.original_id = 12;
    row.native_id = 99;
    row.slots = 2;
    row.nodes[0] = 0;
    row.nodes[1] = 1;
    result.contributors.push_back(row);
    result.stiffness = {{0, 0.}, {1, 0.}};
    return result;
}
}
TEST(ContactSeedDigest, SourceNativeIdentityUnitsAndZeroOccurrenceOrderAreBound) {
    const auto good = Packet();
    const auto source = Binding();
    const auto digest = detail::ContributorDigest(good, source, 1u << 20);
    auto changed = good;
    changed.contributors[0].native_id = 100;
    EXPECT_NE(detail::ContributorDigest(changed, source, 1u << 20), digest);
    changed = good;
    std::swap(changed.stiffness[0], changed.stiffness[1]);
    EXPECT_NE(detail::ContributorDigest(changed, source, 1u << 20), digest);
    auto units = source;
    units.units.length_m = .01;
    EXPECT_NE(detail::ContributorDigest(good, units, 1u << 20), digest);
    EXPECT_EQ(detail::ContributorDigest(good, source, 1u << 20), digest);
}
TEST(ContactSeedDigest, SignedZeroAndLaterChunkAreNotLost) {
    auto good = Packet();
    good.stiffness.resize(65537, {0, 0.});
    const auto digest = detail::ContributorDigest(good, Binding(), 1u << 20);
    good.stiffness.back().stiffness = -0.;
    EXPECT_NE(detail::ContributorDigest(good, Binding(), 1u << 20), digest);
    EXPECT_THROW(detail::ContributorDigest(good, Binding(), 1), std::exception);
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::test
