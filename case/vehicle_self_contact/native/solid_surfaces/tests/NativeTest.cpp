#include "Fixture.h"
#include "lib_utest/qualification/radioss_type25_surface_source/NativeOracle.h"
namespace crash::cases::vehicle_self_contact::native::initial_surfaces::test {
TEST(InitialSurfaceNative, CertifiedExternalRosterMatchesCompleteDonorForBothTraversals) {
    std::vector<leaf::Case> cases;
    auto hex = leaf::SingleHex();
    hex.quads = {{201,10,{0,1,2,3}}, {202,10,{8,9,10,11}}};
    cases.push_back(hex);
    auto penta = leaf::Case{};
    penta.solids = {leaf::Penta()};
    penta.triangles = {{301,10,{0,1,2,2}}, {302,20,{12,13,14,14}}};
    cases.push_back(penta);
    auto adjacent = leaf::Adjacent();
    adjacent.quads = {{201,10,{4,5,6,7}}, {202,30,{7,6,5,4}}};
    cases.push_back(adjacent);
    for (auto c : cases) {
        for (int order = 0; order != 2; ++order) {
            const auto geometry = Geometry(c);
            const auto packed = detail::Pack(geometry, c.parts);
            Certificate proof;
            const auto actual = Faces(c, proof);
            ASSERT_EQ(detail::CertifyOrder(actual, proof).status, Status::Ready);
            const auto native = leaf::Oracle(packed.Input(false));
            ASSERT_EQ(actual.size(), native.faces.size());
            for (std::size_t i = 0; i < actual.size(); ++i)
                Same(actual[i], detail::ExternalFace(native.faces[i], packed, geometry));
            const Extracted numerical(packed.Input(false));
            ASSERT_EQ(native.surface_solid_flags.size(), numerical.snapshot.solid_count);
            for (std::size_t i = 0; i < native.surface_solid_flags.size(); ++i)
                EXPECT_EQ(native.surface_solid_flags[i], numerical.snapshot.surface_solid_flags[i]);
            std::reverse(c.solids.begin(), c.solids.end());
            std::reverse(c.quads.begin(), c.quads.end());
            std::reverse(c.triangles.begin(), c.triangles.end());
        }
    }
}
TEST(InitialSurfaceNative, ConflictingMembershipHasAnActualOrderDependentNativeResult) {
    auto c = leaf::SingleHex();
    c.quads = {{201,10,{0,1,2,3}}, {202,20,{0,1,2,3}}};
    std::size_t counts[2]{};
    for (int order = 0; order != 2; ++order) {
        Certificate proof;
        EXPECT_EQ(Membership(c, proof).status, Status::NeedsNativeReaderOrder);
        const auto geometry = Geometry(c);
        const auto packed = detail::Pack(geometry, c.parts);
        counts[order] = leaf::Oracle(packed.Input(false)).faces.size();
        std::reverse(c.quads.begin(), c.quads.end());
    }
    EXPECT_NE(counts[0], counts[1]);
}
}
