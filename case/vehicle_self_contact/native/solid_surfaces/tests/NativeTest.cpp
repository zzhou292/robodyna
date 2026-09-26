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
    auto raw = leaf::SingleHex();
    raw.solids = {{601,10,values::SolidTopology::NativeRaw8,{0,1,2,3,4,4,5,5}}};
    raw.triangles = {{701,10,{0,1,4,4}}};
    cases.push_back(raw);
    for (auto c : cases) {
        for (int order = 0; order != 2; ++order) {
            const auto geometry = Geometry(c);
            const auto packed = detail::Pack(geometry, c.parts);
            Certificate proof;
            const auto actual = Faces(c, proof);
            ASSERT_EQ(detail::CertifyConsumerOrder(actual, proof).status, Status::Ready);
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
TEST(InitialSurfaceNative, ReorderedEqualRoleOriginsKeepConsumedFieldsAndBothNativeSolidTags) {
    auto c = leaf::SingleHex();
    c.solids.push_back({102,10,values::SolidTopology::NativeRaw8,{6,7,3,2,8,8,9,9}});
    std::vector<Face> previous;
    for (int order = 0; order != 2; ++order) {
        const auto geometry = Geometry(c);
        const auto packed = detail::Pack(geometry, c.parts);
        const auto native = leaf::Oracle(packed.Input(false));
        Certificate proof;
        auto faces = Faces(c, proof);
        std::vector<OriginGroup> groups;
        ASSERT_EQ(detail::CertifyConsumerOrder(faces, proof, &groups).status, Status::Ready);
        ASSERT_GT(proof.differing_origin_groups, 0u);
        ASSERT_EQ(native.surface_solid_flags, (std::vector<std::uint8_t>{1,1}));
        ASSERT_EQ(faces.size(), native.faces.size());
        for (std::size_t i = 0; i < faces.size(); ++i)
            Same(faces[i], detail::ExternalFace(native.faces[i], packed, geometry));
        bool saw_first = false, saw_second = false;
        for (const auto group : groups) {
            for (std::size_t i = 0; i < group.face_count; ++i) {
                const auto& face = faces[group.first_face+i];
                saw_first = saw_first || face.source.element_id == 101;
                saw_second = saw_second || face.source.element_id == 102;
            }
        }
        EXPECT_TRUE(saw_first && saw_second);
        if (order) {
            ASSERT_EQ(previous.size(), faces.size());
            for (std::size_t i = 0; i < faces.size(); ++i) {
                EXPECT_EQ(previous[i].nodes, faces[i].nodes);
                EXPECT_EQ(previous[i].raw_role, faces[i].raw_role);
            }
        }
        auto forged = native.faces.front();
        forged.source.element_id += 1000;
        EXPECT_THROW(detail::ExternalFace(forged, packed, geometry), std::exception);
        previous = faces;
        std::reverse(c.solids.begin(), c.solids.end());
    }
}

}
