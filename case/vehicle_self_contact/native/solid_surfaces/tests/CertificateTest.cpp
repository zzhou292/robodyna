#include "Fixture.h"
namespace crash::cases::vehicle_self_contact::native::initial_surfaces::test {
TEST(InitialSurfaceCertificate, CompleteSameMembershipAcceptsButConflictingFirstMatchDoesNot) {
    for (const auto part : {std::uint64_t{10}, std::uint64_t{20}}) {
        auto c = leaf::SingleHex();
        c.quads = {{201, part, {0,1,2,3}}, {202, part, {3,2,1,0}}};
        Certificate proof;
        const auto report = Membership(c, proof);
        ASSERT_EQ(report.status, Status::Ready) << report.reason;
        EXPECT_TRUE(proof.membership_complete);
        EXPECT_EQ(proof.queried_solid_faces, 6u);
        EXPECT_EQ(proof.matching_physical_shells, 2u);
    }
    auto c = leaf::SingleHex();
    c.quads = {{201, 10, {0,1,2,3}}, {202, 20, {3,2,1,0}}};
    const Certificate prior{17, 19, 23, false, true};
    for (int order = 0; order != 2; ++order) {
        auto proof = prior;
        const auto report = Membership(c, proof);
        EXPECT_EQ(report.status, Status::NeedsNativeReaderOrder);
        EXPECT_EQ(report.solid_element, 101u);
        EXPECT_NE(report.first_candidate_element, report.conflicting_candidate_element);
        Same(proof, prior);
        std::reverse(c.quads.begin(), c.quads.end());
    }
}
TEST(InitialSurfaceCertificate, CancelledInternalFacesDoNotDemandUnusedSuppressionOrder) {
    auto c = leaf::Adjacent();
    c.quads = {{201, 10, {4,5,6,7}}, {202, 30, {7,6,5,4}}};
    Certificate proof;
    ASSERT_EQ(Membership(c, proof).status, Status::Ready);
    EXPECT_EQ(proof.queried_solid_faces, 10u);
    EXPECT_EQ(proof.matching_physical_shells, 0u);
    // With only the first solid selected that same face reaches suppression.
    c.parts = {10};
    Certificate required;
    EXPECT_EQ(Membership(c, required).status, Status::NeedsNativeReaderOrder);
}
TEST(InitialSurfaceCertificate, NativeMatchingFamiliesStaySeparate) {
    auto c = leaf::Case{};
    c.solids = {leaf::Penta()};
    c.triangles = {{201, 10, {0,1,2,2}}};
    // A Q4 containing all three triangle nodes is not a native SH3N match.
    c.quads = {{202, 20, {0,1,2,6}}};
    Certificate proof;
    ASSERT_EQ(Membership(c, proof).status, Status::Ready);
    EXPECT_EQ(proof.queried_solid_faces, 5u);
    EXPECT_EQ(proof.matching_physical_shells, 1u);
}
TEST(InitialSurfaceCertificate, EqualConsumedWordsRetainEveryOriginWithoutSelectingAnOwner) {
    auto c = leaf::Case{};
    c.quads = {{201, 10, {0,1,2,3}}, {202, 10, {0,1,2,3}}};
    Certificate proof;
    auto faces = Faces(c, proof);
    ASSERT_EQ(faces.size(), 2u);
    std::vector<OriginGroup> groups;
    ASSERT_EQ(detail::CertifyConsumerOrder(faces, proof, &groups).status, Status::Ready);
    EXPECT_EQ(proof.equal_node_key_groups, 1u);
    EXPECT_EQ(proof.differing_origin_groups, 1u);
    ASSERT_EQ(groups.size(), 1u);
    EXPECT_EQ(groups[0].first_face, 0u);
    EXPECT_EQ(groups[0].face_count, 2u);
    EXPECT_NE(faces[0].source.element_id, faces[1].source.element_id);
    EXPECT_NE(faces[0].source.source_line, faces[1].source.source_line);
    const auto before = proof;
    const auto prior_groups = groups;
    faces[1].raw_role = 1;
    EXPECT_EQ(detail::CertifyConsumerOrder(faces, proof, &groups).status, Status::NeedsNativeReaderOrder);
    Same(proof, before);
    ASSERT_EQ(groups.size(), prior_groups.size());
    EXPECT_EQ(groups[0].first_face, prior_groups[0].first_face);
    EXPECT_EQ(groups[0].face_count, prior_groups[0].face_count);
}
TEST(InitialSurfaceCertificate, RepresentativePermutationsPreserveTypedExternalFaces) {
    auto c = leaf::Adjacent();
    c.parts = {10,20,30};
    c.quads = {{201, 30, {12,13,14,15}}, {202, 30, {16,17,18,19}}};
    c.triangles = {{301, 30, {20,21,22,22}}};
    Certificate a, b;
    const auto original = Faces(c, a);
    ASSERT_EQ(detail::CertifyConsumerOrder(original, a).status, Status::Ready);
    std::reverse(c.solids.begin(), c.solids.end());
    std::reverse(c.quads.begin(), c.quads.end());
    const auto permuted = Faces(c, b);
    ASSERT_EQ(detail::CertifyConsumerOrder(permuted, b).status, Status::Ready);
    ASSERT_EQ(original.size(), permuted.size());
    for (std::size_t i = 0; i < original.size(); ++i) Same(original[i], permuted[i]);
    Same(a, b);
    for (const auto& face : original) {
        EXPECT_NE(face.source.element_id, 0u);
        if (face.source.kind == values::ParentKind::Solid) {
            EXPECT_GE(face.source.solid_face, 1u);
            EXPECT_LE(face.source.solid_face, 6u);
        } else {
            EXPECT_EQ(face.source.solid_face, 0u);
        }
    }
}
TEST(InitialSurfaceCertificate, PackingRejectsForeignMembershipPhaseAndFaceIdentity) {
    auto c = leaf::SingleHex();
    c.quads = {{201,10,{8,9,10,11}}};
    auto geometry = Geometry(c);
    geometry.shells[0].contact_selected = false;
    EXPECT_THROW(detail::Pack(geometry, c.parts), std::exception);
    geometry = Geometry(c);
    geometry.solids[0].phase = static_cast<coated::PacketPhase>(99);
    EXPECT_THROW(detail::Pack(geometry, c.parts), std::exception);
    geometry = Geometry(c);
    const auto packed = detail::Pack(geometry, c.parts);
    const Extracted actual(packed.Input(false));
    ASSERT_GT(actual.snapshot.face_count, 0u);
    auto face = actual.snapshot.faces[0];
    ++face.source.element_id;
    EXPECT_THROW(detail::ExternalFace(face, packed, geometry), std::exception);
    EXPECT_THROW(detail::Pack(geometry, {10,10}), std::exception);
    EXPECT_NO_THROW(detail::Pack(geometry, c.parts));
}
TEST(InitialSurfaceCertificate, ReaderShapeSelectsExplicitRawBrickWithoutChangingSourceSlots) {
    auto c = leaf::Case{};
    c.solids = {leaf::Hex(101), leaf::Penta(102,10,8),
        {103,10,values::SolidTopology::NativeRaw8,{16,17,18,19,20,20,21,21}}};
    const auto geometry = Geometry(c);
    const auto packed = detail::Pack(geometry, c.parts);
    ASSERT_EQ(packed.solids.size(), 3u);
    EXPECT_EQ(packed.solids[0].topology, values::SolidTopology::Hex8);
    EXPECT_EQ(packed.solids[1].topology, values::SolidTopology::DeclaredPenta6);
    EXPECT_EQ(packed.solids[2].topology, values::SolidTopology::NativeRaw8);
    EXPECT_EQ(geometry.solids[2].kind, coated::ReaderKind::Hex8);
    for (std::size_t i = 0; i < packed.solids.size(); ++i)
        for (unsigned k = 0; k < 8; ++k)
            EXPECT_EQ(packed.solids[i].nodes[k], geometry.solids[i].nodes[k]);
    const Extracted extraction(packed.Input(false));
    EXPECT_EQ(extraction.snapshot.face_count, 16u);
    EXPECT_EQ(extraction.snapshot.counts.degenerate_faces, 2u);
}

}
