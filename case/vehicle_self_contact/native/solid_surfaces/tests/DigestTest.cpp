#include "Fixture.h"
namespace crash::cases::vehicle_self_contact::native::initial_surfaces::test {
TEST(InitialSurfaceDigest, InputBindsSourceUnitsCoordinatesTopologyAndSelection) {
    auto c = leaf::SingleHex();
    c.quads = {{201,10,{8,9,10,11}}};
    const auto original = Geometry(c);
    const auto packed = detail::Pack(original, c.parts);
    Provenance provenance;
    provenance.source_digest = std::string(64, 'a');
    provenance.selection_digest = std::string(64, 'b');
    provenance.control_rule = "reviewed-rule";
    const auto digest = detail::InputDigest(original, packed, provenance, 1u<<20);
    ASSERT_EQ(digest.size(), 64u);
    EXPECT_EQ(detail::InputDigest(original, packed, provenance, 1u<<20), digest);
    // Hash sensitivity only. These modifications do not assert source admission.
    for (unsigned field = 0; field != 9; ++field) {
        auto modified = original;
        auto binding = provenance;
        auto selection = packed;
        if (field == 0) modified.nodes[0].native_position.y = -0.0;
        if (field == 1) ++modified.nodes[0].source_id;
        if (field == 2) modified.units.length_m = .01;
        if (field == 3) ++modified.solids[0].nodes[7];
        if (field == 4) ++modified.shells[0].primary.source_id;
        if (field == 5) ++modified.shells[0].part_id;
        if (field == 6) ++selection.selected_parts[0];
        if (field == 7) binding.source_digest[0] = 'c';
        if (field == 8) binding.control_rule += "-changed";
        EXPECT_NE(detail::InputDigest(modified, selection, binding, 1u<<20), digest) << field;
    }
}
TEST(InitialSurfaceDigest, OutputBindsExternalOwnershipFlagsAndCertificateWithoutPadding) {
    const auto c = leaf::SingleHex();
    Certificate certificate;
    const auto original = Faces(c, certificate);
    ASSERT_EQ(detail::CertifyOrder(original, certificate).status, Status::Ready);
    Provenance provenance;
    provenance.input_digest = std::string(64, 'a');
    const std::vector<std::uint8_t> flags{1};
    const auto digest = detail::OutputDigest(original, flags, certificate, provenance, 1u<<20);
    for (unsigned field = 0; field != 9; ++field) {
        auto faces = original;
        auto changed_flags = flags;
        auto proof = certificate;
        if (field == 0) ++faces[0].source.element_id;
        if (field == 1) ++faces[0].source.part_id;
        if (field == 2) ++faces[0].source.solid_face;
        if (field == 3) ++faces[0].source.canonical_row;
        if (field == 4) ++faces[0].source.source_line;
        if (field == 5) ++faces[0].nodes[0];
        if (field == 6) ++faces[0].raw_role;
        if (field == 7) changed_flags[0] = 0;
        if (field == 8) proof.sort_order_complete = false;
        EXPECT_NE(detail::OutputDigest(faces, changed_flags, proof, provenance, 1u<<20), digest) << field;
    }
}
}
