#include "Fixture.h"
namespace crash::cases::vehicle_self_contact::native::mixed_interface::test {
TEST(MixedInterfaceValues, CompleteNativeChainUsesPhysicalContextAndRealSideCounts) {
    for (auto input : {leaf::SolidOnly(), leaf::Mixed(), leaf::Mixed(true), leaf::Origins()}) {
        Values value(input);
        ASSERT_EQ(value.Certify().status, Status::Ready);
        value.Expand();
        const auto native = leaf::Oracle(value.packed.Input());
        leaf::Same(value.classified, native);
        leaf::Same(value.sides, native);
        EXPECT_EQ(value.sides.main_count, value.sides.primary_count+value.sides.shell_primary_count);
        EXPECT_TRUE(value.certificate.complete_origins);
        EXPECT_TRUE(value.certificate.complete_solid_flags);
        EXPECT_EQ(value.observations.size(), input.raw.size());
        for (std::size_t p = 0; p < value.sides.primary_count; ++p)
            EXPECT_EQ(value.sides.primary_to_partner[p] == 0,
                value.sides.primary_identities[p].kind == s::PrimaryFaceKind::Solid);
    }
}
TEST(MixedInterfaceValues, AmbiguousFirstSolidRemainsExplicitAndPreservesCertificate) {
    Values value(leaf::FirstSolid());
    ASSERT_FALSE(value.roles.contact_complete);
    value.certificate.raw_shells = 73;
    value.observations = {{91, 101}};
    const auto report = value.Certify();
    EXPECT_EQ(report.status, Status::NeedsNativeReaderOrder);
    EXPECT_EQ(report.source_element, 201u);
    EXPECT_EQ(value.certificate.raw_shells, 73u);
    ASSERT_EQ(value.observations.size(), 1u);
    EXPECT_EQ(value.observations[0].native_role, 91);
    // Native first-solid behavior really differs for this declared geometry.
    auto reversed = leaf::FirstSolid();
    std::reverse(reversed.physical.solids.begin(), reversed.physical.solids.end());
    const auto a = leaf::Oracle(leaf::FirstSolid().Input());
    const auto b = leaf::Oracle(reversed.Input());
    ASSERT_EQ(a.classifications.size(), 1u);
    ASSERT_EQ(b.classifications.size(), 1u);
    EXPECT_EQ(a.classifications[0].role, -b.classifications[0].role);
}
TEST(MixedInterfaceValues, ForgedSourceLocationRoleAndLostOriginAreRejected) {
    Values value(leaf::Mixed());
    ASSERT_EQ(value.Certify().status, Status::Ready);
    const auto prior = value.certificate;
    const auto observed = value.observations;
    auto forged = value.faces;
    ++forged.back().source.source_line;
    EXPECT_THROW(detail::Pack(value.geometry, value.packed.physical.selected_parts, forged), std::exception);
    auto roles = value.roles;
    for (auto& role : roles.roles) if (role.matches == 1) role.state = coated::RoleState::Ordinary;
    EXPECT_EQ(detail::Certify(value.geometry, value.packed, roles, value.flags, value.classified,
        value.certificate, value.observations).status, Status::InvalidInput);
    auto lost = value.classified;
    --lost.raw_face_count;
    EXPECT_EQ(detail::Certify(value.geometry, value.packed, value.roles, value.flags, lost,
        value.certificate, value.observations).status, Status::InvalidInput);
    auto undefined = value.roles;
    const auto row = std::size_t{0}; // Mixed fixture's first shell is the unique coating.
    ASSERT_EQ(undefined.roles[row].matches, 1u);
    undefined.contact_complete = false;
    undefined.first_unready_contact_shell = row;
    EXPECT_EQ(detail::SelectedRoleReport(value.geometry, undefined).status, Status::UnsupportedSource);
    auto flags = value.flags;
    flags[0] ^= 1;
    EXPECT_EQ(detail::Certify(value.geometry, value.packed, value.roles, flags, value.classified,
        value.certificate, value.observations).status, Status::InvalidInput);
    EXPECT_EQ(value.certificate.raw_shells, prior.raw_shells);
    EXPECT_EQ(value.certificate.complete_origins, prior.complete_origins);
    ASSERT_EQ(value.observations.size(), observed.size());
    for (std::size_t i = 0; i < observed.size(); ++i) {
        EXPECT_EQ(value.observations[i].native_role, observed[i].native_role);
        EXPECT_EQ(value.observations[i].unique_coating_solid_eid, observed[i].unique_coating_solid_eid);
    }
}
TEST(MixedInterfaceValues, MultipleOriginsRemainUnavailableAsSingleOwnerAndDigestBindsThem) {
    Values value(leaf::Origins());
    ASSERT_EQ(value.Certify().status, Status::Ready);
    value.Expand();
    EXPECT_GT(value.certificate.multi_origin_primaries, 0u);
    EXPECT_GT(value.certificate.coalesced_origins, 0u);
    Provenance provenance;
    provenance.source_digest = std::string(64, 'a');
    provenance.initial_digest = std::string(64, 'b');
    const auto original = detail::Digest(provenance, value.classified, value.sides, value.observations, value.certificate, 1u<<20);
    auto changed = value.sides;
    std::vector<s::PrimaryFaceIdentity> origins(changed.raw_origins, changed.raw_origins+changed.raw_origin_count);
    ++origins.front().physical_parent_id;
    changed.raw_origins = origins.data();
    EXPECT_NE(original, detail::Digest(provenance, value.classified, changed, value.observations, value.certificate, 1u<<20));
    for (std::size_t p = 0; p < value.sides.primary_count; ++p) {
        const auto& identity = value.sides.primary_identities[p];
        if (identity.origin == s::PrimaryOrigin::MultipleOrigins) {
            EXPECT_EQ(identity.physical_parent_id, 0u);
            EXPECT_EQ(value.classified.primary[p].source_id, 0u);
            EXPECT_GT(identity.origin_count, 1u);
        }
    }
}
TEST(MixedInterfaceValues, ReadyMetadataNeverClaimsNormalsSupportOrRuntime) {
    Preparation result;
    const auto document = ResultDocument(result);
    EXPECT_STREQ(document["status"].GetString(), "invalid_input");
    EXPECT_NE(std::string(document["scope"].GetString()).find("no support"), std::string::npos);
    EXPECT_THROW(ResultDocument(result, 1), std::exception);
    result.report.status = Status::Ready;
    EXPECT_THROW(ResultDocument(result), std::exception);
}
}
