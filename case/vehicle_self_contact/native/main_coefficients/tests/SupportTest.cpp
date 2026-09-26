#include "Fixture.h"
#include "lib_utest/qualification/radioss_type25_coated_coefficients/NativeOracle.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
TEST(MainShellSupport, ThicknessThenModulusAndAllEqualWinnersMatchOriginalSource) {
    for (const bool triangle : {false, true}) {
        Fixture f(triangle);
        f.packed.parts[0].coefficient.property_thickness = .5;
        f.packed.parts[0].coefficient.young = 1000.; // E*T would incorrectly select this row.
        f.packed.parts[1].coefficient.young = 20.;
        auto selected = f.Select();
        ASSERT_EQ(selected.owner, 1u);
        EXPECT_EQ(selected.proof, OwnerProof::UniqueBest);
        EXPECT_EQ(f.Oracle({2,0,1}), selected.owner);
        f.packed.parts[2].coefficient.young = 20.;
        selected = f.Select();
        ASSERT_EQ(selected.winners.size(), 2u);
        EXPECT_EQ(selected.owner, triangle ? 2u : 1u);
        EXPECT_EQ(selected.proof, OwnerProof::NativeMaterialGroupOrder);
        EXPECT_EQ(f.Oracle(StorageOrder(f)), selected.owner);
    }
}
TEST(MainShellSupport, CornerMajorPrecedenceOverridesElementStorageOrder) {
    for (const bool triangle : {false, true}) {
        Fixture f(triangle);
        // Move the common first node to another original corner of row1.
        auto& nodes = f.input.shells[1].primary.nodes;
        if (triangle) { nodes[0]=1; nodes[1]=2; nodes[2]=0; nodes[3]=0; }
        else { nodes[0]=1; nodes[1]=2; nodes[2]=3; nodes[3]=0; }
        f.packed.parts[0].coefficient.young = 1.;
        f.Refresh();
        const auto selected = f.Select(false);
        ASSERT_EQ(selected.proof, OwnerProof::NativeCornerOrder);
        EXPECT_EQ(selected.owner, triangle ? 1u : 2u);
        for (const auto order : {std::vector<unsigned>{0,1,2}, std::vector<unsigned>{2,1,0},
                std::vector<unsigned>{1,0,2}}) EXPECT_EQ(f.Oracle(order), selected.owner);
    }
}
TEST(MainShellSupport, ExactTiesAndAdjacentDoublesPreserveOppositeQ4T3Policies) {
    for (const bool triangle : {false, true}) {
        Fixture f(triangle);
        EXPECT_EQ(f.Oracle({0,1,2}), triangle ? 2u : 0u);
        EXPECT_EQ(f.Oracle({2,1,0}), triangle ? 0u : 2u);
        f.packed.parts[1].coefficient.young = std::nextafter(10., 11.);
        EXPECT_EQ(f.Select(false).owner, 1u);
        EXPECT_EQ(f.Oracle({2,0,1}), 1u);
        f.packed.parts[2].coefficient.property_thickness = std::nextafter(1., 2.);
        EXPECT_EQ(f.Select(false).owner, 2u);
        EXPECT_EQ(f.Oracle({1,0,2}), 2u);
    }
}
TEST(MainShellSupport, GroupOrderRequiresIndependentContextAndPreservedMaterialIds) {
    Fixture f;
    EXPECT_EQ(f.Select().proof, OwnerProof::NativeMaterialGroupOrder);
    EXPECT_EQ(f.Select(false).owner, SIZE_MAX);
    f.packed.parts[1].native_material_id_preserved = false;
    EXPECT_EQ(f.Select().owner, SIZE_MAX);
    f.packed.parts[1].native_material_id_preserved = true;
    f.packed.parts[1].mid = f.packed.parts[0].mid;
    EXPECT_EQ(f.Select().owner, SIZE_MAX);
    f.packed.parts[1].mid = 21;
    f.packed.parts[1].ordinary_part_controls = false; // e.g. nonblank HGID.
    EXPECT_EQ(f.Select().owner, SIZE_MAX);
    f.packed.parts[1].ordinary_part_controls = true;
    f.sections[1].cards[0].values[3] = 5.;
    EXPECT_EQ(f.Select().owner, SIZE_MAX);
    f.sections[1].cards[0].values[3] = 3.;
    f.materials[1].cards[0].values[3] = .31;
    EXPECT_EQ(f.Select().owner, SIZE_MAX);
    EXPECT_EQ(f.Select().winners.size(), 3u); // No layer removed to force readiness.
}
TEST(MainShellSupport, EqualValuesNeverChooseAnUncertifiedOwner) {
    Fixture f;
    const auto support = f.Select(false);
    ASSERT_EQ(support.owner, SIZE_MAX);
    ASSERT_EQ(support.winners.size(), 3u);
    const auto first = detail::EvaluateValues(f.packed.parts[0].coefficient, n::ShellLayout::Quad4, nullptr);
    for (const auto i : support.winners)
        EXPECT_TRUE(detail::SameValues(first, detail::EvaluateValues(f.packed.parts[i].coefficient,
            n::ShellLayout::Quad4, nullptr)));
    auto changed = first;
    changed.partner = std::nextafter(changed.partner, 100.);
    EXPECT_FALSE(detail::SameValues(first, changed));
    changed = first;
    changed.characteristic_length = -0.;
    EXPECT_FALSE(detail::SameValues(first, changed));
}
TEST(MainShellSupport, SignedSolidSupportAndEncodedPartnerUseQualifiedNativeBlocks) {
    for (const auto layout : {n::ShellLayout::Quad4, n::ShellLayout::Triangle3}) {
        Fixture f(layout == n::ShellLayout::Triangle3);
        for (const double volume : {8., -8.}) {
            n::NativeSolidMainCoefficientInput solid;
            solid.face = n::MainFaceKind::OrdinaryExterior;
            solid.layout = n::SolidLayout::EightSlot;
            solid.area = 4.; solid.volume = volume; solid.bulk = 80.; solid.controlled_bulk = 160.;
            for (const int control : {0,1}) {
                solid.incompressibility_control = control;
                const auto value = detail::EvaluateValues(f.packed.parts[0].coefficient, layout, &solid);
                auto shell = f.packed.parts[0].coefficient;
                shell.face = n::MainFaceKind::Coating; shell.layout = layout;
                const auto native = coated_coefficient_test::Oracle({shell, solid});
                EXPECT_TRUE(tl::math::SameScalarBits(value.primary, native.primary_stiffness));
                EXPECT_TRUE(tl::math::SameScalarBits(value.partner, native.partner_stiffness));
                EXPECT_TRUE(tl::math::SameScalarBits(value.characteristic_length, native.solid_characteristic_length));
            }
        }
    }
}
TEST(MainShellSupport, IdentityCertificateRejectsRealPermutationWithoutChangingTopology) {
    Fixture f;
    const auto main = f.Main();
    n::NativeExteriorMainGeometryResult geometry;
    for (unsigned i=0; i<4; ++i) geometry.source_corner[i]=i;
    EXPECT_NO_THROW(detail::CheckIdentity(main, geometry, 0, 1000));
    geometry.source_corner[0]=1; geometry.source_corner[1]=0;
    EXPECT_THROW(detail::CheckIdentity(main, geometry, 0, 1000), detail::Failure);
    EXPECT_EQ(main.nodes[0], 0u);
    EXPECT_EQ(main.nodes[1], 1u);
    Fixture tri(true);
    const auto triangle=tri.Main();
    geometry.source_corner[0]=0; geometry.source_corner[1]=1; geometry.source_corner[2]=2; geometry.source_corner[3]=2;
    EXPECT_NO_THROW(detail::CheckIdentity(triangle, geometry, 0, 1000));
}
TEST(MainShellSupport, OriginalUnsignedOrderUsesAllEightKeyWords) {
    std::vector<std::array<std::uint32_t,8>> keys{{0,0,4,8,20,100,0,0}, {0,0,4,8,19,500,0,0},
        {0,0,4,8,20,1,0,0}, {0,1,0,0,1,1,0,0}, {0,0,4,UINT32_C(0x80000000),1,1,0,0}};
    EXPECT_EQ(NativeOrder(keys), (std::vector<unsigned>{1,2,0,4,3}));
}
TEST(MainShellSupport, GenuineStarterUsesContactParentAndRetainsSeparateMechanicalSupport) {
    Fixture f;
    f.packed.parts[0].coefficient.property_thickness = .5;
    f.packed.parts[0].coefficient.young = 1.;
    auto classified = coated::Classify(f.input);
    ASSERT_TRUE(classified.contact_complete);
    const auto order = coated::SurfaceOrder(f.input, classified);
    ASSERT_EQ(order.primary_to_physical.size(), 1u);
    coated::detail::TopologyInput packed(f.input, classified, order);
    const auto forecast = coated::s::Preflight(packed.View(), {});
    ASSERT_EQ(forecast.status, coated::s::Status::Ok);
    tl::util::HostArena output, scratch;
    ASSERT_TRUE(output.Initialize(forecast.output_bytes));
    ASSERT_TRUE(scratch.Initialize(forecast.scratch_bytes));
    coated::s::Snapshot snapshot;
    ASSERT_EQ(coated::s::BuildStarter(packed.View(), {}, output, scratch, &snapshot).status, coated::s::Status::Ok);
    const auto support = detail::SelectSupport(f.input, f.packed, snapshot.mains[0], 0, true);
    ASSERT_EQ(support.owner, 1u);
    EXPECT_EQ(snapshot.mains[0].source_id, 1000u);
    EXPECT_EQ(f.input.shells[support.owner].primary.source_id, 1001u);
    EXPECT_EQ(snapshot.primary_to_partner[0], 2u);
    EXPECT_EQ(snapshot.mains[1].source_id, 1000u);
}

TEST(MainShellSupport, MixedFamilyOracleReportsBothMatchesAndNativeT3Precedence) {
    NativeCandidate quad;
    quad.layout = n::ShellLayout::Quad4;
    quad.nodes = {0,1,2,3}; quad.thickness = 2.; quad.young = 40.;
    NativeCandidate tri;
    tri.layout = n::ShellLayout::Triangle3;
    tri.nodes = {0,1,2,2}; tri.thickness = 1.; tri.young = 10.;
    const auto result = NativeSupport({quad,tri}, tri.nodes);
    EXPECT_EQ(result.q4, 0u);
    EXPECT_EQ(result.t3, 1u);
    EXPECT_EQ(result.selected, 1u); // I25GAPM prioritizes nonzero NELTG.
}
TEST(MainShellSupport, ActualGeometryOutcomesDriveTheNoPermutationCertificate) {
    Fixture f;
    const auto main = f.Main();
    n::NativeExteriorMainGeometryInput packet;
    packet.layout = n::ShellLayout::Quad4;
    for (unsigned i = 0; i < 4; ++i) packet.face[i] = f.input.nodes[main.nodes[i]].native_position;
    unsigned identities = 0, reversals = 0;
    for (const double z : {-1., 1.}) {
        for (unsigned i = 0; i < 4; ++i) {
            packet.solid_raw[i] = packet.face[i];
            packet.solid_raw[i+4] = packet.face[i];
            packet.solid_raw[i+4].z = z;
        }
        n::NativeExteriorMainGeometryResult result;
        ASSERT_EQ(n::EvaluateNativeExteriorMainGeometry(packet, &result), n::CoefficientStatus::Ok);
        if (result.reversed) {
            ++reversals;
            EXPECT_THROW(detail::CheckIdentity(main, result, 0, 1000), detail::Failure);
        } else {
            ++identities;
            EXPECT_NO_THROW(detail::CheckIdentity(main, result, 0, 1000));
        }
    }
    EXPECT_EQ(identities, 1u);
    EXPECT_EQ(reversals, 1u);
}

}
