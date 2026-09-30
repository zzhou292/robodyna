#include "../Internal.h"
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>

namespace crash::modelio::type45 {
namespace {
std::string Card(std::initializer_list<std::uint64_t> nodes) {
    std::ostringstream out;
    for (auto node : nodes) out << std::setw(10) << node;
    return out.str();
}
detail::Evidence Source(std::string kind, std::uint64_t id, std::string card) {
    const auto header = Card({id});
    detail::Evidence result;
    result.block = {"yaris-coarse-v1l.key", "*CONSTRAINED_JOINT_" + kind + "_ID", {}, {}, 10, 12};
    result.block.raw_text = result.block.keyword + "\n" + header + "\n" + card + "\n";
    result.block.sha256 = output::Sha256(result.block.raw_text);
    result.cards = {{11, header}, {12, card}};
    return result;
}
std::vector<detail::Evidence> Sources() {
    return {Source("SPHERICAL", 100, Card({10, 20})), Source("REVOLUTE", 101, Card({20, 30, 40, 50})),
            Source("CYLINDRICAL", 102, Card({10, 30, 40, 50, 900, 901}))};
}
const std::vector<std::uint64_t> Ids{10, 20, 30, 40, 50};
const std::vector<double> Xyz{-0., 0, 0, .03, .02, 0, .04, .01, 0, .1, 0, 0, .03, .08, 0};
tl::fea::NodalNodeDomain Domain(bool zero_changed = false) {
    tl::fea::NodalDomainNode nodes[4]{};
    for (unsigned i = 0; i < 4; ++i) {
        const auto source = 3 - i;
        nodes[i] = {Ids[source], {Xyz[3*source], Xyz[3*source+1], Xyz[3*source+2]}};
    }
    if (zero_changed) nodes[3].position.x = 0.;
    tl::fea::NodalNodeDomain result;
    const auto report = result.Initialize({71, nodes, 4});
    EXPECT_TRUE(report);
    return result;
}
}
TEST(VehicleType45Fields,DirectImportDefaultsPreserveSourceBlanksAndUnusedCylindricalColumns) {
    Data data;
    data.rows = detail::Read(Sources(), {});
    detail::ResolveProperties(data);
    ASSERT_EQ(data.rows.size(), 3);
    for (unsigned kind = 0; kind < 3; ++kind) {
        const auto& p = data.properties[kind];
        EXPECT_EQ(p.origin_joint_id, 100 + kind);
        EXPECT_EQ(p.value.kind, static_cast<native::Kind>(kind + 1));
        EXPECT_EQ(output::Bits(p.value.automatic_stiffness_scale), output::Bits(.01));
        EXPECT_EQ(output::Bits(p.value.critical_damping_ratio), output::Bits(.05));
        EXPECT_EQ(p.value.free_stiffness.translation.x, 0);
        EXPECT_EQ(p.value.free_stiffness.rotation.x, 0);
        EXPECT_EQ(p.value.free_viscosity.translation.x, 0);
        EXPECT_EQ(p.value.free_viscosity.rotation.x, 0);
        EXPECT_EQ(data.rows[kind].blank_mask & 0xc0u, 0xc0u);
    }
    EXPECT_EQ(data.rows[2].source_node_count, 4);
    ASSERT_TRUE(data.rows[2].unused_columns[0]);
    EXPECT_EQ(*data.rows[2].unused_columns[0], 900);
    EXPECT_EQ(*data.rows[2].unused_columns[1], 901);
}
TEST(VehicleType45Fields,ExplicitZeroLateMalformedAndDuplicateDeclarationsPreservePriorRows) {
    auto rows = detail::Read(Sources(), {});
    auto changed = Sources();
    changed.back() = Source("CYLINDRICAL", 102, Card({10, 30, 40, 50, 900, 901, 0}));
    EXPECT_THROW(rows = detail::Read(changed, {}), std::runtime_error);
    EXPECT_EQ(rows.back().source_id, 102);
    changed = Sources(); changed.back() = Source("CYLINDRICAL", 101, Card({10, 30, 40, 50}));
    EXPECT_THROW(rows = detail::Read(changed, {}), std::runtime_error);
    changed = Sources(); changed.back().block.raw_text.back() = 'x';
    EXPECT_THROW(rows = detail::Read(changed, {}), std::runtime_error);
    Limits cap; cap.rows = 2;
    EXPECT_THROW(rows = detail::Read(Sources(), cap), std::runtime_error);
    EXPECT_NO_THROW(rows = detail::Read(Sources(), {}));
}
TEST(VehicleType45Fields,DomainOrderSignedZerosAxisAndSourceOnlyRolesRemainDistinct) {
    auto rows = detail::Read(Sources(), {});
    const std::vector<detail::BodyMember> members{
        {10, {BodyKind::PartRoot, 801, 0, true}}, {20, {BodyKind::PlainGroup, 802, 2, true}},
        {50, {BodyKind::PlainGroup, 803, 3, false}}};
    detail::Map(rows, Ids, Xyz, Domain(), members);
    EXPECT_EQ(rows[0].nodes[0].domain_index, 3);
    EXPECT_EQ(output::Bits(rows[0].nodes[0].position_m.x), output::Bits(-0.));
    EXPECT_EQ(rows[0].nodes[0].body.kind, BodyKind::PartRoot);
    EXPECT_EQ(rows[0].nodes[1].body.kind, BodyKind::PlainGroup);
    EXPECT_EQ(rows[1].nodes[2].use, NodeUse::InitialAxis);
    EXPECT_EQ(rows[1].nodes[3].use, NodeUse::OriginalEvidence);
    EXPECT_EQ(rows[1].nodes[3].domain_index, SIZE_MAX);
    EXPECT_FALSE(rows[1].nodes[3].body.retained);
    EXPECT_EQ(rows[0].Geometry().source_node_id[2], 0);
    EXPECT_EQ(rows[2].Geometry().source_node_id[2], 40);
}
TEST(VehicleType45Fields,LateCoordinateAndMembershipRejectionLeaveMappedRowsUnchanged) {
    auto rows = detail::Read(Sources(), {});
    detail::Map(rows, Ids, Xyz, Domain(), {});
    auto wrong = Xyz; wrong.back() = INFINITY;
    EXPECT_THROW(detail::Map(rows, Ids, wrong, Domain(), {}), std::runtime_error);
    EXPECT_EQ(rows.back().nodes[3].position_m.z, 0);
    EXPECT_THROW(detail::Map(rows, Ids, Xyz, Domain(true), {}), std::runtime_error);
    const std::vector<detail::BodyMember> duplicate{
        {10, {BodyKind::PlainGroup, 802, 0, true}}, {10, {BodyKind::PartRoot, 801, 0, true}}};
    EXPECT_THROW(detail::Map(rows, Ids, Xyz, Domain(), duplicate), std::runtime_error);
    EXPECT_EQ(output::Bits(rows[0].nodes[0].position_m.x), output::Bits(-0.));
    EXPECT_NO_THROW(detail::Map(rows, Ids, Xyz, Domain(), {}));
}
} // namespace crash::modelio::type45
