#include "../VehicleType45Source.h"
#include "modelio/physical_scope/tests/ActualSupport.h"
#include "modelio/vehicle_source/SourceCards.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>

namespace crash::modelio::type45 {
namespace {
const physical_domain::VehiclePhysicalDomain& Domain() {
    static const auto result = physical_domain::VehiclePhysicalDomain::Prepare(
        physical_scope::test::Actual(), physical_domain::Policy::RetainedShellAssembliesV1);
    return result;
}
const VehicleType45Source& Actual() {
    static const auto result = [] {
        const auto forecast = VehicleType45Source::Preflight(Domain(), Policy::OriginalDirectSdiType45V1);
        std::cout << "VehicleType45 complete forecast=" << forecast.total_bytes << std::endl;
        return VehicleType45Source::Prepare(Domain(), Policy::OriginalDirectSdiType45V1);
    }();
    return result;
}
}
TEST(VehicleType45Original,All44LiteralCardsAnd38RequiredConnectionsPreserveExactSourceDomainRoles) {
    const auto& result = Actual();
    const auto& data = result.data();
    const auto& domain = result.source_domain().domain();
    const auto& evidence = result.source_domain().source().point_mass_source().rigid_source().data().sources;
    ASSERT_EQ(data.rows.size(), 44);
    EXPECT_EQ(data.required, 38); EXPECT_EQ(data.boundaries, 6);
    EXPECT_EQ(domain.node_count(), 372435);
    EXPECT_TRUE(domain.SharesStorage(Domain().domain()));
    EXPECT_EQ(result.readiness(), RuntimeReadiness::RequiresOwnerTt0Context);
    const std::uint64_t first_kind[]{2200512, 2200516, 2200532};
    for (unsigned kind = 0; kind < 3; ++kind) {
        EXPECT_EQ(data.properties[kind].origin_joint_id, first_kind[kind]);
        EXPECT_EQ(data.properties[kind].value.automatic_stiffness_scale, .01);
        EXPECT_EQ(data.properties[kind].value.critical_damping_ratio, .05);
    }
    std::size_t plain = 0, part = 0, unused = 0;
    for (std::size_t i = 0; i < data.rows.size(); ++i) {
        const auto& row = data.rows[i];
        SCOPED_TRACE(row.source_id);
        EXPECT_EQ(row.source_id, 2200512 + i);
        ASSERT_LT(row.source_index, evidence.size());
        const auto& source = evidence[row.source_index];
        ASSERT_EQ(source.cards.size(), 2);
        EXPECT_EQ(source.cards[0].first, row.header_line);
        EXPECT_EQ(source.cards[1].first, row.card_line);
        EXPECT_EQ(row.blank_mask & 0xc0u, 0xc0u);
        for (unsigned n = 0; n < row.source_node_count; ++n) {
            const auto& node = row.nodes[n];
            const auto literal = vehicle::detail::SourceScalar(source.cards[1].second, n);
            ASSERT_TRUE(literal);
            EXPECT_EQ(*literal, node.source_id);
            EXPECT_EQ(node.domain_index, domain.Find(node.source_id));
            if (node.domain_index != SIZE_MAX) {
                const auto position = domain.nodes()[node.domain_index].position;
                EXPECT_EQ(output::Bits(position.x), output::Bits(node.position_m.x));
                EXPECT_EQ(output::Bits(position.y), output::Bits(node.position_m.y));
                EXPECT_EQ(output::Bits(position.z), output::Bits(node.position_m.z));
            }
            if (n < 2 && row.disposition == Disposition::Required) {
                EXPECT_TRUE(node.body.retained);
                plain += node.body.kind == BodyKind::PlainGroup;
                part += node.body.kind == BodyKind::PartRoot;
            }
        }
        for (unsigned slot = 0; slot < 2; ++slot) {
            EXPECT_EQ(row.unused_columns[slot], vehicle::detail::SourceScalar(source.cards[1].second, slot + 4));
            unused += bool(row.unused_columns[slot]);
        }
        const auto geometry = row.Geometry();
        EXPECT_EQ(geometry.source_joint_id, row.source_id);
        EXPECT_EQ(geometry.source_node_id[2], row.property_index ? row.nodes[2].source_id : 0);
    }
    EXPECT_EQ(plain, 48); EXPECT_EQ(part, 28); EXPECT_EQ(unused, 8);
    RecordProperty("source_rows", data.rows.size()); RecordProperty("required", data.required);
    RecordProperty("boundaries", data.boundaries); RecordProperty("complete_forecast", result.forecast().total_bytes);
    RecordProperty("new_owned_payload", data.owned_payload_bytes);
}
TEST(VehicleType45Original,ExactCapCopyLifetimeAndRejectedPolicyPreserveImmutableResult) {
    auto result = Actual();
    const auto* rows = result.data().rows.data();
    const auto exact = result.forecast().total_bytes;
    Limits limits; limits.host_bytes = exact;
    EXPECT_EQ(VehicleType45Source::Preflight(Domain(), Policy::OriginalDirectSdiType45V1, limits).total_bytes, exact);
    --limits.host_bytes;
    EXPECT_THROW(result = VehicleType45Source::Prepare(Domain(), Policy::OriginalDirectSdiType45V1, limits), std::runtime_error);
    EXPECT_EQ(result.data().rows.data(), rows);
    EXPECT_THROW(result = VehicleType45Source::Prepare(Domain(), static_cast<Policy>(77)), std::runtime_error);
    EXPECT_EQ(result.data().rows.back().source_id, 2200555);
    auto copy = [&] { auto temporary = result; return temporary; }();
    EXPECT_EQ(copy.data().rows.data(), rows);
    EXPECT_TRUE(copy.source_domain().domain().SharesStorage(Domain().domain()));
}
} // namespace crash::modelio::type45
