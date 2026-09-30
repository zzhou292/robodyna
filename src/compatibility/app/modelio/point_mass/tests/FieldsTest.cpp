#include "../Mapping.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>

namespace crash::modelio::point_mass {
namespace {
tl::fea::NodalNodeDomain Domain() {
    const tl::fea::NodalDomainNode nodes[]{{30, {0, 0, 1}}, {10, {-0., 0, 0}}};
    tl::fea::NodalNodeDomain domain;
    output::Require(bool(domain.Initialize({71, nodes, 2})), "Tiny mass domain failed");
    return domain;
}
std::vector<physical_scope::rigid::point_mass::Record> Records() {
    std::vector<physical_scope::rigid::point_mass::Record> result(3);
    for (unsigned i = 0; i < 3; ++i) {
        result[i].value.source_element_id = 101 + i;
        result[i].value.source_node_id = i == 1 ? 20 : 10;
        result[i].value.supplied_mass_source = .000123 * (i + 1);
    }
    return result;
}
}
TEST(VehiclePointMassValues,PreservesSourceOrderDistinctCardsAndExplicitOmissions) {
    const auto domain = Domain();
    const auto records = Records();
    const auto mapped = detail::Map(records, domain, 3);
    ASSERT_EQ(mapped.dispositions.size(), 3);
    ASSERT_EQ(mapped.retained.size(), 2);
    EXPECT_EQ(mapped.dispositions[0].domain_node, 1);
    EXPECT_EQ(mapped.dispositions[1].domain_node, SIZE_MAX);
    EXPECT_EQ(mapped.dispositions[2].source_record, 2);
    EXPECT_EQ(mapped.retained[0].source_element_id, 101);
    EXPECT_EQ(mapped.retained[1].source_element_id, 103);
    EXPECT_EQ(mapped.retained[1].domain_node, 1);
    tl::fea::ElementMassContributions native;
    ASSERT_TRUE(native.Initialize(domain, {71, 1000, mapped.retained.data(), mapped.retained.size()}));
    ASSERT_EQ(native.records().size(), 2);
    for (std::size_t row = 0; row < 2; ++row) {
        EXPECT_EQ(output::Bits(native.records()[row].mass_kg),
                  output::Bits(mapped.retained[row].mass_source * 1000));
        EXPECT_EQ(native.records()[row].isotropic_inertia_kg_m2(), 0);
    }
}
TEST(VehiclePointMassValues,ExactSourceCapRejectsWithoutChangingPriorMapping) {
    const auto domain = Domain();
    auto result = detail::Map(Records(), domain, 3);
    EXPECT_THROW(result = detail::Map(Records(), domain, 2), std::runtime_error);
    EXPECT_EQ(result.retained.back().source_element_id, 103);
    EXPECT_THROW(detail::Map(Records(), domain, 0), std::runtime_error);
    tl::fea::NodalNodeDomain empty;
    EXPECT_THROW(detail::Map(Records(), empty, 3), std::runtime_error);
    EXPECT_NO_THROW(result = detail::Map(Records(), domain, 3));
}
} // namespace crash::modelio::point_mass
