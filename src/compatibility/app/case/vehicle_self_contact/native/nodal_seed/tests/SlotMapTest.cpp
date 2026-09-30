#include "../SolidSlots.h"
#include "lib_utest/qualification/solid6z_reference/TestSupport.h"
#include <gtest/gtest.h>

namespace crash::cases::vehicle_self_contact::native::nodal_seed::test {
namespace {
struct Penta {
    tl::fea::NodalNodeDomain domain;
    tl::fea::solids::Parent6z parent;
    modelio::solid_source::Row source;
    explicit Penta(bool reverse) {
        auto input = solid6z_test::Distorted();
        if (reverse)
            for (unsigned i = 0; i < 3; ++i) std::swap(input.position_m[i], input.position_m[i + 3]);
        output::Require(tl::fea::solid6z::InitializeReference(input, parent.reference) ==
            tl::fea::solid6z::Status::Success, "Penta source fixture rejected");
        tl::fea::NodalDomainNode nodes[6];
        for (unsigned i = 0; i < 6; ++i) {
            nodes[i] = {input.source_node_id[i], input.position_m[i]};
            parent.domain_nodes[i] = i;
        }
        output::Require(bool(domain.Initialize({7, nodes, 6})), "Penta fixture domain rejected");
        source.element_id = input.source_element_id;
        source.part_id = input.source_part_id;
        source.family = modelio::solid_source::Family::Solid6z;
        source.raw_node_ids = {100, 101, 104, 103, 102, 102, 105, 105};
        source.six_to_raw = {0, 1, 4, 3, 2, 6};
    }
};
}
TEST(ContactSeedSlots, PentaInactiveRawSlotsKeepOriginalNodesAfterNativeReorientation) {
    Penta original(false), reversed(true);
    const std::array<std::uint32_t, 8> expected_original{0, 1, 2, 0, 3, 4, 5, 3};
    const std::array<std::uint32_t, 8> expected_reversed{3, 4, 5, 0, 0, 1, 2, 3};
    EXPECT_EQ(detail::PostInitiaSolidSlots(original.source, original.parent, original.domain, 6), expected_original);
    EXPECT_EQ(detail::PostInitiaSolidSlots(reversed.source, reversed.parent, reversed.domain, 6), expected_reversed);
    EXPECT_NE(expected_reversed[0], expected_reversed[3]);
    EXPECT_NE(expected_reversed[4], expected_reversed[7]);
}
TEST(ContactSeedSlots, ForeignElementRawNodeAndPhysicalNodeMappingsReject) {
    Penta fixture(true);
    auto bad = fixture.source;
    ++bad.element_id;
    EXPECT_THROW(detail::PostInitiaSolidSlots(bad, fixture.parent, fixture.domain, 6), std::exception);
    bad = fixture.source;
    ++bad.raw_node_ids[4];
    EXPECT_THROW(detail::PostInitiaSolidSlots(bad, fixture.parent, fixture.domain, 6), std::exception);
    auto parent = fixture.parent;
    parent.domain_nodes[0] = 4;
    EXPECT_THROW(detail::PostInitiaSolidSlots(fixture.source, parent, fixture.domain, 6), std::exception);
    EXPECT_NO_THROW(detail::PostInitiaSolidSlots(fixture.source, fixture.parent, fixture.domain, 6));
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::test
