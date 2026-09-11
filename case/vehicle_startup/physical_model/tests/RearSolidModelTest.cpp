#include "../Internal.h"
#include "modelio/solid_source/tests/ActualSupport.h"

namespace crash::cases::vehicle_startup::physical_model::test {
namespace src = modelio::solid_source;
namespace fe = tl::fea;
namespace {
constexpr auto Policy = src::Policy::OriginalAdhesive18ExtendedRubberRearLaw44V3;
constexpr std::size_t ModelCap = 128 * 1024 * 1024;
src::VehicleSolidSource ReadSource() {
    return src::VehicleSolidSource::Prepare(modelio::vehicle::test::Canonical(),
        src::test::MemberBytes(), Policy);
}
std::vector<fe::NodalDomainNode> DomainNodes(const src::VehicleSolidSource& source) {
    const auto& canonical = source.canonical().data();
    const auto ids = src::detail::Decode<std::uint64_t>(canonical, "node_ids");
    const auto positions = src::detail::Decode<double>(canonical, "node_positions");
    std::vector<fe::NodalDomainNode> nodes;
    nodes.reserve(source.data().canonical_nodes.size());
    for (auto it = source.data().canonical_nodes.rbegin(); it != source.data().canonical_nodes.rend(); ++it) {
        const auto n = *it;
        nodes.push_back({ids.at(n), {positions.at(3*n), positions.at(3*n+1), positions.at(3*n+2)}});
    }
    return nodes;
}
}
TEST(VehicleRearSolidModel, All3555SourceParentsBecomeOwnedTypedMechanics) {
    fe::NodalNodeDomain domain;
    fe::solids::Model model;
    std::array<double,46> strain{}, stress{};
    {
        const auto source = ReadSource();
        const auto nodes = DomainNodes(source);
        ASSERT_TRUE(domain.Initialize({7301, nodes.data(), nodes.size()}, fe::NodalDomainLimits::Vehicle()));
        ASSERT_NO_THROW(detail::PrepareSolids(source, domain, ModelCap, model));
        ASSERT_EQ(model.profile(), fe::solids::ModelProfile::ExtendedLaw44Law90);
        EXPECT_EQ(model.solid18().size(), 908u);
        EXPECT_EQ(model.solid24().size(), 1991u);
        EXPECT_EQ(model.solid6z().size(), 350u);
        ASSERT_EQ(model.solid18_law44().size(), 306u);
        ASSERT_EQ(model.materials44().size(), 2u);
        ASSERT_EQ(model.contributions()->parents().size(), 3555u);
        std::copy_n(source.data().rear_plastic_strain.data(), 46, strain.begin());
        std::copy_n(source.data().rear_yield_stress_pa.data(), 46, stress.begin());
        unsigned repeated = 0;
        for (const auto& parent : model.solid18_law44()) {
            const auto& input = parent.reference.input();
            SCOPED_TRACE(input.source_element_id);
            const auto& material = model.materials44()[parent.material_index].value;
            EXPECT_EQ(material.material.young_pa, input.source_part_id == 2000016 ? 50e9 : 200e9);
            EXPECT_NE(material.curve.plastic_strain, source.data().rear_plastic_strain.data());
            for (unsigned slot = 0; slot < 8; ++slot)
                EXPECT_EQ(parent.domain_nodes[slot], domain.Find(input.source_node_id[slot]));
            if (parent.reference.topology() == fe::solid18::law44::SourceTopology::RepeatedPairs56And78) {
                ++repeated;
                EXPECT_EQ(parent.domain_nodes[4], parent.domain_nodes[5]);
                EXPECT_EQ(parent.domain_nodes[6], parent.domain_nodes[7]);
            }
        }
        EXPECT_EQ(repeated, 109u);
    }
    // The source and temporary input vectors are gone; model-owned curves survive.
    for (const auto& material : model.materials44()) {
        ASSERT_EQ(material.value.curve.count, 46u);
        for (unsigned i = 0; i < 46; ++i) {
            EXPECT_EQ(output::Bits(material.value.curve.plastic_strain[i]), output::Bits(strain[i]));
            EXPECT_EQ(output::Bits(material.value.curve.yield_stress_pa[i]), output::Bits(stress[i]));
        }
    }
    RecordProperty("parents", model.contributions()->parents().size());
    RecordProperty("model_owned_bytes", model.owned_payload_bytes());
}
TEST(VehicleRearSolidModel, MissingDomainOrInsufficientBudgetRejectsThenFreshModelRetries) {
    const auto source = ReadSource();
    const auto nodes = DomainNodes(source);
    fe::NodalNodeDomain incomplete, complete;
    ASSERT_TRUE(incomplete.Initialize({7302, nodes.data(), nodes.size()-1}, fe::NodalDomainLimits::Vehicle()));
    ASSERT_TRUE(complete.Initialize({7302, nodes.data(), nodes.size()}, fe::NodalDomainLimits::Vehicle()));
    fe::solids::Model model;
    EXPECT_THROW(detail::PrepareSolids(source, incomplete, ModelCap, model), std::runtime_error);
    EXPECT_FALSE(model.prepared());
    EXPECT_THROW(detail::PrepareSolids(source, complete, 1, model), std::runtime_error);
    EXPECT_FALSE(model.prepared());
    ASSERT_NO_THROW(detail::PrepareSolids(source, complete, ModelCap, model));
    EXPECT_TRUE(model.prepared());
}
} // namespace crash::cases::vehicle_startup::physical_model::test
