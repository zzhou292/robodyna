#include "../VehicleType25Source.h"
#include "modelio/physical_scope/tests/ActualSupport.h"
#include "modelio/source_assembly/ResolvedSpotweldProperty.h"
#include "lib_src/elements/type25/Type25Frame.h"
#include "lib_src/elements/type25/Type25Property.h"
#include "output/BoundedArrayIO.h"
#include <cmath>
#include <iostream>

namespace crash::modelio::type25 {
namespace {
constexpr Declaration Original{Policy::OriginalDefaultSpotweldsV1, 0x59415249533235ULL};
tl::fea::NodalNodeDomain OriginalDomain(bool extra = false, bool altered_tail = false) {
    const auto& source = physical_scope::test::Actual();
    const auto& canonical = source.tied_source().canonical().data();
    const auto& id_array = physical_scope::source::FindArray(canonical, "node_ids");
    const auto& xyz_array = physical_scope::source::FindArray(canonical, "node_positions");
    const auto ids = output::arrays::Decode<std::uint64_t>(id_array.descriptor, id_array.bytes);
    const auto positions = output::arrays::Decode<double>(xyz_array.descriptor, xyz_array.bytes);
    std::vector<tl::fea::NodalDomainNode> nodes;
    nodes.reserve(source.data().counts.with_type25_nodes + bool(extra));
    for (std::size_t n = ids.size(); n-- > 0;) {
        const bool required = source.data().node_roles[n] &
            (physical_scope::PhysicalRoles | physical_scope::ProvisionalType25);
        if (!required && !extra) continue;
        nodes.push_back({ids[n], {positions[n * 3], positions[n * 3 + 1], positions[n * 3 + 2]}});
        if (!required) extra = false;
    }
    if (altered_tail) nodes.back().position.x = std::nextafter(nodes.back().position.x, INFINITY);
    tl::fea::NodalNodeDomain domain;
    const auto report = domain.Initialize({source.point_mass_source().rigid_source().topology().source_instance_id(),
        nodes.data(), nodes.size()}, tl::fea::NodalDomainLimits::Vehicle());
    output::Require(bool(report), report.message);
    return domain;
}
const tl::fea::NodalNodeDomain& Domain() { static const auto domain = OriginalDomain(); return domain; }
const VehicleType25Source& Actual() {
    static const auto result = [] {
        const auto& source = physical_scope::test::Actual();
        const auto forecast = VehicleType25Source::Preflight(source, Domain(), Original);
        std::cout << "VehicleType25 complete preflight bytes=" << forecast.total_bytes << std::endl;
        return VehicleType25Source::Prepare(source, Domain(), Original);
    }();
    return result;
}
}
TEST(VehicleType25Original,All2828OriginalWeldsPreserveCardsMappingReferencesAndEndpointCoefficients) {
    const auto& source = Actual();
    const auto& model = source.model();
    const auto& rows = source.source().data().spotwelds;
    ASSERT_EQ(rows.size(), 2828);
    ASSERT_EQ(model.connection_count(), rows.size());
    ASSERT_EQ(model.property_count(), 1);
    EXPECT_EQ(model.global_node_count(), Domain().node_count());
    EXPECT_EQ(model.properties()[0].source_property_id, Original.generated_property_id);
    const auto& evidence = source.source().tied_source().data().sources;
    long double mass = 0;
    for (std::size_t e = 0; e < rows.size(); ++e) {
        SCOPED_TRACE(rows[e].id);
        ASSERT_TRUE(rows[e].default_only);
        ASSERT_LT(rows[e].source_index, evidence.size());
        ASSERT_LT(rows[e].first_card + 1, evidence[rows[e].source_index].cards.size());
        EXPECT_EQ(evidence[rows[e].source_index].block.keyword, "*CONSTRAINED_SPOTWELD_ID");
        const auto& c = model.connections()[e];
        EXPECT_EQ(c.source_element_id, rows[e].id);
        for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
            EXPECT_EQ(c.source_node_id[endpoint], rows[e].nodes[endpoint]);
            EXPECT_EQ(c.global_node[endpoint], Domain().Find(rows[e].nodes[endpoint]));
            const auto& node = Domain().nodes()[c.global_node[endpoint]];
            EXPECT_EQ(output::Bits(c.position[endpoint].x), output::Bits(node.position.x));
            EXPECT_EQ(output::Bits(c.position[endpoint].y), output::Bits(node.position.y));
            EXPECT_EQ(output::Bits(c.position[endpoint].z), output::Bits(node.position.z));
            const auto& actual_mass = model.endpoint_mass()[2 * e + endpoint];
            EXPECT_EQ(actual_mass.source_node_id, node.source_id);
            EXPECT_EQ(actual_mass.global_node, c.global_node[endpoint]);
            EXPECT_EQ(actual_mass.source_element_id, rows[e].id);
            EXPECT_EQ(actual_mass.source_property_id, Original.generated_property_id);
            EXPECT_EQ(output::Bits(actual_mass.mass_kg), output::Bits(.0005));
            EXPECT_EQ(output::Bits(actual_mass.isotropic_inertia_kg_m2), output::Bits(5e-9));
            mass += actual_mass.mass_kg;
        }
        native::Reference expected;
        ASSERT_EQ(native::InitializeReference(assembly::SpotweldSourceUnits, c.position, c.seed, expected), native::Status::Success);
        const auto& reference = model.references()[e];
        EXPECT_EQ(output::Bits(reference.length_m), output::Bits(expected.length_m));
        EXPECT_EQ(output::Bits(reference.transverse_axis.x), output::Bits(expected.transverse_axis.x));
        EXPECT_EQ(output::Bits(reference.transverse_axis.y), output::Bits(expected.transverse_axis.y));
        EXPECT_EQ(output::Bits(reference.transverse_axis.z), output::Bits(expected.transverse_axis.z));
        EXPECT_TRUE(model.initial_histories()[e].active);
    }
    EXPECT_NEAR(static_cast<double>(mass), 2.828, 1e-14);
    RecordProperty("connections", model.connection_count());
    RecordProperty("domain_nodes", model.global_node_count());
    RecordProperty("complete_forecast", source.forecast().total_bytes);
    RecordProperty("native_payload", model.owned_payload_bytes());
    RecordProperty("native_startup", model.startup_payload_bytes());
}
TEST(VehicleType25Original,SupersetScopeExactCapsAndLatePositionFailureLeaveOriginalModelIntact) {
    auto source = Actual();
    const auto* before = source.model().connections();
    const auto& census = physical_scope::test::Actual();
    Limits limits;
    limits.host_bytes = source.forecast().total_bytes - 1;
    EXPECT_THROW(source = VehicleType25Source::Prepare(census, Domain(), Original, limits), std::runtime_error);
    EXPECT_EQ(source.model().connections(), before);
    limits.host_bytes += 1;
    EXPECT_NO_THROW(source = VehicleType25Source::Prepare(census, Domain(), Original, limits));
    EXPECT_TRUE(source.model().Matches(Actual().model()));
    limits = {};
    limits.model.max_connections = 2827;
    EXPECT_THROW(VehicleType25Source::Prepare(census, Domain(), Original, limits), std::runtime_error);
    limits = {};
    limits.model.max_host_bytes = source.model().startup_payload_bytes() - 1;
    EXPECT_THROW(VehicleType25Source::Prepare(census, Domain(), Original, limits), std::runtime_error);
    ++limits.model.max_host_bytes;
    EXPECT_NO_THROW(VehicleType25Source::Prepare(census, Domain(), Original, limits));
    const auto* accepted = source.model().connections();
    EXPECT_THROW(source = VehicleType25Source::Prepare(census, OriginalDomain(false, true), Original), std::runtime_error);
    EXPECT_EQ(source.model().connections(), accepted);
    EXPECT_THROW(VehicleType25Source::Prepare(census, Domain(), {Original.policy, 0}), std::runtime_error);
    EXPECT_THROW(VehicleType25Source::Prepare(census, Domain(),
        {static_cast<Policy>(99), Original.generated_property_id}), std::runtime_error);
    EXPECT_THROW(VehicleType25Source::Prepare(census, Domain(), {Original.policy,
        census.tied_source().canonical().data().parts.front().section}), std::runtime_error);
    auto superset = VehicleType25Source::Prepare(census, OriginalDomain(true), Original);
    EXPECT_EQ(superset.domain().node_count(), Domain().node_count() + 1);
    EXPECT_EQ(superset.model().connection_count(), source.model().connection_count());
    EXPECT_EQ(&source.source().tied_source().data(), &census.tied_source().data());
    EXPECT_TRUE(source.domain().SharesStorage(Domain()));
}
} // namespace crash::modelio::type25
