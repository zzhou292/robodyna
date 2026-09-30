#include "../VehiclePointMassSource.h"
#include "modelio/physical_scope/tests/ActualSupport.h"
#include "output/BoundedArrayIO.h"
#include <cmath>
#include <set>

namespace crash::modelio::point_mass {
namespace {
tl::fea::NodalNodeDomain Domain(bool all_masses = false, bool altered = false) {
    const auto& source = physical_scope::test::Actual();
    const auto& canonical = source.tied_source().canonical().data();
    const auto& id_array = physical_scope::source::FindArray(canonical, "node_ids");
    const auto& xyz_array = physical_scope::source::FindArray(canonical, "node_positions");
    const auto ids = output::arrays::Decode<std::uint64_t>(id_array.descriptor, id_array.bytes);
    const auto xyz = output::arrays::Decode<double>(xyz_array.descriptor, xyz_array.bytes);
    std::set<std::uint64_t> extra;
    if (all_masses)
        for (const auto& mass : source.point_mass_source().data().records) extra.insert(mass.value.source_node_id);
    std::vector<tl::fea::NodalDomainNode> nodes;
    nodes.reserve(source.data().counts.with_type25_nodes + extra.size());
    for (std::size_t n = ids.size(); n-- > 0;)
        if ((source.data().node_roles[n] & (physical_scope::PhysicalRoles | physical_scope::ProvisionalType25)) ||
            extra.count(ids[n]))
            nodes.push_back({ids[n], {xyz[3*n], xyz[3*n+1], xyz[3*n+2]}});
    if (altered) nodes.back().position.x = std::nextafter(nodes.back().position.x, INFINITY);
    tl::fea::NodalNodeDomain domain;
    const auto report = domain.Initialize({source.point_mass_source().rigid_source().topology().source_instance_id(),
        nodes.data(), nodes.size()}, tl::fea::NodalDomainLimits::Vehicle());
    output::Require(bool(report), report.message);
    return domain;
}
void CheckRows(const VehiclePointMassSource& result, const tl::fea::NodalNodeDomain& domain, std::size_t count) {
    const auto& source = result.source().point_mass_source().data().records;
    const auto& rows = result.contributions().records();
    ASSERT_EQ(source.size(), 155);
    ASSERT_EQ(result.dispositions().size(), source.size());
    ASSERT_EQ(rows.size(), count);
    ASSERT_TRUE(result.contributions().domain()->SharesStorage(domain));
    std::size_t consumed = 0;
    for (std::size_t r = 0; r < source.size(); ++r) {
        SCOPED_TRACE(r);
        const auto& value = source[r].value;
        const auto index = domain.Find(value.source_node_id);
        EXPECT_EQ(result.dispositions()[r].source_record, r);
        EXPECT_EQ(result.dispositions()[r].domain_node, index);
        if (index == SIZE_MAX) continue;
        ASSERT_LT(consumed, rows.size());
        const auto& actual = rows[consumed++];
        EXPECT_EQ(actual.source.source_element_id, value.source_element_id);
        EXPECT_EQ(actual.source.source_node_id, value.source_node_id);
        EXPECT_EQ(actual.source.domain_node, index);
        EXPECT_EQ(output::Bits(actual.source.mass_source), output::Bits(value.supplied_mass_source));
        EXPECT_EQ(output::Bits(actual.mass_kg), output::Bits(value.supplied_mass_kg));
        EXPECT_EQ(actual.isotropic_inertia_kg_m2(), 0);
    }
    EXPECT_EQ(consumed, count);
}
}
TEST(VehiclePointMassOriginal,IncludesTwoRetainedShellMassesAndAccountsEveryOriginalCard) {
    const auto& source = physical_scope::test::Actual();
    const auto domain = Domain();
    const auto forecast = VehiclePointMassSource::Preflight(source, domain);
    std::cout << "VehiclePointMass complete preflight bytes=" << forecast.total_bytes << std::endl;
    const auto result = VehiclePointMassSource::Prepare(source, domain);
    CheckRows(result, domain, 56);
    std::size_t shell_masses = 0;
    for (const auto& row : result.contributions().records())
        if (row.source.source_element_id == 2409343 || row.source.source_element_id == 2409344) ++shell_masses;
    EXPECT_EQ(shell_masses, 2);
    EXPECT_EQ(result.dispositions().size() - result.contributions().records().size(), 99);
    const auto expanded = Domain(true);
    const auto all = VehiclePointMassSource::Prepare(source, expanded);
    CheckRows(all, expanded, 155);
    RecordProperty("baseline_mass_records", result.contributions().records().size());
    RecordProperty("all_mass_records", all.contributions().records().size());
    RecordProperty("complete_forecast", result.forecast().total_bytes);
    RecordProperty("native_payload", result.contributions().owned_payload_bytes());
    RecordProperty("baseline_domain_nodes", domain.node_count());
    RecordProperty("expanded_domain_nodes", expanded.node_count());
}
TEST(VehiclePointMassOriginal,ExactCapsAndLateCoordinateFailurePreserveAcceptedSource) {
    const auto& source = physical_scope::test::Actual();
    const auto domain = Domain();
    auto result = VehiclePointMassSource::Prepare(source, domain);
    const auto* before = result.contributions().records().data();
    Limits limits;
    limits.host_bytes = result.forecast().total_bytes - 1;
    EXPECT_THROW(result = VehiclePointMassSource::Prepare(source, domain, limits), std::runtime_error);
    EXPECT_EQ(result.contributions().records().data(), before);
    ++limits.host_bytes;
    EXPECT_NO_THROW(result = VehiclePointMassSource::Prepare(source, domain, limits));
    const auto* accepted = result.contributions().records().data();
    EXPECT_THROW(result = VehiclePointMassSource::Prepare(source, Domain(false, true)), std::runtime_error);
    EXPECT_EQ(result.contributions().records().data(), accepted);
    limits = {};
    limits.native.max_host_bytes = result.contributions().startup_payload_bytes() - 1;
    EXPECT_THROW(VehiclePointMassSource::Prepare(source, domain, limits), std::runtime_error);
    ++limits.native.max_host_bytes;
    EXPECT_NO_THROW(VehiclePointMassSource::Prepare(source, domain, limits));
    EXPECT_EQ(&result.source().data(), &source.data());
}
} // namespace crash::modelio::point_mass
