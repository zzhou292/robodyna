#include "ActualFixture.h"
#include "../../nodal_seed/tests/ActualMembers.h"
#include "../../coated/tests/ActualFixture.h"
namespace crash::cases::vehicle_self_contact::native::mixed_interface::test {
namespace physical = vehicle_startup::physical_model::supports_test;
namespace {
constexpr std::size_t ExportReservation = 128u << 20;
const Initial& InitialSource() { return ActualInitialSource(); }

void OuterBound(const Forecast& forecast) {
    output::Require(forecast.peak_bytes <= (std::size_t{10}<<30)-ExportReservation,
        "Mixed actual source plus qualification overhead exceeds unchanged guard");
}
void Write(const char* name, output::Document document) {
    const auto* text = std::getenv("ROBO_MIXED_INTERFACE_OUTPUT");
    output::Require(text && *text, "Missing create-only mixed interface source output");
    const std::filesystem::path directory(text);
    output::Require(std::filesystem::create_directory(directory), "Mixed interface output already exists");
    output::Integer(document, "qualification_export_reservation", ExportReservation);
    output::WriteJson(directory/name, document);
    const auto bytes = output::ReadBounded(directory/name, 1u<<20);
    output::Document receipt;
    receipt.SetObject();
    output::String(receipt, "schema", "robo_dyna.mixed_interface_source_receipt.v1");
    output::String(receipt, "scope", "classification/filter/sides before support; not runtime admission");
    output::String(receipt, "file", name);
    output::String(receipt, "sha256", output::Sha256(bytes));
    output::Integer(receipt, "bytes", bytes.size());
    output::String(receipt, "initial_digest", InitialSource().provenance().output_digest);
    output::WriteJson(directory/"manifest.json", receipt);
}
}
TEST(MixedInterfaceActual, InclusiveForecastAndOneByteShortPreserveSource) {
    const auto& initial = InitialSource();
    const auto forecast = MixedInterfaceSource::Preflight(initial);
    OuterBound(forecast);
    Limits exact;
    exact.host_bytes = forecast.peak_bytes;
    EXPECT_EQ(MixedInterfaceSource::Preflight(initial, exact).peak_bytes, forecast.peak_bytes);
    --exact.host_bytes;
    const auto rejected = MixedInterfaceSource::Prepare(initial, exact);
    EXPECT_EQ(rejected.report.status, Status::ResourceLimit);
    EXPECT_FALSE(rejected.source);
    EXPECT_EQ(initial.faces().size(), 341613u);
    EXPECT_EQ(initial.origin_groups().size(), 66u);
    RecordProperty("peak_reservation_bytes", std::to_string(forecast.peak_bytes));
    Write("forecast.json", ForecastDocument(forecast));
}
TEST(MixedInterfaceActual, CompleteOriginsRealSidesAndSameContextAdmissionCensus) {
    const auto& initial = InitialSource();
    const auto forecast = MixedInterfaceSource::Preflight(initial);
    OuterBound(forecast);
    const auto prepared = MixedInterfaceSource::Prepare(initial);
    Write("mixed.json", ResultDocument(prepared));
    ASSERT_EQ(prepared.report.status, Status::Ready) << prepared.report.reason;
    ASSERT_TRUE(prepared.source);
    const auto copy = *prepared.source;
    EXPECT_EQ(copy.initial().faces().data(), initial.faces().data());
    EXPECT_EQ(copy.initial().geometry().nodes.data(), initial.geometry().nodes.data());
    EXPECT_EQ(copy.provenance().stage, Stage::ClassifiedAndSidesBeforeSupport);
    EXPECT_FALSE(copy.provenance().native_reader_ordinals_available);
    const auto& c = copy.certificate();
    EXPECT_EQ(c.raw_shells, 337092u);
    EXPECT_EQ(c.raw_solids, 4521u);
    EXPECT_EQ(c.unique_coatings, 741u); // Qualification census, never a product predicate.
    EXPECT_TRUE(c.unique_selected_membership && c.complete_origins && c.complete_solid_flags);
    const auto& sides = copy.sides();
    EXPECT_EQ(sides.node_count, 376930u);
    EXPECT_EQ(sides.raw_origin_count, initial.faces().size());
    EXPECT_EQ(sides.main_count, sides.primary_count+sides.shell_primary_count);
    EXPECT_EQ(sides.shell_primary_count, c.shell_primaries);
    EXPECT_GT(c.solid_primaries, 0u);
    EXPECT_GT(c.multi_origin_primaries, 0u);
    EXPECT_EQ(c.coalesced_origins+sides.primary_count, sides.raw_origin_count);
    for (std::size_t i = 0; i < sides.raw_origin_count; ++i) {
        const auto& source = initial.faces()[i].source;
        const auto& origin = sides.raw_origins[i];
        ASSERT_EQ(origin.physical_parent_id, source.element_id) << i;
        ASSERT_EQ(origin.local_face, source.solid_face) << i;
        ASSERT_EQ(origin.origin, s::PrimaryOrigin::SingleSourceFace);
        ASSERT_LT(sides.raw_origin_to_primary[i], sides.primary_count);
        ASSERT_EQ(origin.kind == s::PrimaryFaceKind::Solid, source.kind == initial_surfaces::values::ParentKind::Solid);
    }
    for (std::size_t p = 0; p < sides.primary_count; ++p) {
        const auto& identity = sides.primary_identities[p];
        ASSERT_EQ(sides.primary_to_partner[p] == 0, identity.kind == s::PrimaryFaceKind::Solid);
        if (identity.origin == s::PrimaryOrigin::MultipleOrigins) {
            ASSERT_EQ(identity.physical_parent_id, 0u);
            ASSERT_EQ(copy.primary()[p].source_id, 0u);
            ASSERT_GT(identity.origin_count, 1u);
        }
    }
    const auto& census = copy.admission_census();
    EXPECT_EQ(census.nodes, 376930u);
    EXPECT_EQ(census.positive_mass+census.zero_mass+census.negative_mass+census.nonfinite_mass, census.nodes);
    EXPECT_EQ(census.negative_mass, 0u);
    EXPECT_EQ(census.nonfinite_mass, 0u);
    EXPECT_TRUE(census.actual_rigid_binding_available);
    const auto& physical = initial.context().pre_correction().physical();
    EXPECT_EQ(census.rigid_groups, physical.rigid_assembly().groups().size());
    EXPECT_EQ(census.rigid_members, physical.rigid_assembly().members().size());
    EXPECT_FALSE(census.finalized_cin_available);
    std::size_t source_policies = 0;
    for (const auto& family : census.shell_failure_policies)
        for (auto count : family) source_policies += count;
    EXPECT_EQ(source_policies, 349645u);
    for (std::size_t i = 0; i < census.mass_example_count; ++i) {
        const auto& example = census.mass_examples[i];
        ASSERT_LT(example.domain_node, physical.coefficients().nodes().size());
        EXPECT_EQ(example.source_node_id, physical.source_domain().domain().nodes()[example.domain_node].source_id);
        EXPECT_EQ(output::Bits(example.raw_mass_kg),
            output::Bits(physical.coefficients().nodes()[example.domain_node].coefficients.mass));
    }
    RecordProperty("primary_count", std::to_string(sides.primary_count));
    RecordProperty("main_count", std::to_string(sides.main_count));
    RecordProperty("zero_mass_nodes", std::to_string(census.zero_mass));
    RecordProperty("output_digest", copy.provenance().output_digest);
}
}
