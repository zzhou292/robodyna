#include "../Internal.h"
#include "../../nodal_seed/tests/ActualMembers.h"
#include "../../coated/tests/ActualFixture.h"
#include <unordered_map>
namespace crash::cases::vehicle_self_contact::native::initial_surfaces::test {
namespace physical = vehicle_startup::physical_model::supports_test;
namespace {
const nodal_seed::test::ActualMembers& Members() {
    static const nodal_seed::test::ActualMembers value(
        physical::Model().shell_source().references().source().canonical());
    return value;
}
const Context& Corrected() {
    static const auto value = [] {
        const auto before = nodal_seed::PreCorrectionNodalSource::Prepare(
            physical::Model(), physical::Joints(), Members().Input());
        auto prepared = Context::Prepare(before, Members().Input());
        output::Require(prepared.report.status == nodal_correction::Status::Ready && bool(prepared.source),
            "Actual initial surface fixture requires qualified corrected source");
        return *prepared.source;
    }();
    return value;
}
const Selection& Selected() { return coated::test::Selection(); }
void Write(const char* name, const output::Document& document) {
    const auto* path = std::getenv("ROBO_INITIAL_SURFACE_OUTPUT");
    output::Require(path && *path, "Missing create-only initial surface output");
    const std::filesystem::path directory(path);
    output::Require(std::filesystem::create_directory(directory), "Initial surface output already exists");
    output::WriteJson(directory/name, document);
    const auto bytes = output::ReadBounded(directory/name, 1u<<20);
    output::Document receipt;
    receipt.SetObject();
    output::String(receipt, "schema", "robo_dyna.initial_surface_source_receipt.v1");
    output::String(receipt, "scope", "initial clause only; no mixed interface or runtime admission");
    output::String(receipt, "file", name);
    output::String(receipt, "sha256", output::Sha256(bytes));
    output::Integer(receipt, "bytes", bytes.size());
    output::String(receipt, "source_digest", Corrected().provenance().source_digest);
    output::WriteJson(directory/"manifest.json", receipt);
}
}
TEST(InitialSurfaceActual, InclusiveForecastAndOneByteShortPreserveSource) {
    const auto& source = Corrected();
    const auto forecast = InitialSurfaceSource::Preflight(source, Selected());
    Limits exact;
    exact.host_bytes = forecast.peak_bytes;
    EXPECT_EQ(InitialSurfaceSource::Preflight(source, Selected(), exact).peak_bytes, forecast.peak_bytes);
    --exact.host_bytes;
    const auto rejected = InitialSurfaceSource::Prepare(source, Selected(), "unread-invalid-member", exact);
    EXPECT_EQ(rejected.report.status, Status::ResourceLimit);
    EXPECT_FALSE(rejected.source);
    EXPECT_EQ(source.provenance().certificate.affected_nodes, 6656u);
    RecordProperty("peak_reservation_bytes", std::to_string(forecast.peak_bytes));
    Write("forecast.json", ForecastDocument(forecast));
}
TEST(InitialSurfaceActual, CompletePhysicalContextPublishesOnlyCertifiedTypedFaces) {
    const auto& source = Corrected();
    const auto coefficients = source.coefficients();
    const std::vector<double> unchanged(coefficients.begin(), coefficients.end());
    const auto result = InitialSurfaceSource::Prepare(source, Selected(), physical::Inputs().member);
    // Preserve an informative failure census before asserting admission.
    Write("surface.json", ResultDocument(result));
    ASSERT_EQ(result.report.status, Status::Ready) << result.report.reason;
    ASSERT_TRUE(result.source);
    const auto copied = *result.source;
    const auto& c = copied.census();
    EXPECT_EQ(c.nodes, 376930u);
    EXPECT_EQ(c.physical_shells, 349645u);
    EXPECT_EQ(c.physical_solids, 4980u);
    EXPECT_EQ(c.reader_bricks, 4630u);
    EXPECT_EQ(c.native_raw8_bricks, 113u);
    EXPECT_EQ(c.declared_penta, 350u);
    EXPECT_EQ(c.original_selected_solids, 2952u);
    EXPECT_EQ(c.extraction.selected_solids, 2496u);
    EXPECT_EQ(c.omitted_selected_solids, 456u);
    EXPECT_EQ(c.extraction.selected_quads + c.extraction.selected_triangles, 337092u);
    EXPECT_EQ(c.faces, c.extraction.solid_faces + c.extraction.shell_faces);
    EXPECT_EQ(c.faces, c.quad_faces + c.triangle_faces);
    EXPECT_EQ(c.extraction.shell_faces, 337092u);
    EXPECT_TRUE(copied.certificate().membership_complete);
    EXPECT_TRUE(copied.certificate().sort_order_complete);
    EXPECT_FALSE(copied.provenance().native_reader_ordinals_available);
    EXPECT_EQ(copied.provenance().stage, Stage::InitialClauseBeforeI25Classification);
    EXPECT_EQ(copied.provenance().output_digest.size(), 64u);
    EXPECT_EQ(copied.faces().data(), result.source->faces().data());
    const auto& geometry = copied.geometry();
    std::unordered_map<std::uint64_t, std::size_t> solids, shells;
    for (std::size_t i = 0; i < geometry.solids.size(); ++i)
        ASSERT_TRUE(solids.emplace(geometry.solids[i].source_id, i).second);
    for (std::size_t i = 0; i < geometry.shells.size(); ++i)
        ASSERT_TRUE(shells.emplace(geometry.shells[i].primary.source_id, i).second);
    std::vector<std::uint8_t> emitted(geometry.solids.size(), 0);
    for (const auto& face : copied.faces()) {
        for (const auto node : face.nodes) ASSERT_LT(node, geometry.nodes.size());
        if (face.source.kind == values::ParentKind::Solid) {
            const auto found = solids.find(face.source.element_id);
            ASSERT_NE(found, solids.end());
            const auto& parent = geometry.solids[found->second];
            EXPECT_EQ(face.source.part_id, parent.part_id);
            EXPECT_EQ(face.source.canonical_row, parent.canonical_row);
            EXPECT_EQ(face.source.source_line, parent.source_line);
            EXPECT_GE(face.source.solid_face, 1u);
            EXPECT_LE(face.source.solid_face, 6u);
            for (const auto node : face.nodes)
                EXPECT_NE(std::find(parent.nodes.begin(), parent.nodes.end(), node), parent.nodes.end());
            emitted[found->second] = 1;
        } else {
            const auto found = shells.find(face.source.element_id);
            ASSERT_NE(found, shells.end());
            const auto& parent = geometry.shells[found->second];
            EXPECT_TRUE(parent.contact_selected);
            EXPECT_EQ(face.source.part_id, parent.part_id);
            EXPECT_EQ(face.source.canonical_row, parent.canonical_row);
            EXPECT_EQ(face.source.source_line, parent.source_line);
            EXPECT_EQ(face.source.solid_face, 0u);
            for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(face.nodes[i], parent.primary.nodes[i]);
        }
    }
    EXPECT_EQ(emitted, copied.emitted_solid_flags());
    ASSERT_EQ(unchanged.size(), source.coefficients().size());
    for (std::size_t i = 0; i < unchanged.size(); ++i)
        ASSERT_EQ(output::Bits(unchanged[i]), output::Bits(source.coefficients()[i]));
    RecordProperty("initial_faces", std::to_string(c.faces));
    RecordProperty("solid_faces", std::to_string(c.extraction.solid_faces));
    RecordProperty("output_digest", copied.provenance().output_digest);
    RecordProperty("scope", "complete declared V5 initial surface clause; no native reader ordinals, K or runtime");
}
}
