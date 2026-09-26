#include "../WallSource.h"
#include "../Internal.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_self_contact/native/nodal_seed/tests/ActualMembers.h"
#include "case/vehicle_startup/physical_model/tests/supports/Support.h"
#include "output/BoundedArrayJson.h"
#include "lib_src/math/ScalarBits.h"
#include <cstdlib>
#include <filesystem>
#include <sstream>

namespace crash::cases::vehicle_wall::native::test {
namespace physical = vehicle_startup::physical_model::supports_test;
namespace member_fixture = vehicle_self_contact::native::nodal_seed::test;
namespace {
constexpr std::size_t ExportBytes = 128u << 10;
const member_fixture::ActualMembers& Members() {
    static const member_fixture::ActualMembers value(physical::Domain().source().tied_source().canonical());
    return value;
}
const std::string& WallBytes() {
    static const auto value = [] {
        const auto* path = std::getenv("ROBO_NATIVE_WALL_MANIFEST");
        output::Require(path && *path, "Missing authenticated original wall manifest");
        return case_data::ReadPinnedWallManifest(path);
    }();
    return value;
}
const case_data::CanonicalWall& Wall() {
    static const auto value = [] {
        auto wall = std::make_unique<case_data::CanonicalWall>();
        std::istringstream input(WallBytes());
        output::Require(wall->Load(input).status == case_data::WallStatus::Ok, "Original wall source rejected");
        return wall;
    }();
    return *value;
}
Declaration DeclaredWall() {
    Declaration result;
    result.profile = Profile::EnvelopeFixedElasticV1;
    return result;
}
void SourceCounts() {
    ASSERT_EQ(physical::Domain().policy(), modelio::physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5);
    ASSERT_EQ(physical::Domain().domain().node_count(), 376930u);
    ASSERT_EQ(physical::Domain().source().tied_source().canonical().data().canonical_nodes, 393165u);
    ASSERT_EQ(Wall().vertices().size(), 62u);
    ASSERT_EQ(Wall().source_quads().size(), 46u);
}
std::filesystem::path Destination() {
    const auto* text = std::getenv("ROBO_NATIVE_WALL_OUTPUT");
    output::Require(text && *text, "Missing create-only wall source output path");
    const std::filesystem::path path(text);
    output::Require(std::filesystem::create_directory(path), "Wall source output already exists");
    return path;
}
void ForecastFields(output::Document& doc, const Forecast& f) {
    output::Integer(doc, "retained_vehicle_bytes", f.retained_vehicle);
    output::Integer(doc, "import_context_bytes", f.import_context);
    output::Integer(doc, "namespace_workspace_bytes", f.namespace_workspace);
    output::Integer(doc, "common_domain_bytes", f.common_domain);
    output::Integer(doc, "input_packing_bytes", f.input_packing);
    output::Integer(doc, "masks_bytes", f.masks);
    output::Integer(doc, "geometry_bytes", f.geometry);
    output::Integer(doc, "digest_bytes", f.digest);
    output::Integer(doc, "source_peak_bytes", f.peak_bytes);
    output::Integer(doc, "qualification_export_bytes", ExportBytes);
    output::Integer(doc, "qualification_peak_bytes", f.peak_bytes + ExportBytes);
}
void Write(const WallSource& source) {
    output::Document doc;
    doc.SetObject();
    output::String(doc, "schema", "robo_dyna.native_envelope_wall_source.v1");
    output::String(doc, "scope", "Declared envelope material, intrinsic component values and fresh common domain only");
    output::String(doc, "digest", source.digest());
    output::String(doc, "namespace_digest", source.namespace_report().digest);
    output::String(doc, "source_digest", source.namespace_report().source_digest);
    output::String(doc, "profile", "EnvelopeFixedElasticV1");
    output::Integer(doc, "vehicle_nodes", source.vehicle_prefix().nodes);
    output::Integer(doc, "combined_nodes", source.domain().node_count());
    output::Integer(doc, "added_nodes", 4);
    output::Integer(doc, "added_q4_parents", 1);
    output::Integer(doc, "complete_namespace_definitions", source.namespace_report().definitions);
    output::Integer(doc, "maximum_declared_id", source.namespace_report().maximum_declared);
    output::Integer(doc, "maximum_derived_spring_id", source.namespace_report().maximum_spring);
    output::Integer(doc, "allocation_ceiling", source.namespace_report().allocation_ceiling);
    const auto& ids = source.ids();
    for(unsigned i = 0; i < 4; ++i)
        output::Integer(doc, ("wall_node_" + std::to_string(i)).c_str(), ids.nodes[i]);
    output::Integer(doc, "wall_element_id", ids.shell);
    output::Integer(doc, "wall_part_id", ids.part);
    output::Integer(doc, "wall_material_id", ids.material);
    output::Integer(doc, "wall_section_id", ids.section);
    output::Integer(doc, "wall_node_set_id", ids.node_set);
    output::Integer(doc, "wall_surface_id", ids.surface);
    output::Integer(doc, "wall_interface_id", ids.interface);
    const auto& g = source.geometry();
    output::Number(doc, "young_pa", source.declaration().material.young_pa);
    output::Number(doc, "poisson", source.declaration().material.poisson);
    output::Number(doc, "density_kg_m3", source.declaration().material.density_kg_m3);
    output::Number(doc, "thickness_m", source.declaration().material.thickness_m);
    output::Number(doc, "friction", source.declaration().wall_friction);
    output::Number(doc, "front_contact_plane_m", g.placement.represented_wall_x_m);
    output::Number(doc, "reference_plane_m", g.reference_plane_m);
    output::Number(doc, "reference_offset_m", g.reference_offset_m);
    output::Number(doc, "component_half_gap_native", g.native_half_gap);
    output::Number(doc, "component_primary_stiffness_native", g.component_primary_stiffness_native);
    output::Number(doc, "wall_mass_kg", g.wall_mass_kg);
    output::Number(doc, "wall_area_m2", g.reference.area);
    output::Number(doc, "projection_working_length_m", g.native_working_length_m);
    output::Number(doc, "display_density_native_provenance_only", source.namespace_report().display_density_native);
    output::Boolean(doc, "runtime_ready", false);
    output::Boolean(doc, "final_contact_arrays_ready", false);
    output::Boolean(doc, "vehicle_prefix_bit_preserved", true);
    output::Boolean(doc, "generated_nodes_follow_complete_explicit_input", source.namespace_report().generated_node_after_complete_input);
    ForecastFields(doc, source.forecast());
    output::WriteJson(Destination() / "source.json", doc);
}
}
TEST(NativeEnvelopeWallActual, ForecastRejectsOneByteShortWithoutDomainPublication) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto forecast = WallSource::Preflight(physical::Domain(), Members().Input());
    auto exact = Limits{};
    exact.host_bytes = forecast.peak_bytes;
    EXPECT_EQ(WallSource::Preflight(physical::Domain(), Members().Input(), exact).peak_bytes, forecast.peak_bytes);
    --exact.host_bytes;
    const auto rejected = WallSource::Prepare(physical::Domain(), Members().Input(), Wall(), WallBytes(), DeclaredWall(), exact);
    EXPECT_EQ(rejected.report.status, Status::ResourceLimit);
    EXPECT_FALSE(rejected.source);
    output::Document doc;
    doc.SetObject();
    output::String(doc, "schema", "robo_dyna.native_envelope_wall_forecast.v1");
    ForecastFields(doc, forecast);
    output::WriteJson(Destination() / "forecast.json", doc);
}
TEST(NativeEnvelopeWallActual, CompleteSourceAllocatesFreshIdsAndPreservesEveryVehicleNode) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto prepared = WallSource::Prepare(physical::Domain(), Members().Input(), Wall(), WallBytes(), DeclaredWall());
    ASSERT_EQ(prepared.report.status, Status::Ready) << prepared.report.reason
        << " file=" << prepared.report.file << " row=" << prepared.report.row
        << " source_id=" << prepared.report.source_id;
    ASSERT_TRUE(prepared.source);
    const auto& source = *prepared.source;
    const auto& original = physical::Domain().domain();
    ASSERT_EQ(source.domain().node_count(), 376934u);
    ASSERT_EQ(source.vehicle_prefix().begin, 0u);
    ASSERT_EQ(source.vehicle_prefix().nodes, original.node_count());
    ASSERT_FALSE(source.domain().SharesStorage(original));
    ASSERT_TRUE(source.vehicle_origin().domain().SharesStorage(original));
    for(std::size_t i = 0; i < original.node_count(); ++i) {
        const auto& a = original.nodes()[i];
        const auto& b = source.domain().nodes()[i];
        ASSERT_EQ(a.source_id, b.source_id) << i;
        ASSERT_TRUE(tl::math::SameScalarBits(a.position.x, b.position.x) &&
            tl::math::SameScalarBits(a.position.y, b.position.y) &&
            tl::math::SameScalarBits(a.position.z, b.position.z)) << i;
        ASSERT_EQ(source.translation_fixed_bits()[i], 0);
        ASSERT_EQ(source.rotation_fixed()[i], 0);
    }
    for(unsigned i = 0; i < 4; ++i) {
        const auto n = original.node_count() + i;
        ASSERT_EQ(source.domain().nodes()[n].source_id, source.ids().nodes[i]);
        ASSERT_GT(source.ids().nodes[i], source.namespace_report().allocation_ceiling);
        ASSERT_EQ(source.translation_fixed_bits()[n], 7);
        ASSERT_EQ(source.rotation_fixed()[n], 1);
        ASSERT_GT(source.geometry().reference.nodal_mass[i], 0);
        ASSERT_GT(source.geometry().reference.isotropic_inertia[i], 0);
    }
    EXPECT_EQ(source.startup().kind, tl::fea::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation);
    EXPECT_EQ(source.geometry().native_half_gap, .5);
    EXPECT_EQ(source.geometry().component_primary_stiffness_native, 200000.);
    EXPECT_EQ(source.geometry().reference_plane_m - source.geometry().reference_offset_m,
        source.geometry().placement.represented_wall_x_m);
    const auto copy = source;
    EXPECT_TRUE(copy.domain().SharesStorage(source.domain()));
    EXPECT_EQ(copy.digest(), source.digest());
    Write(source);
}
} // namespace crash::cases::vehicle_wall::native::test
