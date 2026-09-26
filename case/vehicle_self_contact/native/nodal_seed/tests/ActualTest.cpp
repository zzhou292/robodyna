#include "ActualMembers.h"
#include "case/vehicle_startup/physical_model/tests/supports/Support.h"
#include "output/BoundedArrayJson.h"
#include <cmath>

namespace crash::cases::vehicle_self_contact::native::nodal_seed::test {
namespace physical = vehicle_startup::physical_model::supports_test;
namespace {
const ActualMembers& Members() {
    static const ActualMembers value(physical::Model().shell_source().references().source().canonical());
    return value;
}
void CountsAreComplete(const Counts& count) {
    EXPECT_EQ(count.nodes, 376930u);
    EXPECT_EQ(count.shells, 349645u);
    EXPECT_EQ(count.solids, 4980u);
    EXPECT_EQ(count.solid_families, (std::array<std::size_t, 5>{908, 1991, 350, 386, 1345}));
    EXPECT_EQ(count.beams, 142u);
    EXPECT_EQ(count.type13, 4442u);
    EXPECT_EQ(count.type25, 2828u);
    EXPECT_EQ(count.type45, 44u);
    EXPECT_EQ(count.volume_occurrences, 39840u);
    EXPECT_EQ(count.stiffness_occurrences, 14912u);
    EXPECT_EQ(count.shell_occurrences, 1377279u);
}
void Write(const PreCorrectionNodalSource& value) {
    const auto* destination = std::getenv("ROBO_VEHICLE_CONTACT_SEED_OUTPUT");
    output::Require(destination && *destination, "Missing create-only contact seed output path");
    const std::filesystem::path directory(destination);
    output::Require(std::filesystem::create_directory(directory), "Contact seed output directory already exists");
    output::Document document;
    document.SetObject();
    output::String(document, "schema", "robo_dyna.v5_pre_correction_nodal_source.v1");
    output::String(document, "scope", "complete declared V5 pre-correction source; no interface K, gaps, owner or runtime admission");
    output::String(document, "canonical_sha256", value.provenance().canonical_sha256);
    output::String(document, "import_source_digest", value.provenance().import_source_digest);
    output::String(document, "spring_mapping_digest", value.provenance().spring_mapping_digest);
    output::String(document, "contributor_digest", value.provenance().contributor_digest);
    output::Integer(document, "nodes", value.counts().nodes);
    output::Integer(document, "direct_requested_interior_solids", value.interior().retained_solids);
    output::Integer(document, "peak_source_reservation_bytes", value.forecast().peak_bytes);
    const auto seeds = value.seed();
    const auto fields = value.fields();
    std::vector<double> values;
    values.reserve(6 * seeds.node_count);
    for (std::size_t i = 0; i < seeds.node_count; ++i) {
        values.push_back(seeds.nodes[i].volume);
        values.push_back(seeds.nodes[i].bulk_volume);
        values.push_back(seeds.nodes[i].existing_stiffness);
        values.push_back(fields[i].shell_young_thickness_sum);
        values.push_back(double(fields[i].shell_incidence_count));
        values.push_back(fields[i].stiffness_before_control);
    }
    const auto file = output::arrays::Write<double>(directory, "pre-correction.bin",
        {output::arrays::Scalar::Float64, seeds.node_count, 6,
         {"native_volume", "native_bulk_volume", "native_direct_stiffness", "native_shell_et", "shell_incidence", "native_stiffness_before_control"}},
        values.data(), values.size());
    output::array_json::Child(document, "pre_correction_fields", output::arrays::DescriptorDocument(file));
    output::WriteJson(directory / "source.json", document);
}
}
TEST(VehicleContactSeedActual, CompleteReservationRejectsOneByteShortBeforePreparation) {
    const auto members = Members().Input();
    const auto forecast = PreCorrectionNodalSource::Preflight(physical::Model(), physical::Joints(), members);
    CountsAreComplete(forecast.counts);
    auto exact = Limits{};
    exact.host_bytes = forecast.peak_bytes;
    EXPECT_EQ(PreCorrectionNodalSource::Preflight(physical::Model(), physical::Joints(), members, exact).peak_bytes,
        forecast.peak_bytes);
    --exact.host_bytes;
    EXPECT_THROW(PreCorrectionNodalSource::Prepare(physical::Model(), physical::Joints(), members, exact), std::exception);
    RecordProperty("peak_source_reservation_bytes", std::to_string(forecast.peak_bytes));
}
TEST(VehicleContactSeedActual, CompleteSourceProducesExplicitPreCorrectionAndDirectInteriorCensus) {
    const auto value = PreCorrectionNodalSource::Prepare(physical::Model(), physical::Joints(), Members().Input());
    CountsAreComplete(value.counts());
    ASSERT_EQ(value.seed().node_count, 376930u);
    ASSERT_EQ(value.fields().size(), 376930u);
    EXPECT_EQ(value.counts().namespace_only_springs, 35u);
    EXPECT_EQ(value.interior().parts.size(), 14u);
    EXPECT_EQ(value.interior().original_solids, 3848u);
    EXPECT_EQ(value.interior().retained_solids, 3686u);
    std::size_t shell_incidence = 0, zero_joint_slots = 0;
    for (const auto& row : value.contributors()) {
        if (row.kind == ContributorKind::Type45) zero_joint_slots += row.slots;
        if (row.kind == ContributorKind::PentaLaw42) EXPECT_EQ(row.slots, 8u);
    }
    EXPECT_EQ(zero_joint_slots, 88u);
    for (std::size_t i = 0; i < value.seed().node_count; ++i) {
        const auto& seed = value.seed().nodes[i];
        const auto& fields = value.fields()[i];
        ASSERT_TRUE(std::isfinite(seed.volume) && std::isfinite(seed.bulk_volume) &&
            std::isfinite(seed.existing_stiffness) && std::isfinite(fields.stiffness_before_control));
        ASSERT_GE(fields.shell_incidence_count, 0);
        shell_incidence += fields.shell_incidence_count;
    }
    EXPECT_EQ(shell_incidence, value.counts().shell_occurrences);
    EXPECT_TRUE(value.physical().SharesStorage(physical::Model()));
    EXPECT_TRUE(value.joints().physical().SharesStorage(physical::Model()));
    RecordProperty("source_scope", "PreCorrection only; effective property sharing and corrected interface products unavailable");
    RecordProperty("contributor_digest", value.provenance().contributor_digest);
    Write(value);
}
TEST(VehicleContactSeedActual, MissingOrCorruptedImportMemberCannotPublishThenValidRetryRemainsPossible) {
    auto members = Members().Input();
    members.members.pop_back();
    EXPECT_THROW(PreCorrectionNodalSource::Prepare(physical::Model(), physical::Joints(), members), std::exception);
    members = Members().Input();
    std::string corrupt(members.members.front().bytes);
    corrupt[0] = corrupt[0] == '*' ? '$' : '*';
    members.members.front().bytes = corrupt;
    EXPECT_THROW(PreCorrectionNodalSource::Prepare(physical::Model(), physical::Joints(), members), std::exception);
    EXPECT_NO_THROW(PreCorrectionNodalSource::Preflight(physical::Model(), physical::Joints(), Members().Input()));
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::test
