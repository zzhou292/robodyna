#include "../Connectivity.h"
#include "../Report.h"
#include "../RunAnalysis.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>
#include <fstream>
#include <sstream>

namespace crash::analysis::impact_response::test {
namespace {

using records = output::full_shell::ParentPoints;

output::full_shell::Identity Identity() {
    return {1, 2, 3, 4, 5, 6, 1024, std::string(64, 'a'),
        std::string(64, 'b')};
}

std::vector<ParentCatalogEntry> Catalog() {
    const std::array<std::array<std::uint32_t, 4>, 5> mapped{{
        {{0, 1, 2, 2}}, {{0, 1, 2, 2}}, {{0, 1, 3, 2}},
        {{1, 2, 3, 0}}, {{0, 2, 3, 3}}}};
    const std::array<std::uint64_t, 4> source_nodes{{100, 101, 102, 103}};
    std::vector<ParentCatalogEntry> result(5);
    const std::uint64_t eids[]{500, 501, 502, 503, 504};
    const std::uint64_t pids[]{20, 21, 21, 22, 23};
    const std::uint64_t mids[]{30, 31, 31, 32, 33};
    const std::uint64_t sids[]{40, 41, 41, 42, 43};
    const std::uint32_t elforms[]{2, 2, 2, 16, 2};
    const std::uint32_t families[]{1, 1, 1, 2, 3};
    const std::uint32_t points[]{0, 1, 3, 4, 0};
    const output::full_shell::PlasticField fields[]{
        output::full_shell::PlasticField::NotApplicable,
        output::full_shell::PlasticField::NativeEquivalentPlasticStrain,
        output::full_shell::PlasticField::NativeEquivalentPlasticStrain,
        output::full_shell::PlasticField::NativeEquivalentPlasticStrain,
        output::full_shell::PlasticField::Unavailable};
    for (std::size_t i = 0; i < result.size(); ++i) {
        auto& entry = result[i];
        entry.source_element = eids[i];
        entry.source_part = pids[i];
        entry.source_material = mids[i];
        entry.source_section = sids[i];
        entry.source_elform = elforms[i];
        entry.native_family = families[i];
        entry.family_index = i;
        entry.canonical_parent = 10 + i;
        entry.native_points = points[i];
        entry.plastic = fields[i];
        entry.mapped_nodes = mapped[i];
        for (unsigned slot = 0; slot < 4; ++slot)
            entry.source_nodes[slot] = source_nodes[mapped[i][slot]];
    }
    return result;
}

output::full_shell::Context Context(const std::vector<ParentCatalogEntry>& catalog) {
    std::vector<records> parents;
    for (const auto& entry : catalog)
        parents.push_back({entry.source_element, entry.source_part,
            entry.source_elform, entry.native_family, entry.native_points,
            entry.plastic});
    return output::full_shell::Context::Create(
        Identity(), 4, parents.data(), parents.size(), .125);
}

output::full_shell::FrameStamp Stamp(std::uint64_t epoch) {
    if (!epoch) return {};
    const auto base = (epoch - 1) * .125;
    return {epoch, epoch - 1, epoch + 1, base + .125, base,
        base + .0625, epoch == 1 ? .0625 : .125};
}

output::physical_run::Sample Sample(const output::full_shell::Context& context,
    std::uint64_t epoch, std::vector<double> plastic,
    std::vector<std::uint8_t> active, double shift = 0) {
    const auto stamp = Stamp(epoch);
    std::vector<double> positions{
        shift, 0, 0, 1 + shift, 0, 0,
        shift, 1, 0, 1 + shift, 1, 0};
    output::full_shell::FrameRecord frame{
        stamp, std::move(positions), std::move(plastic)};
    output::full_shell::activity::ActivityInput input{
        context.identity(), context.point_layout_sha256(), stamp,
        active.data(), active.size()};
    auto activity = output::full_shell::activity::ActivityRecord::Create(
        context, input, stamp);
    return {std::move(frame), std::move(activity)};
}

PlasticityResult Result() {
    auto catalog = Catalog();
    const auto context = Context(catalog);
    PlasticityAccumulator analysis(context, std::move(catalog));
    analysis.Observe(Sample(context, 0, std::vector<double>(8),
        std::vector<std::uint8_t>(5, 1)));
    analysis.Observe(Sample(context, 2,
        {.1, 0, .2, 0, .3, 0, 0, 0},
        {1, 1, 1, 0, 1}, .01));
    analysis.Observe(Sample(context, 4,
        {.15, .4, .1, 0, 0, 0, 0, 0},
        {1, 1, 1, 1, 1}, .02));
    return analysis.Finish();
}

std::string Connectivity(bool wrong_shell = false) {
    std::ostringstream text;
    text << "{\"schema\":\"robo_dyna.original_physical_connectivity.v2\","
         << "\"scope\":\"Static weak incidence/potential transfer; no current activity, DOF rank or complete load-path admission\","
         << "\"archive_sha256\":\"" << std::string(64, 'c') << "\","
         << "\"member_sha256\":\"" << std::string(64, 'b') << "\","
         << "\"canonical_sha256\":\"" << std::string(64, 'a') << "\","
         << "\"tire_policy\":\"omit_original_tire_shells\","
         << "\"kind_codes\":[\"qeph\",\"t3\",\"qbat\",\"solid18\",\"solid24\",\"solid6z\",\"type13\",\"type25\",\"part_root\",\"plain_group\",\"cin\"],"
         << "\"counts\":{\"nodes\":4,\"relations\":6,\"ordered_slots\":17,"
         << "\"element_components\":2,\"potential_transfer_components\":1,"
         << "\"by_kind\":[2,0,1,1,0,0,0,0,1,0,1]},"
         << "\"nodes\":[[100,100,100,1],[101,100,100,3],"
         << "[102,100,100,5],[103,103,100,1]],\"relations\":["
         << "[0,0," << (wrong_shell ? 999 : 501)
         << ",21,0,[0,1,2]],[0,0,502,21,1,[0,1,3,2]],"
         << "[2,0,503,22,0,[1,2,3,0]],[3,0,600,30,0,[0,1]],"
         << "[8,1,40,40,0,[1,2],0],[10,2,700,31,0,[3,2]]]}";
    return text.str();
}

output::full_shell::RecordFile File(const char* name, char hash,
    std::size_t bytes = 1) {
    return {name, std::string(64, hash), bytes};
}

RunMetadata Metadata() {
    RunMetadata metadata;
    metadata.identity = Identity();
    metadata.source_units = {"t", "mm", "s", 1000, .001, 1};
    metadata.viewer_input = File("viewer-input.json", 'd');
    metadata.archive_manifest = File("manifest.json", 'e');
    metadata.source_canonical_manifest = File("manifest.json", 'a');
    metadata.source_scope_report = File("scope.json", 'f');
    metadata.source_member = File("source.key", 'b');
    metadata.run_summary = File("run-summary.json", '1');
    metadata.connectivity_report = File("connectivity.json", '2');
    metadata.tire_policy = "omit_original_tire_shells";
    metadata.mapping_sha256 = std::string(64, 'b');
    metadata.source_part_titles = {
        {20, "not_applicable"}, {21, "yielded_qeph"},
        {22, "yielded_qbat"}, {23, "unavailable"}};
    metadata.mapped_nodes = 4;
    metadata.mapped_parents = 5;
    metadata.mapped_triangles = 8;
    metadata.stored_native_points = 8;
    metadata.planned_intervals = 10;
    metadata.accepted_intervals = 4;
    metadata.stop_reason = "diagnostic";
    metadata.fixed_dt_s = .125;
    return metadata;
}

std::string SummaryBytes(const RunMetadata& metadata,
    const PlasticityResult& result, std::size_t final_positive_points) {
    const auto& final = result.samples.back();
    std::ostringstream text;
    text << "{\"schema\":\"robo_dyna.vehicle_run_summary.v1\","
         << "\"status\":\"diagnostic_interval_limit\","
         << "\"reason\":\"diagnostic\",\"physical_profile\":\"fixture\","
         << "\"valid_archive_manifest\":true,"
         << "\"accepted_intervals\":" << metadata.accepted_intervals << ','
         << "\"actual_completed_time_s\":" << final.stamp.time_s << ','
         << "\"fixed_dt_s\":" << metadata.fixed_dt_s << ','
         << "\"sampled_shell_plasticity\":{"
         << "\"schema\":\"robo_dyna.sampled_shell_plasticity.v1\","
         << "\"available\":true,\"saved_samples\":" << result.samples.size()
         << ",\"last_saved_epoch\":" << final.stamp.epoch
         << ",\"last_saved_attempt\":" << final.stamp.attempt
         << ",\"last_saved_time_s\":" << final.stamp.time_s
         << ",\"native_fields_available\":true,\"stored_native_points\":"
         << result.native_points << ",\"last_saved_positive_points\":"
         << final_positive_points
         << ",\"positive_saved_sample_observed\":true,"
         << "\"last_saved_max_native_equivalent_plastic_strain\":"
         << final.maximum.value
         << ",\"peak_saved_native_equivalent_plastic_strain\":"
         << result.peak.value << ",\"first_positive_saved_epoch\":"
         << result.first_positive.occurrence.stamp.epoch
         << ",\"first_positive_saved_time_s\":"
         << result.first_positive.occurrence.stamp.time_s << "},"
         << "\"last_accepted_active_parents\":" << final.active_parents << ','
         << "\"contact_observations_available\":true,"
         << "\"peak_observed_force_n\":1,\"peak_observed_penetration_m\":0.1,"
         << "\"peak_observed_same_mask_potential_j\":2,"
         << "\"last_same_mask_potential_j\":1,"
         << "\"last_removed_potential_j\":0,"
         << "\"archive_manifest_sha256\":\""
         << metadata.archive_manifest.sha256 << "\","
         << "\"viewer_input_sha256\":\"" << metadata.viewer_input.sha256
         << "\",\"accepted_mechanics\":{\"available\":true,"
         << "\"solids\":{\"included_law36_law44_plastic_work\":"
         << "{\"accepted_increment_sum_j\":0}},"
         << "\"beam18\":{\"included_plastic_work\":"
         << "{\"accepted_increment_sum_j\":0}},"
         << "\"motion\":{\"translation_departure_m\":{\"last\":0,\"peak\":0},"
         << "\"velocity_departure_m_s\":{\"last\":0,\"peak\":0},"
         << "\"orientation_component_departure\":{\"last\":0,\"peak\":0},"
         << "\"spin_component_rad_s\":{\"last\":0,\"peak\":0}}}}";
    return text.str();
}

}  // namespace

TEST(ImpactResponseAnalysis,
    VariablePointLayoutsActivityAndSampledOnsetRemainDistinct) {
    const auto result = Result();
    ASSERT_EQ(result.samples.size(), 3u);
    EXPECT_EQ(result.native_point_layouts,
        (std::array<std::size_t, 5>{2, 1, 1, 1, 0}));
    EXPECT_EQ(result.available_parents, 3u);
    EXPECT_EQ(result.not_applicable_parents, 1u);
    EXPECT_EQ(result.unavailable_parents, 1u);
    EXPECT_EQ(result.native_points, 8u);
    EXPECT_EQ(result.ever_positive_points, 4u);
    EXPECT_EQ(result.ever_positive_parents, 3u);
    EXPECT_EQ(result.ever_positive_parts, 2u);
    EXPECT_TRUE(result.inactive_positive_history_observed);
    ASSERT_TRUE(result.first_positive.available);
    EXPECT_EQ(result.catalog[result.first_positive.parent_index].source_element,
        501u);
    EXPECT_EQ(result.first_positive.occurrence.stamp.epoch, 2u);
    ASSERT_TRUE(result.peak.available);
    EXPECT_EQ(result.catalog[result.peak.parent_index].source_element, 502u);
    EXPECT_EQ(result.peak.value, .4);
    EXPECT_EQ(result.samples[1].inactive_positive_points, 1u);
    EXPECT_EQ(result.samples.back().positive_points, 3u);
    EXPECT_EQ(result.samples.back().positive_parts, 1u);
    EXPECT_TRUE(result.parents[3].ever_inactive);
    EXPECT_TRUE(result.parents[3].active_at_final_sample);
    EXPECT_EQ(result.parents[3].final_positive_points, 0u);
    const auto part = std::find_if(result.parts.begin(), result.parts.end(),
        [](const PartPlasticity& value) { return value.source_part == 21; });
    ASSERT_NE(part, result.parts.end());
    EXPECT_EQ(part->ever_positive_parents, 2u);
    EXPECT_EQ(part->ever_positive_points, 3u);
    EXPECT_EQ(part->final_positive_points, 3u);
    EXPECT_EQ(part->peak_strain, .4);
}

TEST(ImpactResponseAnalysis,
    StreamingConnectivityBindsYieldedParentsAndDirectIncidentKinds) {
    const auto plasticity = Result();
    const ConnectivitySourceAuthority authority{
        std::string(64, 'a'), std::string(64, 'b'),
        "omit_original_tire_shells"};
    const auto evidence = AnalyzeConnectivity(
        Connectivity(), plasticity, authority);
    EXPECT_EQ(evidence.nodes, 4u);
    EXPECT_EQ(evidence.relations, 6u);
    EXPECT_EQ(evidence.potential_transfer_components, 1u);
    EXPECT_EQ(evidence.yielded_source_nodes, 4u);
    EXPECT_EQ(evidence.incident_relations, 6u);
    EXPECT_EQ(evidence.yielded_transfer_component_labels,
        (std::vector<std::uint64_t>{100}));
    ASSERT_EQ(evidence.parents.size(), 3u);
    for (const auto& parent : evidence.parents) {
        EXPECT_EQ(parent.matched_constitutive_relations, 1u);
        EXPECT_FALSE(parent.incident_relations_by_kind.empty());
    }
    EXPECT_THROW(AnalyzeConnectivity(
        Connectivity(true), plasticity, authority), std::exception);
}

TEST(ImpactResponseAnalysis,
    RunSummaryCompanionMustMatchEverySampledShellCrossCheck) {
    const auto result = Result();
    auto metadata = Metadata();
    const auto bytes = SummaryBytes(
        metadata, result, result.samples.back().positive_points);
    metadata.run_summary = {
        "run-summary.json", output::Sha256(bytes), bytes.size()};
    const auto summary = VerifyRunSummary(bytes, metadata, result);
    EXPECT_EQ(summary.peak_wall_force_n, 1);
    EXPECT_EQ(summary.actual_completed_time_s,
        result.samples.back().stamp.time_s);

    auto wrong_stop = metadata;
    wrong_stop.stop_reason = "different diagnostic";
    EXPECT_THROW(VerifyRunSummary(bytes, wrong_stop, result), std::exception);

    const auto changed = SummaryBytes(
        metadata, result, result.samples.back().positive_points + 1);
    metadata.run_summary = {
        "run-summary.json", output::Sha256(changed), changed.size()};
    EXPECT_THROW(VerifyRunSummary(changed, metadata, result), std::exception);
}

TEST(ImpactResponseAnalysis,
    ReportKeepsSampledAndStaticScopesAndPublishesCreateOnlyOutsideRun) {
    const auto plasticity = Result();
    const ConnectivitySourceAuthority authority{
        std::string(64, 'a'), std::string(64, 'b'),
        "omit_original_tire_shells"};
    const auto connectivity = AnalyzeConnectivity(
        Connectivity(), plasticity, authority);
    SummaryEvidence summary;
    summary.schema = "robo_dyna.vehicle_run_summary.v1";
    summary.status = "diagnostic_interval_limit";
    summary.reason = "diagnostic";
    summary.physical_profile = "fixture";
    summary.actual_completed_time_s = plasticity.samples.back().stamp.time_s;
    const auto document = BuildReport(
        Metadata(), summary, plasticity, connectivity);
    EXPECT_STREQ(document["schema"].GetString(),
        "robo_dyna.impact_response_analysis.v1");
    EXPECT_EQ(document["yielded_parents"].Size(), 3u);
    EXPECT_EQ(document["parts"].Size(), 4u);
    EXPECT_EQ(document["static_connectivity"]
        ["potential_transfer_components"].GetUint64(), 1u);

    output::full_shell::test::Directory directory;
    std::filesystem::create_directory(directory.path / "run");
    std::filesystem::create_directory(directory.path / "reports");
    {
        std::ofstream viewer(directory.path / "run" / "viewer-input.json");
        viewer << "{}\n";
    }
    const auto report = directory.path / "reports" / "analysis.json";
    EXPECT_NO_THROW(WriteReport(
        directory.path / "run" / "viewer-input.json", report, document));
    EXPECT_TRUE(std::filesystem::is_regular_file(report));
    EXPECT_THROW(WriteReport(
        directory.path / "run" / "viewer-input.json", report, document),
        std::exception);
    EXPECT_THROW(WriteReport(
        directory.path / "run" / "viewer-input.json",
        directory.path / "run" / "inside.json", document), std::exception);
}

}  // namespace crash::analysis::impact_response::test
